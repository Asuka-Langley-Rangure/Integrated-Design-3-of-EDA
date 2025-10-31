// myplace.cpp
// -------------------------------------------------------------
// Initial placement (quadratic wirelength) + progress using LiveCounter
// Key fixes: exclude FILLER, soft-anchor to chip center, jitter init
// -------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

#include "common.h"
#include "objects.h"
#include "placedata.h"
#include "myplace.h"

// =========================== helpers ===========================

static inline double dclamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

// ======================== initialPlacement ======================

void MyPlacer::initialPlacement()
{
    // ---------------- 0) 收集未知量（排除 fixed / filler） ----------------
    std::vector<Module*> movableMods;
    std::unordered_map<Module*, int> modToIndex;
    movableMods.reserve(db->Nodes.size());

    for (Module* mod : db->Nodes) {
        if (!mod) continue;
        if (mod->isFixed || mod->isFiller) continue; // 仍然排除 FILLER
        modToIndex[mod] = static_cast<int>(movableMods.size());
        movableMods.push_back(mod);
    }
    const int N = static_cast<int>(movableMods.size());
    if (N == 0) {
        std::cout << "No movable modules (after excluding fixed/filler). Skip initial placement.\n";
        return;
    }

    auto tick = [&](const char* stage, int pct) {
        std::cout << "\r[" << std::setw(3) << pct << "%] " << stage << std::flush;
    };
    tick("Collect movable modules", 5);

    // ---- 芯片几何（用于锚点 / 边界势 / 初值）----
    auto [ll_site, ur_site] = getSiteRowBoundingBox();
    const double dieLx = (double)ll_site.x, dieLy = (double)ll_site.y;
    const double dieUx = (double)ur_site.x, dieUy = (double)ur_site.y;
    const double chipW = std::max(1e-6, dieUx - dieLx);
    const double chipH = std::max(1e-6, dieUy - dieLy);
    const double chip_cx = 0.5 * (dieLx + dieUx);
    const double chip_cy = 0.5 * (dieLy + dieUy);

    // ---------------- 0.1) 初始化可动单元中心：芯片中心 + 轻微抖动 ----------------
    std::mt19937 rng(1234567);
    std::normal_distribution<double> n01(0.0, 1.0);
    const double jitter_x = 0.01 * chipW; // 1% die width
    const double jitter_y = 0.01 * chipH; // 1% die height
    for (Module* m : movableMods) {
        if (std::abs(m->center.x) < 1e-9 && std::abs(m->center.y) < 1e-9) {
            m->center.x = (float)(chip_cx + n01(rng) * jitter_x);
            m->center.y = (float)(chip_cy + n01(rng) * jitter_y);
        }
        // 若初值明显落在芯片外，也拉回芯片框内（避免极端初值）
        m->center.x = (float)std::clamp((double)m->center.x, dieLx, dieUx);
        m->center.y = (float)std::clamp((double)m->center.y, dieLy, dieUy);
    }

    // ---------------- 1) Triplets & RHS ----------------
    std::vector<Eigen::Triplet<double>> tripX, tripY;
    tripX.reserve(N * 20);
    tripY.reserve(N * 20);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(N);

    // 参数
    constexpr int    K_CLIQUE_MAX  = 8;     // 小网阈值
    constexpr int    K_SAMPLE      = 4;     // 全可动大网退化阈值
    constexpr double LAMBDA_DIAG   = 1e-6;  // 数值稳性
    constexpr double MU_EDGE_ANCH  = 5e-5;  // 芯片边界软势（轻）
    constexpr double MU_COMP_ANCH  = 2e-4;  // 连通分量级软锚（中）
    constexpr double MU_SOFT_ANCH  = 0.0;   // 全局中心软锚（置 0；有组件锚时不需要）

    auto add_diag = [&](int idx, double w){
        tripX.emplace_back(idx, idx, w);
        tripY.emplace_back(idx, idx, w);
    };
    auto add_edge = [&](int i, int j, double w){
        // 拉普拉斯装配
        tripX.emplace_back(i, i,  w);
        tripX.emplace_back(j, j,  w);
        tripX.emplace_back(i, j, -w);
        tripX.emplace_back(j, i, -w);
        tripY.emplace_back(i, i,  w);
        tripY.emplace_back(j, j,  w);
        tripY.emplace_back(i, j, -w);
        tripY.emplace_back(j, i, -w);
    };

    // ---------------- 2) 进度计数 ----------------
    size_t totalPins = 0;
    for (Net* net : db->Nets) if (net) totalPins += net->netPins.size();
    LiveCounter lc;
    lc.init(totalPins, /*step_gate=*/256, /*ms_gate=*/0);

    // ---------------- 2.1) 并查集：记录可动图的连通性 ----------------
    // 只对可动模块做 union（有边相连即合并）
    struct DSU {
        std::vector<int> p, r;
        explicit DSU(int n=0): p(n), r(n,0){ std::iota(p.begin(), p.end(), 0); }
        int find(int x){ return p[x]==x?x:p[x]=find(p[x]); }
        void unite(int a,int b){
            a=find(a); b=find(b); if(a==b) return;
            if(r[a]<r[b]) std::swap(a,b);
            p[b]=a; if(r[a]==r[b]) r[a]++;
        }
    } dsu(N);

    std::vector<char> compHasFixed; compHasFixed.assign(N, 0);

    // ---------------- 3) 按 net 装配 ----------------
    tick("Build net model & weights", 25);
    for (Net* net : db->Nets) {
        if (!net) continue;
        auto& pins = net->netPins;
        const int k = static_cast<int>(pins.size());
        lc.tick(k);
        if (k < 2) continue;

        // 分类：可动 / 固定
        std::vector<int>    movIdx; movIdx.reserve(k);
        std::vector<double> movDx;  movDx.reserve(k);
        std::vector<double> movDy;  movDy.reserve(k);

        std::vector<double> fixX;   fixX.reserve(k);
        std::vector<double> fixY;   fixY.reserve(k);
        std::vector<double> fixDx;  fixDx.reserve(k);
        std::vector<double> fixDy;  fixDy.reserve(k);

        bool hasFixedOnThisNet = false;

        for (int t = 0; t < k; ++t) {
            Pin* p = pins[t];
            if (!p) continue;
            Module* m = p->module;

            if (m == nullptr) {
                // 如果有 IO 绝对坐标字段，可在此纳入固定锚：
                // fixX.push_back(p->abs.x); fixY.push_back(p->abs.y);
                // fixDx.push_back(0.0); fixDy.push_back(0.0);
                // hasFixedOnThisNet = true;
                continue;
            }

            if (m->isFixed) {
                fixX.push_back((double)m->center.x);
                fixY.push_back((double)m->center.y);
                fixDx.push_back((double)p->offset.x);
                fixDy.push_back((double)p->offset.y);
                hasFixedOnThisNet = true;
            } else {
                auto it = modToIndex.find(m);
                if (it == modToIndex.end()) continue;
                movIdx.push_back(it->second);
                movDx.push_back((double)p->offset.x);
                movDy.push_back((double)p->offset.y);
            }
        }

        const int km = (int)movIdx.size();
        const int kf = (int)fixX.size();
        if (km == 0) continue;

        // 并查集合并（可动-可动同网即连接）
        for (int a = 1; a < km; ++a) dsu.unite(movIdx[0], movIdx[a]);
        if (hasFixedOnThisNet) {
            // 标记：该网的可动节点连通分量含固定端
            for (int a = 0; a < km; ++a) compHasFixed[ dsu.find(movIdx[a]) ] = 1;
        }

        // 避免大网权重过重
        const double w = 1.0 / (double)std::max(1, k - 1);

        if (k <= K_CLIQUE_MAX) {
            // 小网：clique
            for (int a = 0; a < km; ++a) {
                const int ia = movIdx[a];
                for (int b = a + 1; b < km; ++b) {
                    const int ib = movIdx[b];
                    add_edge(ia, ib, w);
                    const double ddx = movDx[a] - movDx[b];
                    const double ddy = movDy[a] - movDy[b];
                    b_x[ia] +=  w * ddx;  b_x[ib] -=  w * ddx;
                    b_y[ia] +=  w * ddy;  b_y[ib] -=  w * ddy;
                }
                for (int f = 0; f < kf; ++f) {
                    add_diag(ia, w);
                    b_x[ia] += w * ((fixX[f] + fixDx[f]) - movDx[a]);
                    b_y[ia] += w * ((fixY[f] + fixDy[f]) - movDy[a]);
                }
            }
        } else {
            // 大网：star / 质心锚
            double rx = 0.0, ry = 0.0; int cntR = 0;
            for (int f = 0; f < kf; ++f) {
                rx += (fixX[f] + fixDx[f]); ry += (fixY[f] + fixDy[f]); ++cntR;
            }
            if (cntR == 0) {
                // 用当前可动中心 + offset 的平均
                double sx = 0.0, sy = 0.0;
                for (int a = 0; a < km; ++a) {
                    Module* mm = movableMods[movIdx[a]];
                    sx += (double)mm->center.x + movDx[a];
                    sy += (double)mm->center.y + movDy[a];
                }
                rx = sx / std::max(1, km);
                ry = sy / std::max(1, km);
                cntR = 1;
            }
            rx /= cntR;  ry /= cntR;

            for (int a = 0; a < km; ++a) {
                const int ia = movIdx[a];
                add_diag(ia, w);
                b_x[ia] += w * (rx - movDx[a]);
                b_y[ia] += w * (ry - movDy[a]);
            }

            if (kf > 0) {
                const double wf = w / (double)kf;
                for (int a = 0; a < km; ++a) {
                    const int ia = movIdx[a];
                    for (int f = 0; f < kf; ++f) {
                        add_diag(ia, wf);
                        b_x[ia] += wf * ((fixX[f] + fixDx[f]) - movDx[a]);
                        b_y[ia] += wf * ((fixY[f] + fixDy[f]) - movDy[a]);
                    }
                }
            } else if (km <= K_SAMPLE) {
                // 全可动且很小：退化为小 clique
                for (int a = 0; a < km; ++a) {
                    const int ia = movIdx[a];
                    for (int b = a + 1; b < km; ++b) {
                        const int ib = movIdx[b];
                        add_edge(ia, ib, w);
                        const double ddx = movDx[a] - movDx[b];
                        const double ddy = movDy[a] - movDy[b];
                        b_x[ia] +=  w * ddx;  b_x[ib] -=  w * ddx;
                        b_y[ia] +=  w * ddy;  b_y[ib] -=  w * ddy;
                    }
                }
            } else {
                // 大全可动网：仅轻正则，等密度/平滑阶段再拉拢
                for (int a = 0; a < km; ++a) add_diag(movIdx[a], LAMBDA_DIAG);
            }
        }
    }
    lc.finish();

    // ---------------- 4) 连通分量级软锚（仅对“无固定端”的分量） ----------------
    tick("Component anchors", 70);
    // 统计各根代表下的成员
    std::unordered_map<int, std::vector<int>> compMembers;
    compMembers.reserve(N);
    for (int i = 0; i < N; ++i) {
        int r = dsu.find(i);
        compMembers[r].push_back(i);
    }

    // 为“没有固定端”的分量分配不同的目标点：sunflower pattern
    // 让它们分散在芯片内，而非全到同一点
    int freeCompCount = 0;
    for (auto &kv : compMembers) if (!compHasFixed[kv.first]) ++freeCompCount;

    int idxComp = 0;
    for (auto &kv : compMembers) {
        int root = kv.first;
        auto &members = kv.second;
        if (compHasFixed[root]) continue; // 有固定端的分量无需额外锚

        // sunflower：极坐标均匀
        double t = (freeCompCount > 1) ? (idxComp / (double)(freeCompCount)) : 0.0;
        double theta = 2.39996322972865332 * idxComp; // ~ golden angle
        double rad = 0.35 * std::min(chipW, chipH) * std::sqrt(t); // 由小到大分散
        double anchor_x = chip_cx + rad * std::cos(theta);
        double anchor_y = chip_cy + rad * std::sin(theta);

        // 限制在芯片内
        anchor_x = std::clamp(anchor_x, dieLx + 0.05*chipW, dieUx - 0.05*chipW);
        anchor_y = std::clamp(anchor_y, dieLy + 0.05*chipH, dieUy - 0.05*chipH);

        // 对该分量的每个点加：A(ii)+=μc, b(i)+=μc*anchor
        for (int u : members) {
            add_diag(u, MU_COMP_ANCH);
            b_x[u] += MU_COMP_ANCH * anchor_x;
            b_y[u] += MU_COMP_ANCH * anchor_y;
        }
        ++idxComp;
    }

    // ---------------- 5) 芯片边界软势 + 数值稳性对角 ----------------
    tick("Boundary wells & regularization", 80);
    for (int i = 0; i < N; ++i) {
        // 数值稳性（极小对角）
        add_diag(i, LAMBDA_DIAG);

        // 边界软势：向芯片内“回弹”的轻微二次势（中心点作为目标）
        add_diag(i, MU_EDGE_ANCH);
        b_x[i] += MU_EDGE_ANCH * chip_cx;
        b_y[i] += MU_EDGE_ANCH * chip_cy;
    }

    // 可选：全局中心软锚（默认 0）；
    if (MU_SOFT_ANCH > 0.0) {
        for (int i = 0; i < N; ++i) {
            add_diag(i, MU_SOFT_ANCH);
            b_x[i] += MU_SOFT_ANCH * chip_cx;
            b_y[i] += MU_SOFT_ANCH * chip_cy;
        }
    }

    // ---------------- 6) 构建稀疏矩阵并求解 ----------------
    tick("Assemble A (sparse)", 85);
    Eigen::SparseMatrix<double> A_x(N, N), A_y(N, N);
    A_x.setFromTriplets(tripX.begin(), tripX.end());
    A_y.setFromTriplets(tripY.begin(), tripY.end());
    A_x.makeCompressed();
    A_y.makeCompressed();

    tick("Solve Ax=bx / Ay=by", 95);
    // 若 IncompleteLUT 不可用，可替换为 DiagonalPreconditioner
    Eigen::BiCGSTAB<Eigen::SparseMatrix<double>, Eigen::IncompleteLUT<double>> solver_x, solver_y;
    solver_x.setTolerance(1e-6); solver_y.setTolerance(1e-6);
    solver_x.setMaxIterations(5000); solver_y.setMaxIterations(5000);

    solver_x.compute(A_x);
    solver_y.compute(A_y);
    if (solver_x.info() != Eigen::Success || solver_y.info() != Eigen::Success) {
        std::cerr << "InitialPlacement: factorization failed.\n";
        return;
    }

    Eigen::VectorXd X = solver_x.solve(b_x);
    Eigen::VectorXd Y = solver_y.solve(b_y);
    if (solver_x.info() != Eigen::Success || solver_y.info() != Eigen::Success) {
        std::cerr << "InitialPlacement: solve failed.\n";
        return;
    }

    // ---------------- 7) 写回坐标（并夹在芯片框内，防止极端） ----------------
    for (int i = 0; i < N; ++i) {
        double xi = std::clamp((double)X[i], dieLx, dieUx);
        double yi = std::clamp((double)Y[i], dieLy, dieUy);
        movableMods[i]->center.x = (float)xi;
        movableMods[i]->center.y = (float)yi;
    }

    std::cout << "\nInitial placement completed.\n";
}


// ======================== getSiteRowBoundingBox ========================

std::pair<POS_2D, POS_2D> MyPlacer::getSiteRowBoundingBox() {
    if (db->SiteRows.empty()) return {{0,0},{1000,1000}}; // 兜底

    double min_x =  std::numeric_limits<double>::infinity();
    double min_y =  std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (const auto& row : db->SiteRows) {
        const double sx = (double)row.start.x;
        const double ex = (double)row.end.x;
        const double by = (double)row.bottom;
        const double ty = (double)row.bottom + (double)row.height;

        min_x = std::min(min_x, sx);
        max_x = std::max(max_x, ex);
        min_y = std::min(min_y, by);
        max_y = std::max(max_y, ty);
    }
    if (!std::isfinite(min_x)) return {{0,0},{1000,1000}};
    return {{(float)min_x, (float)min_y}, {(float)max_x, (float)max_y}};
}

// ============================= initializeBins ==========================
// 不影响初始解；仅用于后续密度/电势
void MyPlacer::initializeBins(double targetDensity) {
    auto [ll_site, ur_site] = getSiteRowBoundingBox();
    const double dieW = (double)ur_site.x - (double)ll_site.x;
    const double dieH = (double)ur_site.y - (double)ll_site.y;
    if (dieW <= 0.0 || dieH <= 0.0) {
        std::cerr << "[initializeBins] invalid die bbox: W=" << dieW << " H=" << dieH << "\n";
        return;
    }

    const double binTargetH = std::max(1.0, (double)db->siteHeight * 32.0);
    int rows = (int)std::ceil(dieH / binTargetH);
    rows = std::clamp(rows, 8, 256);

    const double binH = dieH / (double)rows;
    const double binW_target = binH * (dieW / dieH);
    int cols = (int)std::ceil(dieW / binW_target);
    cols = std::clamp(cols, 8, 256);

    bins.assign(rows, std::vector<Bin_2D*>(cols, nullptr));
    const double bw = dieW / (double)cols;
    const double bh = dieH / (double)rows;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (!bins[i][j]) bins[i][j] = new Bin_2D();

            Bin_2D* b = bins[i][j];
            b->ll.x = (float)((double)ll_site.x + j * bw);
            b->ll.y = (float)((double)ll_site.y + i * bh);
            b->ur.x = (float)((double)b->ll.x + bw);
            b->ur.y = (float)((double)b->ll.y + bh);
            b->center.x = (float)(((double)b->ll.x + (double)b->ur.x) * 0.5);
            b->center.y = (float)(((double)b->ll.y + (double)b->ur.y) * 0.5);
            b->width  = (float)bw;
            b->height = (float)bh;
            b->area   = (float)(bw * bh);

            b->nodeDensity     = 0.0f;
            b->fillerDensity   = 0.0f;
            b->terminalDensity = 0.0f;
            b->DarkDensity     = 0.0f;
            b->phi             = 0.0f;
        }
    }

    // 仅统计固定模块（终端/宏）密度
    for (Module* mod : db->Nodes) {
        if (!mod || !mod->isFixed) continue;

        const double mllx = (double)mod->center.x - (double)mod->width  * 0.5;
        const double mlly = (double)mod->center.y - (double)mod->height * 0.5;
        const double murx = (double)mod->center.x + (double)mod->width  * 0.5;
        const double mury = (double)mod->center.y + (double)mod->height * 0.5;

        int j0 = std::clamp((int)std::floor((mllx - (double)ll_site.x) / bw), 0, cols - 1);
        int j1 = std::clamp((int)std::floor((murx - (double)ll_site.x) / bw), 0, cols - 1);
        int i0 = std::clamp((int)std::floor((mlly - (double)ll_site.y) / bh), 0, rows - 1);
        int i1 = std::clamp((int)std::floor((mury - (double)ll_site.y) / bh), 0, rows - 1);

        for (int i = i0; i <= i1; ++i) {
            for (int j = j0; j <= j1; ++j) {
                Bin_2D* bin = bins[i][j];

                const double bllx = (double)bin->ll.x;
                const double blly = (double)bin->ll.y;
                const double burx = (double)bin->ur.x;
                const double bury = (double)bin->ur.y;

                const double ox = std::max(0.0, std::min(murx, burx) - std::max(mllx, bllx));
                const double oy = std::max(0.0, std::min(mury, bury) - std::max(mlly, blly));
                const double overlap = ox * oy;
                if (overlap <= 0.0) continue;

                const double binArea = (double)bin->width * (double)bin->height;
                if (binArea <= 0.0) continue;

                const double occ = dclamp(overlap / binArea, 0.0, 1.0);
                bin->terminalDensity = (float)dclamp((double)bin->terminalDensity + occ, 0.0, 1.0);
            }
        }
    }

    std::cout << "Bin grid initialized: " << rows << " x " << cols << std::endl;
}

// ============================= createfillerCells =======================
// 仅生成占位，不参与初始解；高度保证 >= 1
void MyPlacer::createfillerCells() {
    if (db->SiteRows.empty()) {
        std::cout << "No site rows. Skip filler creation.\n";
        return;
    }

    double dieLx =  std::numeric_limits<double>::infinity();
    double dieLy =  std::numeric_limits<double>::infinity();
    double dieUx = -std::numeric_limits<double>::infinity();
    double dieUy = -std::numeric_limits<double>::infinity();
    double rowArea = 0.0;

    for (const auto& row : db->SiteRows) {
        const double sx = (double)row.start.x;
        const double ex = (double)row.end.x;
        const double by = (double)row.bottom;
        const double ty = (double)row.bottom + (double)row.height;

        dieLx = std::min(dieLx, sx);
        dieLy = std::min(dieLy, by);
        dieUx = std::max(dieUx, ex);
        dieUy = std::max(dieUy, ty);

        rowArea += (ex - sx) * (double)row.height;
    }

    double fixedArea = 0.0;
    std::vector<double> movableAreas; movableAreas.reserve(db->Nodes.size());
    double movableTotalArea = 0.0;

    for (Module* mod : db->Nodes) {
        if (!mod) continue;
        if (mod->isFixed) {
            fixedArea += (double)mod->area;
        } else if (!mod->isMacro && !mod->isFiller) {
            const double a = (double)mod->area;
            movableAreas.push_back(a);
            movableTotalArea += a;
        }
    }

    if (movableAreas.empty()) {
        std::cout << "No movable std cells. Skip filler creation.\n";
        return;
    }

    std::sort(movableAreas.begin(), movableAreas.end());
    const int n = (int)movableAreas.size();
    int s = (int)std::floor(n * 0.05), e = (int)std::ceil(n * 0.95);
    s = std::clamp(s, 0, n); e = std::clamp(e, 0, n);
    if (s >= e) { s = 0; e = n; }

    double sum = 0.0; for (int i = s; i < e; ++i) sum += movableAreas[i];
    const double avgMovableArea = (e > s) ? (sum / (e - s))
                                          : (movableTotalArea / std::max(1, n));

    const double targetDensity = 0.80;
    const double availableArea  = std::max(0.0, rowArea - fixedArea);
    const double requiredMovableArea = targetDensity * availableArea;

    double placedLikeArea = 0.0;
    for (Module* mod : db->Nodes) {
        if (!mod || mod->isFixed) continue;
        const double a = (double)mod->area;
        placedLikeArea += mod->isMacro ? (a * targetDensity) : a;
    }

    double fillerArea = requiredMovableArea - placedLikeArea;
    if (fillerArea <= 0.0) {
        std::cout << "No filler needed (density already satisfied)." << std::endl;
        return;
    }

    const double fillerHeight = std::max(1.0, (double)db->siteHeight); // ★ 防 0 高度
    double fillerWidth = (fillerHeight > 0.0) ? (avgMovableArea / fillerHeight) : 1.0;
    if (fillerWidth <= 0.0) fillerWidth = 1.0;

    const double fillerCellArea = fillerWidth * fillerHeight;
    int numFillers = (int)std::floor(fillerArea / std::max(1.0, fillerCellArea));
    numFillers = std::max(0, numFillers);

    if (numFillers == 0) {
        std::cout << "Computed filler count is zero. Skip." << std::endl;
        return;
    }

    std::cout << "Creating " << numFillers << " filler cells..." << std::endl;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(dieLx, dieUx);
    std::uniform_real_distribution<> dis_y(dieLy, dieUy);

    for (int i = 0; i < numFillers; ++i) {
        Module* filler = new Module();
        filler->name   = "FILLER_" + std::to_string(i);
        filler->width  = (float)fillerWidth;
        filler->height = (float)fillerHeight;
        filler->area   = (float)(fillerWidth * fillerHeight);
        filler->isFixed  = false;
        filler->isFiller = true;
        filler->isMacro  = false;

        filler->center.x = (float)dis_x(gen);
        filler->center.y = (float)dis_y(gen);

        db->Nodes.push_back(filler);
    }

    std::cout << "Added " << numFillers << " filler cells (each "
              << fillerWidth << " x " << fillerHeight << ")." << std::endl;
}
