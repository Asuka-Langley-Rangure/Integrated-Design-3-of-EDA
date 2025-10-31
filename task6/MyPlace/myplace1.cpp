// myplace.cpp
#include "myplace.h"

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <mutex>
#include <numeric>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

// ------------------------ 轻量进度条（线程安全、低刷新） ------------------------
class ProgressBar {
public:
    void init(std::size_t total, std::size_t step_gate = 512, int min_refresh_ms = 150) {
        total_ = total ? total : 1;
        gate_  = step_gate ? step_gate : 512;
        done_.store(0);
        start_ = last_ = std::chrono::steady_clock::now();
        last_done_ = 0;
        min_refresh_ms_ = std::max(0, min_refresh_ms);
    }
    inline void tick(std::size_t delta) {
        std::size_t v = done_.fetch_add(delta) + delta;
        if (v - last_done_ < gate_) return;

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_).count() < min_refresh_ms_) return;

        std::lock_guard<std::mutex> lk(mu_);
        last_ = now;
        last_done_ = v;

        double pct = 100.0 * static_cast<double>(v) / static_cast<double>(total_);
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_).count();
        double rate = v ? static_cast<double>(elapsed) / static_cast<double>(v) : 0.0; // sec per unit
        double eta  = rate * static_cast<double>(total_ > v ? (total_ - v) : 0);

        const int barW = 40;
        int fill = static_cast<int>(barW * pct / 100.0);
        std::cout << "\r[";
        for (int i = 0; i < barW; ++i) std::cout << (i < fill ? '=' : ' ');
        std::cout << "] " << static_cast<int>(pct) << "%  ETA: " << static_cast<int>(eta) << "s   ";
        std::cout.flush();
    }
    inline void finish() {
        tick(total_ - done_.load());
        std::cout << "\r[========================================] 100%  ETA: 0s   \n";
    }
private:
    std::atomic<std::size_t> done_{0};
    std::size_t total_ = 1, gate_ = 512, last_done_ = 0;
    int min_refresh_ms_ = 150;
    std::chrono::steady_clock::time_point start_, last_;
    std::mutex mu_;
};

// ======================================================================
//                       Initial Placement（修订版）
// - 大网避免 clique：可动↔固定锚点；全可动用“几何重心锚”
// - Triplet 一次性装配；充足 reserve；全局正则避免奇异
// - 进度条按 pin 数降频显示
// ======================================================================
void MyPlacer::initialPlacement()
{
    // 0) 收集可移动模块并建索引
    std::vector<Module*> movableMods;
    std::unordered_map<Module*, int> modToIndex;
    movableMods.reserve(db->Nodes.size());

    for (Module* mod : db->Nodes) {
        if (!mod || mod->isFixed) continue;
        modToIndex[mod] = static_cast<int>(movableMods.size());
        movableMods.push_back(mod);
    }
    const int N = static_cast<int>(movableMods.size());
    if (N == 0) {
        std::cout << "No movable modules. Skip initial placement.\n";
        return;
    }

    // 1) Triplet & RHS
    std::vector<Eigen::Triplet<double>> tripX, tripY;
    tripX.reserve(N * 32);
    tripY.reserve(N * 32);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(N);

    // 参数：小网 clique 阈值 / 锚点截断 / 正则
    constexpr int    K_CLIQUE_MAX  = 12;   // 小网阈值（可 8~16 调整）
    constexpr int    TOP_K_FIX     = 8;    // 每个可动 pin 仅连最近的固定 pin 个数
    constexpr double LAMBDA_DIAG   = 1e-6; // 全局小正则

    auto add_diag = [&](int idx, double w){
        tripX.emplace_back(idx, idx, w);
        tripY.emplace_back(idx, idx, w);
    };
    auto add_edge = [&](int i, int j, double w){
        // 拉普拉斯：L(ii)+=w, L(jj)+=w, L(ij)-=w, L(ji)-=w
        tripX.emplace_back(i, i,  w);
        tripX.emplace_back(j, j,  w);
        tripX.emplace_back(i, j, -w);
        tripX.emplace_back(j, i, -w);
        tripY.emplace_back(i, i,  w);
        tripY.emplace_back(j, j,  w);
        tripY.emplace_back(i, j, -w);
        tripY.emplace_back(j, i, -w);
    };

    // 2) 进度条（按 pin 数）
    std::size_t totalPins = 0;
    for (Net* net : db->Nets) if (net) totalPins += net->netPins.size();
    ProgressBar pb;
    pb.init(totalPins, /*gate*/1024, /*min_refresh_ms*/120);

    // 3) 按 net 装配
    for (Net* net : db->Nets) {
        if (!net) continue;
        auto& pins = net->netPins;
        const int k = static_cast<int>(pins.size());
        pb.tick(static_cast<std::size_t>(k));
        if (k < 2) continue;

        // 分类：可动 / 固定
        std::vector<int> movIdx;    movIdx.reserve(k);
        std::vector<double> movDx;  movDx.reserve(k);
        std::vector<double> movDy;  movDy.reserve(k);

        std::vector<double> fixX;   fixX.reserve(k);
        std::vector<double> fixY;   fixY.reserve(k);
        std::vector<double> fixDx;  fixDx.reserve(k);
        std::vector<double> fixDy;  fixDy.reserve(k);

        for (int t = 0; t < k; ++t) {
            Pin* p = pins[t];
            if (!p || !p->module) continue;
            Module* m = p->module;

            if (m->isFixed) {
                fixX.push_back(static_cast<double>(m->center.x));
                fixY.push_back(static_cast<double>(m->center.y));
                fixDx.push_back(static_cast<double>(p->offset.x));
                fixDy.push_back(static_cast<double>(p->offset.y));
            } else {
                auto it = modToIndex.find(m);
                if (it == modToIndex.end()) continue; // safety
                movIdx.push_back(it->second);
                movDx.push_back(static_cast<double>(p->offset.x));
                movDy.push_back(static_cast<double>(p->offset.y));
            }
        }

        const int km = static_cast<int>(movIdx.size());
        const int kf = static_cast<int>(fixX.size());
        if (km == 0) continue;

        // 与原逻辑近似的权重
        const double w = 1.0 / static_cast<double>(std::max(1, k - 1));

        if (k <= K_CLIQUE_MAX) {
            // —— 小网：保持 clique —— //
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
                // 可动-固定（全部固定 pin）
                if (kf > 0) {
                    // 也可仅取 TOP_K_FIX 最近固定 pin（提速）
                    for (int f = 0; f < kf; ++f) {
                        add_diag(ia, w);
                        b_x[ia] += w * ((fixX[f] + fixDx[f]) - movDx[a]);
                        b_y[ia] += w * ((fixY[f] + fixDy[f]) - movDy[a]);
                    }
                }
            }
        } else {
            // —— 大网：避免 O(k^2) —— //
            if (kf > 0) {
                // 只连 可动 ↔ 最近的固定锚点（TOP_K_FIX）
                for (int a = 0; a < km; ++a) {
                    const int ia = movIdx[a];

                    // 线性找最近 TOP_K_FIX（如需更快可用网格/树结构）
                    std::vector<std::pair<double,int>> cand;
                    cand.reserve(kf);
                    const double x_i = static_cast<double>(movableMods[ia]->center.x) + movDx[a];
                    const double y_i = static_cast<double>(movableMods[ia]->center.y) + movDy[a];
                    for (int f = 0; f < kf; ++f) {
                        const double x_f = fixX[f] + fixDx[f];
                        const double y_f = fixY[f] + fixDy[f];
                        const double dx = x_i - x_f;
                        const double dy = y_i - y_f;
                        cand.emplace_back(dx*dx + dy*dy, f);
                    }
                    std::nth_element(cand.begin(),
                                     cand.begin() + std::min(TOP_K_FIX, (int)cand.size()),
                                     cand.end(),
                                     [](const auto& a, const auto& b){ return a.first < b.first; });
                    const int k_used = std::min(TOP_K_FIX, (int)cand.size());

                    add_diag(ia, w * static_cast<double>(k_used));
                    double bx = 0.0, by = 0.0;
                    for (int u = 0; u < k_used; ++u) {
                        const int f = cand[u].second;
                        bx += (fixX[f] + fixDx[f]) - movDx[a];
                        by += (fixY[f] + fixDy[f]) - movDy[a];
                    }
                    b_x[ia] += w * bx;
                    b_y[ia] += w * by;
                }
            } else {
                // 全可动大网：几何重心锚点（替代仅正则，防漂移/团聚）
                if (km <= 4) {
                    // 极小退化为 clique
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
                    double cx = 0.0, cy = 0.0;
                    for (int a = 0; a < km; ++a) {
                        const int ia = movIdx[a];
                        cx += static_cast<double>(movableMods[ia]->center.x) + movDx[a];
                        cy += static_cast<double>(movableMods[ia]->center.y) + movDy[a];
                    }
                    cx /= km; cy /= km;
                    for (int a = 0; a < km; ++a) {
                        const int ia = movIdx[a];
                        add_diag(ia, w);
                        b_x[ia] += w * (cx - movDx[a]);
                        b_y[ia] += w * (cy - movDy[a]);
                    }
                }
            }
        }
    }
    pb.finish();

    // 4) 全局小正则，确保可解
    for (int i = 0; i < N; ++i) add_diag(i, LAMBDA_DIAG);

    // 5) 构建稀疏矩阵并求解
    Eigen::SparseMatrix<double> A_x(N, N), A_y(N, N);
    A_x.setFromTriplets(tripX.begin(), tripX.end(), std::plus<double>());
    A_y.setFromTriplets(tripY.begin(), tripY.end(), std::plus<double>());
    A_x.makeCompressed();
    A_y.makeCompressed();

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver_x, solver_y;
    solver_x.compute(A_x);
    solver_y.compute(A_y);
    if (solver_x.info() != Eigen::Success || solver_y.info() != Eigen::Success) {
        std::cerr << "InitialPlacement: factorization failed. Try increasing regularization.\n";
        return;
    }

    Eigen::VectorXd X = solver_x.solve(b_x);
    Eigen::VectorXd Y = solver_y.solve(b_y);
    if (solver_x.info() != Eigen::Success || solver_y.info() != Eigen::Success) {
        std::cerr << "InitialPlacement: solve failed.\n";
        return;
    }

    // 6) 写回坐标
    for (int i = 0; i < N; ++i) {
        movableMods[i]->center.x = static_cast<float>(X[i]);
        movableMods[i]->center.y = static_cast<float>(Y[i]);
    }

    std::cout << "\nInitial placement completed.\n";
}

// ======================================================================
//                   行区域 Bounding Box（修订版）
// - 统一使用 row.bottom / row.bottom+row.height
// - 全部以 double 计算，避免 float/double 混用
// ======================================================================
std::pair<POS_2D, POS_2D> MyPlacer::getSiteRowBoundingBox() {
    if (db->SiteRows.empty()) return {{0,0},{1000,1000}};

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& row : db->SiteRows) {
        const double sx = static_cast<double>(row.start.x);
        const double ex = static_cast<double>(row.end.x);
        const double by = static_cast<double>(row.bottom);
        const double ty = static_cast<double>(row.bottom + row.height);

        min_x = std::min(min_x, sx);
        max_x = std::max(max_x, ex);
        min_y = std::min(min_y, by);
        max_y = std::max(max_y, ty);
    }
    return {{min_x, min_y}, {max_x, max_y}};
}

// ======================================================================
//                   Bin 初始化（修订版）
// - 稳健的行/列估算（带上下限，匹配长宽比）
// - 仅遍历与固定宏相交的 bin；terminalDensity 归一化到 0..1
// - dieBbox 合法性校验
// ======================================================================
void MyPlacer::initializeBins(double targetDensity) {
    auto [ll_site, ur_site] = getSiteRowBoundingBox();
    const double dieW = static_cast<double>(ur_site.x - ll_site.x);
    const double dieH = static_cast<double>(ur_site.y - ll_site.y);
    if (dieW <= 0.0 || dieH <= 0.0) {
        std::cerr << "[initializeBins] invalid die bbox: W=" << dieW << " H=" << dieH << "\n";
        return;
    }

    const double binTargetH = std::max(1.0, static_cast<double>(db->siteHeight) * 32.0);
    int rows = static_cast<int>(std::ceil(dieH / binTargetH));
    rows = std::clamp(rows, 8, 256);

    const double binH = dieH / static_cast<double>(rows);
    const double binW_target = binH * (dieW / dieH);
    int cols = static_cast<int>(std::ceil(dieW / binW_target));
    cols = std::clamp(cols, 8, 256);

    db->binRows = rows;
    db->binCols = cols;
    db->bins.assign(rows, std::vector<Bin>(cols));

    const double bw = dieW / static_cast<double>(cols);
    const double bh = dieH / static_cast<double>(rows);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            Bin& b = db->bins[i][j];
            b.ll.x = static_cast<float>(ll_site.x + j * bw);
            b.ll.y = static_cast<float>(ll_site.y + i * bh);
            b.ur.x = static_cast<float>(b.ll.x + bw);
            b.ur.y = static_cast<float>(b.ll.y + bh);
            b.center.x = static_cast<float>((static_cast<double>(b.ll.x) + static_cast<double>(b.ur.x)) * 0.5);
            b.center.y = static_cast<float>((static_cast<double>(b.ll.y) + static_cast<double>(b.ur.y)) * 0.5);
            b.width  = static_cast<float>(bw);
            b.height = static_cast<float>(bh);
            b.terminalDensity = 0.0f;
            b.darkDensity     = 0.0f;
        }
    }

    // 固定宏占用（仅遍历相交 bin）
    for (Module* mod : db->Nodes) {
        if (!mod || !mod->isFixed) continue;

        const double mllx = static_cast<double>(mod->center.x) - static_cast<double>(mod->width)  * 0.5;
        const double mlly = static_cast<double>(mod->center.y) - static_cast<double>(mod->height) * 0.5;
        const double murx = static_cast<double>(mod->center.x) + static_cast<double>(mod->width)  * 0.5;
        const double mury = static_cast<double>(mod->center.y) + static_cast<double>(mod->height) * 0.5;

        int j0 = std::clamp(static_cast<int>(std::floor((mllx - static_cast<double>(ll_site.x)) / bw)), 0, cols - 1);
        int j1 = std::clamp(static_cast<int>(std::floor((murx - static_cast<double>(ll_site.x)) / bw)), 0, cols - 1);
        int i0 = std::clamp(static_cast<int>(std::floor((mlly - static_cast<double>(ll_site.y)) / bh)), 0, rows - 1);
        int i1 = std::clamp(static_cast<int>(std::floor((mury - static_cast<double>(ll_site.y)) / bh)), 0, rows - 1);

        for (int i = i0; i <= i1; ++i) {
            for (int j = j0; j <= j1; ++j) {
                Bin& bin = db->bins[i][j];

                const double bllx = static_cast<double>(bin.ll.x);
                const double blly = static_cast<double>(bin.ll.y);
                const double burx = static_cast<double>(bin.ur.x);
                const double bury = static_cast<double>(bin.ur.y);

                const double ox = std::max(0.0, std::min(murx, burx) - std::max(mllx, bllx));
                const double oy = std::max(0.0, std::min(mury, bury) - std::max(mlly, blly));
                const double overlap = ox * oy;
                if (overlap <= 0.0) continue;

                const double binArea = static_cast<double>(bin.width) * static_cast<double>(bin.height);
                if (binArea <= 0.0) continue;

                const double occ = overlap / binArea; // 0~1
                bin.terminalDensity = static_cast<float>(std::clamp(
                    static_cast<double>(bin.terminalDensity) + occ, 0.0, 1.0));
            }
        }
    }

    std::cout << "Bin grid initialized: " << rows << " x " << cols << std::endl;
}

// ======================================================================
//                   Filler 生成（修订版）
// - 统计“可移动标准单元”（非固定、非宏、非 filler）的面积分布
// - 统一 double 计算；bbox 与 initializeBins 一致
// ======================================================================
void MyPlacer::createfillerCells(){
    if (db->SiteRows.empty()) {
        std::cout << "No site rows. Skip filler creation.\n";
        return;
    }

    double dieLx = std::numeric_limits<double>::max();
    double dieLy = std::numeric_limits<double>::max();
    double dieUx = std::numeric_limits<double>::lowest();
    double dieUy = std::numeric_limits<double>::lowest();

    double rowArea = 0.0;
    for(const auto& row : db->SiteRows){
        const double sx = static_cast<double>(row.start.x);
        const double ex = static_cast<double>(row.end.x);
        const double by = static_cast<double>(row.bottom);
        const double ty = static_cast<double>(row.bottom + row.height);

        dieLx = std::min(dieLx, sx);
        dieLy = std::min(dieLy, by);
        dieUx = std::max(dieUx, ex);
        dieUy = std::max(dieUy, ty);

        const double width = ex - sx;
        rowArea += width * static_cast<double>(row.height);
    }

    double fixedArea = 0.0;
    std::vector<double> movableAreas;
    movableAreas.reserve(db->Nodes.size());
    double movableTotalArea = 0.0;

    for(Module* mod : db->Nodes){
        if(!mod) continue;
        if(mod->isFixed){
            fixedArea += static_cast<double>(mod->area);
        } else if(!mod->isMacro && !mod->isFiller) { // 标准单元
            const double a = static_cast<double>(mod->area);
            movableAreas.push_back(a);
            movableTotalArea += a;
        }
    }

    if (movableAreas.empty()) {
        std::cout << "No movable std cells. Skip filler creation." << std::endl;
        return;
    }

    std::sort(movableAreas.begin(), movableAreas.end());
    const int n = static_cast<int>(movableAreas.size());
    int start = static_cast<int>(std::floor(n * 0.05));
    int end   = static_cast<int>(std::ceil (n * 0.95));
    start = std::clamp(start, 0, n);
    end   = std::clamp(end,   0, n);
    if (start >= end) { start = 0; end = n; }

    double sum = 0.0;
    for (int i = start; i < end; ++i) sum += movableAreas[i];
    const double avgMovableArea = (end > start) ? (sum / (end - start))
                                                : (movableTotalArea / std::max(1, n));

    const double targetDensity = 0.80; // 参数可外部化
    const double availableArea = std::max(0.0, rowArea - fixedArea);
    const double requiredMovableArea = targetDensity * availableArea;

    double placedLikeArea = 0.0;
    for(Module* mod : db->Nodes){
        if(!mod || mod->isFixed) continue;
        const double a = static_cast<double>(mod->area);
        if (mod->isMacro) placedLikeArea += a * targetDensity;
        else              placedLikeArea += a;
    }

    double fillerArea = requiredMovableArea - placedLikeArea;
    if (fillerArea <= 0.0) {
        std::cout << "No filler needed (density already satisfied)." << std::endl;
        return;
    }

    const double fillerHeight = static_cast<double>(db->siteHeight);
    double fillerWidth = (fillerHeight > 0.0) ? (avgMovableArea / fillerHeight) : 1.0;
    if (fillerWidth <= 0.0) fillerWidth = 1.0;

    const double cellArea = fillerWidth * fillerHeight;
    int numFillers = static_cast<int>(std::floor(fillerArea / std::max(1.0, cellArea)));
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
        filler->width  = static_cast<float>(fillerWidth);
        filler->height = static_cast<float>(fillerHeight);
        filler->area   = static_cast<float>(fillerWidth * fillerHeight);
        filler->isFixed  = false;
        filler->isFiller = true;
        filler->isMacro  = false;

        filler->center.x = static_cast<float>(dis_x(gen));
        filler->center.y = static_cast<float>(dis_y(gen));

        db->Nodes.push_back(filler);
    }

    std::cout << "Added " << numFillers << " filler cells (each "
              << fillerWidth << " x " << fillerHeight << ")." << std::endl;
}
