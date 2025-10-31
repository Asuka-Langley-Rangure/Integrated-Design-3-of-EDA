#include "myplace.h"
#include "../eigen3/Eigen/Sparse"
#include "../eigen3/Eigen/IterativeLinearSolvers"   // BiCGSTAB
#include "../eigen3/Eigen/SparseCholesky"            // 备用: LDLT
#include <unordered_map>
#include <random>
#include <chrono>
#include <iostream>
#include <iomanip>

// ===================== 工具：轻量级进度条 =====================
static inline void printProgress(size_t cur, size_t tot, const char* label, bool force=false) {
    if (tot == 0) return;
    // 每处理 256 项刷新一次，或强制刷新
    if (!force && (cur + 1) % 256 != 0 && cur + 1 != tot) return;
    double ratio = double(cur + 1) / double(tot);
    int width = 40;
    int filled = int(ratio * width);
    std::cout << "\r[" << label << "] [";
    for (int i = 0; i < width; ++i) std::cout << (i < filled ? '=' : ' ');
    std::cout << "] " << std::fixed << std::setprecision(1) << (ratio * 100.0) << "%";
    std::cout.flush();
    if (cur + 1 == tot) {
        std::cout << std::endl;
    }
}
// ============================================================

void MyPlacer::initialPlacement()
{
    using Clock = std::chrono::high_resolution_clock;
    auto t0 = Clock::now();

    // 1) 收集所有可移动模块（unordered_map 加速）
    std::vector<Module*> movableMods;
    movableMods.reserve(db->Nodes.size());
    std::unordered_map<Module*, int> modToIndex;
    modToIndex.reserve(db->Nodes.size() * 2);

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

    // 2) Triplet 批量构建稀疏矩阵，b 累加到临时数组
    std::vector<Eigen::Triplet<double>> tripX, tripY;
    tripX.reserve(size_t(db->Nets.size()) * 8);
    tripY.reserve(size_t(db->Nets.size()) * 8);

    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(N);

    // 大网使用 Star 近似，减小 O(d^2) 成本
    const int CLIQUE_TO_STAR_CUTOFF = 32;

    // 预处理：统计需要显示进度的有效网数
    size_t validNets = 0;
    for (Net* net : db->Nets) {
        if (net && net->netPins.size() >= 2) ++validNets;
    }
    size_t processed = 0;

    for (Net* net : db->Nets) {
        if (!net || net->netPins.size() < 2) continue;
        const int deg = static_cast<int>(net->netPins.size());
        const double weight = 1.0 / std::max(1, deg - 1);

        auto handle_pair = [&](Pin* p, Pin* q) {
            Module* m = p->module;
            Module* n = q->module;
            const bool mFixed = m->isFixed;
            const bool nFixed = n->isFixed;

            const double px_fixed = m->center.x;
            const double py_fixed = m->center.y;
            const double qx_fixed = n->center.x;
            const double qy_fixed = n->center.y;

            const double dx = p->offset.x;
            const double dy = p->offset.y;
            const double ex = q->offset.x;
            const double ey = q->offset.y;

            if (!mFixed && !nFixed) {
                // 两者可移动：构建 (im,in) 的四个项 + b 的两项
                auto it_m = modToIndex.find(m);
                auto it_n = modToIndex.find(n);
                if (it_m == modToIndex.end() || it_n == modToIndex.end()) return;

                int im = it_m->second, in = it_n->second;
                const double ddx = dx - ex;
                const double ddy = dy - ey;

                // Ax
                tripX.emplace_back(im, im, +weight);
                tripX.emplace_back(in, in, +weight);
                tripX.emplace_back(im, in, -weight);
                tripX.emplace_back(in, im, -weight);
                b_x[im] += weight * ddx;
                b_x[in] -= weight * ddx;

                // Ay
                tripY.emplace_back(im, im, +weight);
                tripY.emplace_back(in, in, +weight);
                tripY.emplace_back(im, in, -weight);
                tripY.emplace_back(in, im, -weight);
                b_y[im] += weight * ddy;
                b_y[in] -= weight * ddy;
            } else if (!mFixed && nFixed) {
                auto it_m = modToIndex.find(m);
                if (it_m == modToIndex.end()) return;
                int im = it_m->second;
                const double target_x = (qx_fixed + ex) - dx;
                const double target_y = (qy_fixed + ey) - dy;

                tripX.emplace_back(im, im, +weight);
                b_x[im] += weight * target_x;

                tripY.emplace_back(im, im, +weight);
                b_y[im] += weight * target_y;
            } else if (mFixed && !nFixed) {
                auto it_n = modToIndex.find(n);
                if (it_n == modToIndex.end()) return;
                int in = it_n->second;
                const double target_x = (px_fixed + dx) - ex;
                const double target_y = (py_fixed + dy) - ey;

                tripX.emplace_back(in, in, +weight);
                b_x[in] += weight * target_x;

                tripY.emplace_back(in, in, +weight);
                b_y[in] += weight * target_y;
            } else {
                // 两者都固定：对未知量无贡献，跳过
            }
        };

        if (deg <= CLIQUE_TO_STAR_CUTOFF) {
            // 团模型：所有两两配对
            for (int i = 0; i < deg; ++i) {
                for (int j = i + 1; j < deg; ++j) {
                    handle_pair(net->netPins[i], net->netPins[j]);
                }
            }
        } else {
            // 星模型：选第一个 pin 为“中心”，只连到中心
            Pin* hub = net->netPins[0];
            for (int j = 1; j < deg; ++j) {
                handle_pair(hub, net->netPins[j]);
            }
        }

        printProgress(processed++, validNets, "build A/b");
    }

    auto t1 = Clock::now();

    // 3) 构建稀疏矩阵
    Eigen::SparseMatrix<double> A_x(N, N), A_y(N, N);
    A_x.setFromTriplets(tripX.begin(), tripX.end());
    A_y.setFromTriplets(tripY.begin(), tripY.end());
    A_x.makeCompressed();
    A_y.makeCompressed();

    // 4) 求解 A_x X = b_x, A_y Y = b_y
    // 建议：BiCGSTAB + IncompleteLUT 预条件（大型稀疏系统通常更快）
    Eigen::BiCGSTAB<Eigen::SparseMatrix<double>, Eigen::IncompleteLUT<double>> solver_x, solver_y;

    solver_x.preconditioner().setFillfactor(10); // 预条件参数，可调
    solver_y.preconditioner().setFillfactor(10);
    solver_x.setTolerance(1e-6);
    solver_y.setTolerance(1e-6);
    solver_x.setMaxIterations(std::max(1000, N * 2));
    solver_y.setMaxIterations(std::max(1000, N * 2));

    solver_x.compute(A_x);
    solver_y.compute(A_y);

    Eigen::VectorXd X = solver_x.solve(b_x);
    printProgress(1, 2, "solve X");
    Eigen::VectorXd Y = solver_y.solve(b_y);
    printProgress(1, 1, "solve Y", /*force=*/true);

    // 如果需要，也可在收敛不佳时回退到 LDLT：
    // if (solver_x.info() != Eigen::Success || solver_y.info() != Eigen::Success) {
    //     Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt_x, ldlt_y;
    //     ldlt_x.compute(A_x); ldlt_y.compute(A_y);
    //     X = ldlt_x.solve(b_x);
    //     Y = ldlt_y.solve(b_y);
    // }

    auto t2 = Clock::now();

    // 5) 写回
    for (int i = 0; i < N; ++i) {
        movableMods[i]->center.x = static_cast<float>(X[i]);
        movableMods[i]->center.y = static_cast<float>(Y[i]);
    }

    auto t3 = Clock::now();

    auto dur_build = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    auto dur_solve = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    auto dur_write = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();

    std::cout << "Initial placement completed. "
              << "[build " << dur_build << " ms, "
              << "solve " << dur_solve << " ms, "
              << "write " << dur_write << " ms]\n";
}

void MyPlacer::createfillerCells(){
    using Clock = std::chrono::high_resolution_clock;
    auto t0 = Clock::now();

    double dieLx = std::numeric_limits<double>::max();
    double dieLy = std::numeric_limits<double>::max();
    double dieUx = std::numeric_limits<double>::lowest();
    double dieUy = std::numeric_limits<double>::lowest();

    double rowArea = 0.0;
    for (const auto& row : db->SiteRows) {
        dieLx = std::min<double>(dieLx, row.start.x);
        dieLy = std::min<double>(dieLy, row.start.y);
        dieUx = std::max<double>(dieUx, row.end.x);
        dieUy = std::max<double>(dieUy, row.bottom + row.height);
        rowArea += (row.end.x - row.start.x) * row.height;
    }

    double fixedArea = 0.0;
    std::vector<double> movableAreas;
    movableAreas.reserve(db->Nodes.size());

    double x = 0.0; // 标准单元面积 + 宏块面积*targetDensity
    for (Module* mod : db->Nodes) {
        if (!mod) continue;
        if (mod->isFixed) {
            fixedArea += mod->area;
        } else {
            movableAreas.push_back(mod->area);
            if (mod->isMacro) x += mod->area * 0.8; // 与 targetDensity 同步
            else               x += mod->area;
        }
    }

    if (movableAreas.empty()) {
        std::cout << "No movable cells. Skip filler creation.\n";
        return;
    }

    std::sort(movableAreas.begin(), movableAreas.end());
    int n = static_cast<int>(movableAreas.size());
    int start = std::max(0, (int)std::floor(n * 0.05));
    int end   = std::max(start + 1, (int)std::ceil(n * 0.95));
    double sum = 0.0;
    for (int i = start; i < end; ++i) sum += movableAreas[i];
    const double avgMovableArea = sum / (end - start);

    const double targetDensity = 0.8;
    const double availableArea = rowArea - fixedArea;
    const double requiredMovableArea = targetDensity * availableArea;
    const double fillerArea = requiredMovableArea - x;

    if (fillerArea <= 0) {
        std::cout << "No filler needed (density already satisfied).\n";
        return;
    }

    double fillerHeight = db->siteHeight;
    double fillerWidth = avgMovableArea / std::max(1.0, fillerHeight);
    if (fillerWidth <= 0) fillerWidth = 1.0;

    int numFillers = static_cast<int>(fillerArea / std::max(1e-9, avgMovableArea));
    if (numFillers <= 0) {
        std::cout << "No filler created (too small requirement).\n";
        return;
    }

    std::cout << "Creating " << numFillers << " filler cells..." << std::endl;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(dieLx, dieUx);
    std::uniform_real_distribution<> dis_y(dieLy, dieUy);

    db->Nodes.reserve(db->Nodes.size() + static_cast<size_t>(numFillers));

    for (int i = 0; i < numFillers; ++i) {
        Module* filler = new Module();
        filler->name = "FILLER_" + std::to_string(i);
        filler->width  = static_cast<float>(fillerWidth);
        filler->height = static_cast<float>(fillerHeight);
        filler->area   = static_cast<float>(fillerWidth * fillerHeight);
        filler->isFixed  = false;
        filler->isFiller = true;
        filler->center.x = static_cast<float>(dis_x(gen));
        filler->center.y = static_cast<float>(dis_y(gen));
        db->Nodes.push_back(filler);

        printProgress(i, numFillers, "create fillers");
    }
    printProgress(numFillers - 1, numFillers, "create fillers", /*force=*/true);

    auto t1 = Clock::now();
    std::cout << "Added " << numFillers << " filler cells (each "
              << fillerWidth << " x " << fillerHeight << "). "
              << "[time " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
              << " ms]\n";
}

std::pair<POS_2D, POS_2D> MyPlacer::getSiteRowBoundingBox() {
    if (db->SiteRows.empty()) {
        return {{0, 0}, {1000, 1000}};
    }
    double min_x = db->SiteRows[0].start.x;
    double min_y = db->SiteRows[0].start.y;
    double max_x = db->SiteRows[0].end.x;
    double max_y = db->SiteRows[0].end.y;
    for (const auto& row : db->SiteRows) {
        min_x = std::min<double>(min_x, row.start.x);
        min_y = std::min<double>(min_y, row.start.y);
        max_x = std::max<double>(max_x, row.end.x);
        max_y = std::max<double>(max_y, row.end.y);
    }
    return {{min_x, min_y}, {max_x, max_y}};
}

void MyPlacer::initializeBins(double targetDensity) {
    using Clock = std::chrono::high_resolution_clock;
    auto t0 = Clock::now();

    double totalArea = 0.0;
    int    nodeCount = 0;
    for (Module* mod : db->Nodes) {
        if (!mod) continue;
        totalArea += mod->width * mod->height;
        ++nodeCount;
    }
    if (nodeCount == 0 || totalArea == 0) {
        std::cerr << "Warning: No modules to compute bin grid.\n";
        return;
    }

    const double avgNodeArea = totalArea / nodeCount;
    const double expectedBinArea = avgNodeArea / std::max(1e-9, targetDensity);

    auto [ll_site, ur_site] = getSiteRowBoundingBox();
    const double layoutW = ur_site.x - ll_site.x;
    const double layoutH = ur_site.y - ll_site.y;
    const double layoutArea = layoutW * layoutH;

    const double M_float = layoutArea / std::max(1e-9, expectedBinArea);
    int M = static_cast<int>(std::ceil(M_float));

    int n = static_cast<int>(std::floor(std::log2(std::sqrt(std::max(1.0, double(M))))));
    int binDim = 1 << n;
    if (binDim * binDim < M) binDim = 1 << (n + 1);

    db->binRows = db->binCols = binDim;
    std::cout << "Initialize bins: " << db->binRows << " x " << db->binCols << std::endl;

    db->bins.assign(db->binRows, std::vector<Bin>(db->binCols));
    const double binWidth  = layoutW / db->binCols;
    const double binHeight = layoutH / db->binRows;

    // 建 bin + 进度条
    const size_t totalBins = static_cast<size_t>(db->binRows) * static_cast<size_t>(db->binCols);
    size_t built = 0;
    for (int i = 0; i < db->binRows; i++) {
        for (int j = 0; j < db->binCols; j++) {
            Bin& bin = db->bins[i][j];
            bin.ll.x = ll_site.x + j * binWidth;
            bin.ll.y = ll_site.y + i * binHeight;
            bin.ur.x = bin.ll.x + binWidth;
            bin.ur.y = bin.ll.y + binHeight;
            bin.center.x = (bin.ll.x + bin.ur.x) / 2.0;
            bin.center.y = (bin.ll.y + bin.ur.y) / 2.0;
            bin.width  = binWidth;
            bin.height = binHeight;
            bin.terminalDensity = 0.0;
            bin.darkDensity     = 0.0;

            printProgress(built++, totalBins, "build bins");
        }
    }
    printProgress(totalBins - 1, totalBins, "build bins", /*force=*/true);

    // 计算 terminalDensity（固定宏重叠）
    size_t fixedCnt = 0, fixedTotal = 0;
    for (Module* mod : db->Nodes) if (mod && mod->isFixed) ++fixedTotal;

    for (Module* mod : db->Nodes) {
        if (!mod || !mod->isFixed) continue;
        const double mod_llx = mod->center.x - mod->width  / 2.0;
        const double mod_lly = mod->center.y - mod->height / 2.0;
        const double mod_urx = mod->center.x + mod->width  / 2.0;
        const double mod_ury = mod->center.y + mod->height / 2.0;

        for (int i = 0; i < db->binRows; i++) {
            for (int j = 0; j < db->binCols; j++) {
                Bin& bin = db->bins[i][j];
                const double overlap_x = std::max<double>(0.0, std::min<double>(mod_urx, bin.ur.x) - std::max<double>(mod_llx, bin.ll.x));
                const double overlap_y = std::max<double>(0.0, std::min<double>(mod_ury, bin.ur.y) - std::max<double>(mod_lly, bin.ll.y));
                const double overlapArea = overlap_x * overlap_y;
                if (overlapArea > 0) {
                    bin.terminalDensity += targetDensity * overlapArea;
                }
            }
        }
        printProgress(fixedCnt++, std::max<size_t>(1, fixedTotal), "terminal dens");
    }
    if (fixedTotal) printProgress(fixedTotal - 1, fixedTotal, "terminal dens", /*force=*/true);

    // 暗节点密度：如需精确计算，可按行/列裁剪 SiteRow 多边形；这里保持为 0
    // for (...) bins[i][j].darkDensity += targetDensity * OutADark;

    auto t1 = Clock::now();
    std::cout << "Bin grid initialized. [time "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
              << " ms]\n";
}
