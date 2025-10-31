#include "myplace.h"

void MyPlacer::initialPlacement()
{
    // 0) 收集所有可移动模块，并建立索引
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

    // 1) 预分配 Triplets 与 RHS
    std::vector<Eigen::Triplet<double>> tripX, tripY;
    tripX.reserve(N * 12);
    tripY.reserve(N * 12);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(N);

    // 参数：小网阈值 / 采样个数 / 正则
    constexpr int    K_CLIQUE_MAX  = 8;
    constexpr int    K_SAMPLE      = 4;
    constexpr double LAMBDA_DIAG   = 1e-6;

    auto add_diag = [&](int idx, double w){
        tripX.emplace_back(idx, idx, w);
        tripY.emplace_back(idx, idx, w);
    };
    auto add_edge = [&](int i, int j, double w){
        // 拉普拉斯装配：L(ii)+=w, L(jj)+=w, L(ij)-=w, L(ji)-=w
        tripX.emplace_back(i, i,  w);
        tripX.emplace_back(j, j,  w);
        tripX.emplace_back(i, j, -w);
        tripX.emplace_back(j, i, -w);
        tripY.emplace_back(i, i,  w);
        tripY.emplace_back(j, j,  w);
        tripY.emplace_back(i, j, -w);
        tripY.emplace_back(j, i, -w);
    };

    // 2) 进度计数（降频刷新）
    size_t totalPins = 0;
    for (Net* net : db->Nets) if (net) totalPins += net->netPins.size();
    LiveCounter lc;
    lc.init(totalPins, /*step_gate=*/128, /*ms_gate=*/0);

    // 3) 按 net 装配
    for (Net* net : db->Nets) {
        if (!net) continue;
        auto& pins = net->netPins;
        const int k = static_cast<int>(pins.size());
        lc.tick(k);
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
                // 固定锚点：记录其绝对中心与 pin offset
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

        // 与原始逻辑保持近似的权重
        const double w = 1.0 / static_cast<double>(std::max(1, k - 1));

        if (k <= K_CLIQUE_MAX) {
            // —— 小网：保持 clique 质量更好 —— //
            for (int a = 0; a < km; ++a) {
                const int ia = movIdx[a];
                for (int b = a + 1; b < km; ++b) {
                    const int ib = movIdx[b];
                    // 可动-可动边
                    add_edge(ia, ib, w);
                    // RHS：偏移差（(x_i+dx_i) ~ (x_j+dx_j)）
                    const double ddx = movDx[a] - movDx[b];
                    const double ddy = movDy[a] - movDy[b];
                    b_x[ia] +=  w * ddx;  b_x[ib] -=  w * ddx;
                    b_y[ia] +=  w * ddy;  b_y[ib] -=  w * ddy;
                }
                // 可动-固定项：对角 + RHS
                for (int f = 0; f < kf; ++f) {
                    add_diag(ia, w);
                    b_x[ia] += w * ((fixX[f] + fixDx[f]) - movDx[a]);
                    b_y[ia] += w * ((fixY[f] + fixDy[f]) - movDy[a]);
                }
            }
        } else {
            // —— 大网：避免 O(k^2) —— //
            if (kf > 0) {
                // 只连 可动↔固定（把固定作为锚点）
                for (int a = 0; a < km; ++a) {
                    const int ia = movIdx[a];
                    add_diag(ia, w * static_cast<double>(kf));
                    double bx = 0.0, by = 0.0;
                    for (int f = 0; f < kf; ++f) {
                        bx += (fixX[f] + fixDx[f]) - movDx[a];
                        by += (fixY[f] + fixDy[f]) - movDy[a];
                    }
                    b_x[ia] += w * bx;
                    b_y[ia] += w * by;
                }
            } else {
                // 全可动大网：避免全连
                if (km <= K_SAMPLE) {
                    // km 很小则退化为 clique
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
                    // 仅加轻微正则，后续由密度/平滑阶段拉拢
                    for (int a = 0; a < km; ++a) add_diag(movIdx[a], LAMBDA_DIAG);
                }
            }
        }
    }
    lc.finish();

    // 4) 全局再加一遍小正则，确保可解
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


// ------------------------ getSiteRowBoundingBox ------------------------
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

// ------------------------ initializeBins ------------------------
void MyPlacer::initializeBins(double targetDensity) {
    // 1) 取行区域 bbox，并做合法性检查
    auto [ll_site, ur_site] = getSiteRowBoundingBox();
    const double dieW = static_cast<double>(ur_site.x - ll_site.x);
    const double dieH = static_cast<double>(ur_site.y - ll_site.y);
    if (dieW <= 0.0 || dieH <= 0.0) {
        std::cerr << "[initializeBins] invalid die bbox: W=" << dieW << " H=" << dieH << "\n";
        return;
    }

    // 2) 稳健的 bin 划分：按“每 bin ≈ 32 行高”估算行数，再按长宽比推列数，并加上下限
    const double binTargetH = std::max(1.0, static_cast<double>(db->siteHeight) * 32.0);
    int rows = static_cast<int>(std::ceil(dieH / binTargetH));
    rows = std::clamp(rows, 8, 256); // 下限避免过粗，上限避免网格爆炸

    // 根据 rows 与 die 的长宽比推导 cols，避免 bin 过扁或过细
    const double binH = dieH / static_cast<double>(rows);
    const double binW_target = binH * (dieW / dieH);
    int cols = static_cast<int>(std::ceil(dieW / binW_target));
    cols = std::clamp(cols, 8, 256);

    db->binRows = rows;
    db->binCols = cols;
    db->bins.assign(rows, std::vector<Bin>(cols));

    const double bw = dieW / static_cast<double>(cols);
    const double bh = dieH / static_cast<double>(rows);

    // 3) 建立 bin 网格几何信息与初始化密度
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
            b.terminalDensity = 0.0;
            b.darkDensity     = 0.0; // 若需处理“非行区域”可在此填充
        }
    }

    // 4) 仅对固定宏（terminal）累计占用密度：只遍历相交 bin，且按 bin 面积归一化
    for (Module* mod : db->Nodes) {
        if (!mod || !mod->isFixed) continue;

        const double mllx = static_cast<double>(mod->center.x) - static_cast<double>(mod->width)  * 0.5;
        const double mlly = static_cast<double>(mod->center.y) - static_cast<double>(mod->height) * 0.5;
        const double murx = static_cast<double>(mod->center.x) + static_cast<double>(mod->width)  * 0.5;
        const double mury = static_cast<double>(mod->center.y) + static_cast<double>(mod->height) * 0.5;

        // 相交 bin 下标范围（裁剪到有效区间）
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

// ------------------------ createfillerCells ------------------------
void MyPlacer::createfillerCells() {
    if (db->SiteRows.empty()) {
        std::cout << "No site rows. Skip filler creation.\n";
        return;
    }

    // 1) 行区域 bbox 与总行面积（全部用 double 计算）
    double dieLx = std::numeric_limits<double>::max();
    double dieLy = std::numeric_limits<double>::max();
    double dieUx = std::numeric_limits<double>::lowest();
    double dieUy = std::numeric_limits<double>::lowest();

    double rowArea = 0.0;
    for (const auto& row : db->SiteRows) {
        const double sx = static_cast<double>(row.start.x);
        const double ex = static_cast<double>(row.end.x);
        const double by = static_cast<double>(row.bottom);
        const double ty = static_cast<double>(row.bottom + row.height);

        dieLx = std::min(dieLx, sx);
        dieLy = std::min(dieLy, by);
        dieUx = std::max(dieUx, ex);
        dieUy = std::max(dieUy, ty);

        rowArea += (ex - sx) * static_cast<double>(row.height);
    }

    // 2) 统计固定面积 & 可移动标准单元面积分布（非固定、非宏、非 filler）
    double fixedArea = 0.0;
    std::vector<double> movableAreas;
    movableAreas.reserve(db->Nodes.size());
    double movableTotalArea = 0.0;

    for (Module* mod : db->Nodes) {
        if (!mod) continue;
        if (mod->isFixed) {
            fixedArea += static_cast<double>(mod->area);
        } else if (!mod->isMacro && !mod->isFiller) { // 仅标准单元
            const double a = static_cast<double>(mod->area);
            movableAreas.push_back(a);
            movableTotalArea += a;
        }
    }

    if (movableAreas.empty()) {
        std::cout << "No movable std cells. Skip filler creation.\n";
        return;
    }

    // 3) 中间 90% 估计平均可移动单元面积，抗极端值
    std::sort(movableAreas.begin(), movableAreas.end());
    const int n = static_cast<int>(movableAreas.size());
    int s = static_cast<int>(std::floor(n * 0.05));
    int e = static_cast<int>(std::ceil (n * 0.95));
    s = std::clamp(s, 0, n);
    e = std::clamp(e, 0, n);
    if (s >= e) { s = 0; e = n; }

    double sum = 0.0;
    for (int i = s; i < e; ++i) sum += movableAreas[i];
    const double avgMovableArea = (e > s) ? (sum / (e - s)) : (movableTotalArea / std::max(1, n));

    // 4) 目标密度与所需填充面积
    const double targetDensity = 0.80; // 可做参数
    const double availableArea  = std::max(0.0, rowArea - fixedArea);
    const double requiredMovableArea = targetDensity * availableArea;

    // 已有可移动（标准单元 + 宏块按目标密度计入）
    double placedLikeArea = 0.0;
    for (Module* mod : db->Nodes) {
        if (!mod || mod->isFixed) continue;
        const double a = static_cast<double>(mod->area);
        if (mod->isMacro) placedLikeArea += a * targetDensity; // 宏占位按目标密度参与
        else              placedLikeArea += a;
    }

    double fillerArea = requiredMovableArea - placedLikeArea;
    if (fillerArea <= 0.0) {
        std::cout << "No filler needed (density already satisfied)." << std::endl;
        return;
    }

    // 5) 生成 filler：高度为 siteHeight，宽度=平均标准单元面积 / 高度
    const double fillerHeight = static_cast<double>(db->siteHeight);
    double fillerWidth = (fillerHeight > 0.0) ? (avgMovableArea / fillerHeight) : 1.0;
    if (fillerWidth <= 0.0) fillerWidth = 1.0;

    const double fillerCellArea = fillerWidth * fillerHeight;
    int numFillers = static_cast<int>(std::floor(fillerArea / std::max(1.0, fillerCellArea)));
    numFillers = std::max(0, numFillers);

    if (numFillers == 0) {
        std::cout << "Computed filler count is zero. Skip." << std::endl;
        return;
    }

    std::cout << "Creating " << numFillers << " filler cells..." << std::endl;

    // 6) 随机初始化位置（初始摆放；后续密度/重叠优化会再处理）
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
        filler->isMacro  = false; // 明确不是宏

        filler->center.x = static_cast<float>(dis_x(gen));
        filler->center.y = static_cast<float>(dis_y(gen));

        db->Nodes.push_back(filler);
    }

    std::cout << "Added " << numFillers << " filler cells (each "
              << fillerWidth << " x " << fillerHeight << ")." << std::endl;
}
