#include "myplace.h"
#include "plot.h"

void MyPlacer::initialPlacement() {
    using SpMat = Eigen::SparseMatrix<double>;
    using Trip  = Eigen::Triplet<double>;

    // -------------------- Step 1. 收集可移动模块 --------------------
    std::vector<Module*> movableMods;
    std::unordered_map<Module*, int> modToIndex;
    movableMods.reserve(db->Nodes.size());
    modToIndex.reserve(db->Nodes.size());

    for (Module* m : db->Nodes) {
        if (m && !m->isFixed) {
            modToIndex[m] = static_cast<int>(movableMods.size());
            movableMods.push_back(m);
        }
    }
    const int N = static_cast<int>(movableMods.size());
    if (N == 0) {
        std::cout << "[Init] No movable modules. Skip.\n";
        return;
    }

    // -------------------- Step 2. 预分配矩阵与向量 --------------------
    std::vector<Trip> tripX, tripY;
    tripX.reserve(N * 12);
    tripY.reserve(N * 12);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(N);

    constexpr int    K_CLIQUE_MAX = 8;
    constexpr int    K_SAMPLE     = 4;
    constexpr double LAMBDA_DIAG  = 1e-6;

    auto add_diag = [&](int i, double v) {
        tripX.emplace_back(i, i, v);
        tripY.emplace_back(i, i, v);
    };

    auto add_edge = [&](int i, int j, double w, double dx, double dy) {
        // X 轴
        tripX.emplace_back(i, i,  w); tripX.emplace_back(j, j,  w);
        tripX.emplace_back(i, j, -w); tripX.emplace_back(j, i, -w);
        b_x(i) += w * dx;  b_x(j) -= w * dx;
        // Y 轴
        tripY.emplace_back(i, i,  w); tripY.emplace_back(j, j,  w);
        tripY.emplace_back(i, j, -w); tripY.emplace_back(j, i, -w);
        b_y(i) += w * dy;  b_y(j) -= w * dy;
    };

    // -------------------- Step 3. 遍历 nets 构建 A 与 b --------------------
    size_t totalPins = 0;
    for (Net* net : db->Nets) if (net) totalPins += net->netPins.size();
    size_t processedPins = 0;

    for (Net* net : db->Nets) {
        if (!net || net->netPins.size() < 2) continue;

        struct MovEnd { int idx; double offx, offy; };
        struct FixEnd { double x, y, offx, offy; };

        std::vector<MovEnd> mov;
        std::vector<FixEnd> fix;
        mov.reserve(net->netPins.size());
        fix.reserve(net->netPins.size());

        for (Pin* p : net->netPins) {
            if (!p || !p->module) continue;
            Module* m = p->module;
            if (m->isFixed)
                fix.push_back({ m->center.x, m->center.y, p->offset.x, p->offset.y });
            else {
                auto it = modToIndex.find(m);
                if (it != modToIndex.end())
                    mov.push_back({ it->second, p->offset.x, p->offset.y });
            }
        }

        processedPins += net->netPins.size();
        const int km = mov.size(), kf = fix.size(), k = km + kf;
        if (k < 2 || km == 0) continue;

        const double w = 1.0 / std::max(1, k - 1);

        if (k <= K_CLIQUE_MAX || (kf == 0 && km <= K_SAMPLE)) {
            // mov-mov
            for (int a = 0; a < km; ++a)
                for (int b = a + 1; b < km; ++b)
                    add_edge(mov[a].idx, mov[b].idx, w,
                             mov[b].offx - mov[a].offx,
                             mov[b].offy - mov[a].offy);

            // mov-fix
            for (int a = 0; a < km; ++a) {
                const int ia = mov[a].idx;
                for (const auto& f : fix) {
                    add_diag(ia, w);
                    b_x(ia) += w * ((f.x + f.offx) - mov[a].offx);
                    b_y(ia) += w * ((f.y + f.offy) - mov[a].offy);
                }
            }
        } else {
            // 大网
            if (kf > 0) {
                for (const auto& m : mov) {
                    add_diag(m.idx, w * kf);
                    double sumx = 0.0, sumy = 0.0;
                    for (const auto& f : fix) {
                        sumx += (f.x + f.offx - m.offx);
                        sumy += (f.y + f.offy - m.offy);
                    }
                    b_x(m.idx) += w * sumx;
                    b_y(m.idx) += w * sumy;
                }
            } else {
                for (const auto& m : mov)
                    add_diag(m.idx, LAMBDA_DIAG);
            }
        }

        if (processedPins % 10000 == 0) {
            double pct = 100.0 * processedPins / std::max<size_t>(1, totalPins);
            std::cout << "\r[Init] assembling nets ... "
                      << std::fixed << std::setprecision(1) << pct << "%";
            std::cout.flush();
        }
    }
    std::cout << "\r[Init] assembling nets ... 100.0%\n";

    // -------------------- Step 4. 构建矩阵 --------------------
    for (int i = 0; i < N; ++i) add_diag(i, LAMBDA_DIAG);
    SpMat A_x(N, N), A_y(N, N);
    A_x.setFromTriplets(tripX.begin(), tripX.end(), std::plus<double>());
    A_y.setFromTriplets(tripY.begin(), tripY.end(), std::plus<double>());
    A_x.makeCompressed();
    A_y.makeCompressed();

    // -------------------- Step 5. 求解并写回 --------------------
    // （把原来两次独立 solveWithSnapshots 替换掉）
    {
        using SpMat = Eigen::SparseMatrix<double>;
        using Trip  = Eigen::Triplet<double>;
        using Vec   = Eigen::VectorXd;

        // 1) 组块对角 A_blk
        const int N2 = 2 * N;
        std::vector<Trip> tBlk;
        tBlk.reserve(A_x.nonZeros() + A_y.nonZeros());
        // A_x -> (0..N-1, 0..N-1)
        for (int k = 0; k < A_x.outerSize(); ++k)
            for (SpMat::InnerIterator it(A_x, k); it; ++it)
                tBlk.emplace_back(it.row(), it.col(), it.value());
        // A_y -> (N..2N-1, N..2N-1)
        for (int k = 0; k < A_y.outerSize(); ++k)
            for (SpMat::InnerIterator it(A_y, k); it; ++it)
                tBlk.emplace_back(N + it.row(), N + it.col(), it.value());

        SpMat A_blk(N2, N2);
        A_blk.setFromTriplets(tBlk.begin(), tBlk.end(), std::plus<double>());
        A_blk.makeCompressed();

        // 2) 拼 b_blk，初值 Z=[X;Y]
        Vec b_blk(N2); b_blk.head(N) = b_x; b_blk.tail(N) = b_y;
        Vec Z = Vec::Zero(N2);

        // 3) 单个求解器，同步小步推进
        Eigen::BiCGSTAB<SpMat, Eigen::DiagonalPreconditioner<double>> solver;
        solver.setTolerance(1e-12);
        solver.compute(A_blk);

        const int    outer_max   = 600;   // 同步轮数
        const int    delta_iters = 1;     // 每轮只推进 1 步
        const double tol_total   = 1e-8;  // 合并残差阈值（相对/绝对可二选一）

        for (int it = 1; it <= outer_max; ++it) {
            solver.setMaxIterations(delta_iters);
            Z = solver.solveWithGuess(b_blk, Z);   // X/Y 同步推进

            // 写回坐标（一次性）
            const Vec X = Z.head(N), Y = Z.tail(N);
            for (int i = 0; i < N; ++i) {
                movableMods[i]->center.x = X[i];
                movableMods[i]->center.y = Y[i];
            }

            // （可选）阻尼更新，进一步避免“拉丝抖动”
            // double alpha = 0.6;  // 0<alpha<=1
            // movableMods[i]->center.x = (1-alpha)*oldX + alpha*X[i]; // 同理 Y

            // 固定视口绘图（防止每帧缩放变化造成“拉丝错觉”）
            PLOTTING::plotPlacement("iteration/iter_" + std::to_string(it), db);

            // 合并残差收敛： ||A_blk Z - b_blk|| / (||b_blk||+eps)
            double r = (A_blk * Z - b_blk).norm();
            double denom = std::max(1e-12, b_blk.norm());
            if (r / denom <= tol_total) {
                std::cout << "[Sync-Block] Converged at " << it
                        << ", relRes=" << (r/denom) << "\n";
                break;
            }
        }
    }

    std::cout << "[Init] placement done.\n";
}