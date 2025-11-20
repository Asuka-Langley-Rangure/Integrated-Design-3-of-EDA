#include "myplace.h"
#include "plot.h"

template<typename SpMatType, typename VecType>
VecType solveWithSnapshots(
    const SpMatType &A,
    const VecType   &b,
    std::vector<Module*>      &movableMods,
    PlaceData                 *db,
    std::function<void(const VecType&)> writeBackCoords,
    const std::string         &tag,
    int                         maxIters   = 1000,
    double                      tol        = 1e-8,
    int                         snapshotInterval = 10
){
    const int N = static_cast<int>(b.size());
    VecType x = VecType::Zero(N);

    VecType r      = b - A * x;
    VecType r_hat  = r;
    VecType p      = r;
    VecType Ap     = VecType::Zero(N);
    VecType s      = VecType::Zero(N);
    VecType t      = VecType::Zero(N);

    double rho_old = r_hat.dot(r);
    if (rho_old == 0.0) rho_old = 1e-30;

    for (int k = 0; k < maxIters; ++k) {
        Ap = A * p;

        double dot_rhat_Ap = r_hat.dot(Ap);
        if (dot_rhat_Ap == 0.0) {
            std::cerr << "[BiCGSTAB-" << tag << "] breakdown at iter=" << k << "\n";
            break;
        }

        double alpha = rho_old / dot_rhat_Ap;

        s = r - alpha * Ap;
        if (s.norm() < tol) {
            x += alpha * p;
            // 写回 & 绘图
            writeBackCoords(x);
            PLOTTING::plotPlacement("/home/czs/桌面/placement_project/task6/iteration/" + tag + "_iter" + std::to_string(k), db);
            std::cout << "[BiCGSTAB-" << tag << "] converged at iter " << k
                      << " resid " << s.norm() << "\n";
            break;
        }

        t = A * s;
        double t_dot_s = t.dot(s);
        double t_dot_t = t.dot(t);
        if (t_dot_t == 0.0) {
            std::cerr << "[BiCGSTAB-" << tag << "] breakdown (t·t=0) at iter=" << k << "\n";
            break;
        }

        double omega = t_dot_s / t_dot_t;

        x += alpha * p + omega * s;
        r  = s   - omega * t;

        // 每隔 snapshotInterval 次做一次写回&绘图
        if ((k % snapshotInterval) == 0) {
            writeBackCoords(x);
            PLOTTING::plotPlacement("/home/czs/桌面/placement_project/task6/iteration/" + tag + "_iter" + std::to_string(k), db);
        }

        if (r.norm() < tol) {
            std::cout << "[BiCGSTAB-" << tag << "] converged at iter " << k
                      << " resid " << r.norm() << "\n";
            break;
        }

        double rho_new = r_hat.dot(r);
        if (rho_old == 0.0) {
            std::cerr << "[BiCGSTAB-" << tag << "] breakdown (rho_old=0) at iter=" << k << "\n";
            break;
        }
        double beta = (rho_new / rho_old) * (alpha / omega);

        p = r + beta * (p - omega * Ap);
        rho_old = rho_new;
    }

    // 最终写回
    writeBackCoords(x);
    return x;
}

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
    int MAX_ITERS = 1000;
    double TOL   = 1e-8;

    // 写回 X 方向坐标的函数
    auto writeX = [&](const Eigen::VectorXd &xVec) {
        for (int i = 0; i < (int)movableMods.size(); ++i) {
            movableMods[i]->center.x = xVec[i];
        }
    };
    // 写回 Y 方向坐标的函数
    auto writeY = [&](const Eigen::VectorXd &yVec) {
        for (int i = 0; i < (int)movableMods.size(); ++i) {
            movableMods[i]->center.y = yVec[i];
        }
    };
    // 求解 X
    Eigen::VectorXd X = solveWithSnapshots<SpMat, Eigen::VectorXd>(
        A_x, b_x,
        movableMods, db,
        writeX,
        "X",
        MAX_ITERS,
        TOL,
        1  // 比如每隔20次做一次绘图
    );
    // 求解 Y
    Eigen::VectorXd Y = solveWithSnapshots<SpMat, Eigen::VectorXd>(
        A_y, b_y,
        movableMods, db,
        writeY,
        "Y",
        MAX_ITERS,
        TOL,
        1
    );

    std::cout << "[Init] placement done.\n";
}
