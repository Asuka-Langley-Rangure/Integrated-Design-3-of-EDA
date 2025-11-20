#include "myplace.h"
#include "plot.h"

void MyPlacer::initialPlacement()
{
    using SpMat = Eigen::SparseMatrix<double>;
    using Vec   = Eigen::VectorXd;
    using Trip  = Eigen::Triplet<double>;

    std::cout << "\n==============================\n";
    std::cout << " Kraftwerk2 Initial Placement\n";
    std::cout << "==============================\n";

    // -------------------------------------------------------------
    // Step 1. Collect movable modules
    // -------------------------------------------------------------
    std::vector<Module*> movable;
    std::unordered_map<Module*,int> id;
    movable.reserve(db->Nodes.size());

    for (Module* m : db->Nodes)
    {
        if (m && !m->isFixed) {
            id[m] = movable.size();
            movable.push_back(m);
        }
    }
    int N = movable.size();
    if (N == 0) return;

    // =============================================================
    // ★★★ 所有可移动节点放到 Die 中心 ★★★
    // =============================================================
    {
        double llx = db->chipRegion.ll.x;
        double lly = db->chipRegion.ll.y;
        double urx = db->chipRegion.ur.x;
        double ury = db->chipRegion.ur.y;

        double cx = (llx + urx) / 2.0;
        double cy = (lly + ury) / 2.0;

        // 可选：微扰避免奇异矩阵（重叠会导致 C 矩阵 rank drop）
        std::mt19937 rng(12345);
        std::uniform_real_distribution<double> rx(-1.0, +1.0);
        std::uniform_real_distribution<double> ry(-1.0, +1.0);

        for (Module* m : movable) {
            m->center.x = cx + rx(rng);
            m->center.y = cy + ry(rng);
        }

        std::cout << "[Init] All movable modules placed at chip center (safe init).\n";
    }

    // -------------------------------------------------------------
    // Step 2. Build Cx, Cy, dx, dy using Bound2Bound model
    // -------------------------------------------------------------
    std::vector<Trip> tripX, tripY;
    tripX.reserve(N * 20);
    tripY.reserve(N * 20);

    Vec dx = Vec::Zero(N);
    Vec dy = Vec::Zero(N);

    const double lambda = 1e-6;  // small diagonal for stability

    auto add_diag = [&](int i, double w) {
        tripX.emplace_back(i, i, w);
        tripY.emplace_back(i, i, w);
    };

    auto add_edge = [&](int i, int j, double w, double dx_ij, double dy_ij) {
        // X
        tripX.emplace_back(i,i, w);
        tripX.emplace_back(j,j, w);
        tripX.emplace_back(i,j,-w);
        tripX.emplace_back(j,i,-w);
        dx(i) += w * dx_ij;
        dx(j) -= w * dx_ij;

        // Y
        tripY.emplace_back(i,i, w);
        tripY.emplace_back(j,j, w);
        tripY.emplace_back(i,j,-w);
        tripY.emplace_back(j,i,-w);
        dy(i) += w * dy_ij;
        dy(j) -= w * dy_ij;
    };

    // ---------- loop nets ----------
    for (Net* net : db->Nets)
    {
        if (!net || net->netPins.size() < 2) continue;

        // Collect movable & fixed endpoints
        struct Mov { int idx; double offx, offy; };
        struct Fix { double x,y, offx,offy; };

        std::vector<Mov> mov;
        std::vector<Fix> fix;

        for (Pin* p : net->netPins)
        {
            Module* m = p->module;
            if (!m) continue;

            if (m->isFixed) {
                fix.push_back({ m->center.x, m->center.y, p->offset.x, p->offset.y });
            } else {
                auto it = id.find(m);
                if (it != id.end())
                    mov.push_back({ it->second, p->offset.x, p->offset.y });
            }
        }

        int km = mov.size(), kf = fix.size();
        int k  = km + kf;
        if (k < 2 || km == 0) continue;

        // B2B weight (simple approximation)
        double w = 1.0 / std::max(1, k - 1);

        // Clique model is acceptable for initial placement
        for (int a = 0; a < km; ++a)
            for (int b = a + 1; b < km; ++b)
                add_edge(mov[a].idx, mov[b].idx, w,
                         mov[b].offx - mov[a].offx,
                         mov[b].offy - mov[a].offy);

        for (int a = 0; a < km; ++a)
        {
            int ia = mov[a].idx;
            for (auto &f : fix)
            {
                add_diag(ia, w);
                dx(ia) += w * ((f.x + f.offx) - mov[a].offx);
                dy(ia) += w * ((f.y + f.offy) - mov[a].offy);
            }
        }
    }

    // tiny diagonal for all nodes
    for (int i = 0; i < N; i++) add_diag(i, lambda);

    // -------------------------------------------------------------
    // Step 3. Build sparse matrices
    // -------------------------------------------------------------
    SpMat Cx(N,N), Cy(N,N);
    Cx.setFromTriplets(tripX.begin(), tripX.end(), std::plus<double>());
    Cy.setFromTriplets(tripY.begin(), tripY.end(), std::plus<double>());
    Cx.makeCompressed();
    Cy.makeCompressed();

    // -------------------------------------------------------------
    // Step 4. Iteratively solve with step damping & boundary clamp
    // -------------------------------------------------------------
    int    K   = 25;          // 外层迭代次数
    double tol = 1e-6;

    Vec X(N), Y(N);
    for (int i = 0; i < N; ++i) {
        X[i] = movable[i]->center.x;
        Y[i] = movable[i]->center.y;
    }

    // 芯片边界 & 目标单步最大位移
    double llx = db->chipRegion.ll.x;
    double lly = db->chipRegion.ll.y;
    double urx = db->chipRegion.ur.x;
    double ury = db->chipRegion.ur.y;
    double chipW = urx - llx;
    double chipH = ury - lly;

    // 每一轮允许的最大位移（可以自己调，比如 5%~10% 芯片尺寸）
    double targetStep = 0.10 * std::max(chipW, chipH);

    Eigen::ConjugateGradient<SpMat, Eigen::Lower|Eigen::Upper> solverX, solverY;
    solverX.setMaxIterations(1);
    solverX.setTolerance(tol);

    solverY.setMaxIterations(1);
    solverY.setTolerance(tol);

    solverX.compute(Cx);
    solverY.compute(Cy);

    for (int it = 1; it <= K; ++it)
    {
        // 1) 用当前 X,Y 作为初始猜测，做 1 步 CG
        Vec X_new = solverX.solveWithGuess(-dx, X);
        Vec Y_new = solverY.solveWithGuess(-dy, Y);

        // 2) 计算本轮“原始”最大位移
        double maxStep = 0.0;
        for (int i = 0; i < N; ++i) {
            double sx = X_new[i] - X[i];
            double sy = Y_new[i] - Y[i];
            double s2 = sx * sx + sy * sy;
            if (s2 > maxStep) maxStep = s2;
        }
        maxStep = std::sqrt(maxStep);

        // 3) 计算缩放因子 alpha：控制单步位移不超过 targetStep
        double alpha = 1.0;
        if (maxStep > 1e-9 && maxStep > targetStep) {
            alpha = targetStep / maxStep;   // 把这一步整体缩小
        }

        // 4) 应用缩放后的位移，并把结果 clamp 到芯片边界内
        for (int i = 0; i < N; ++i) {
            double sx = X_new[i] - X[i];
            double sy = Y_new[i] - Y[i];

            double x = X[i] + alpha * sx;
            double y = Y[i] + alpha * sy;

            // 给一点边界 margin，避免模块一半在芯片外
            double halfW = movable[i]->width  * 0.5;
            double halfH = movable[i]->height * 0.5;

            x = std::max(llx + halfW, std::min(x, urx - halfW));
            y = std::max(lly + halfH, std::min(y, ury - halfH));

            X[i] = x;
            Y[i] = y;

            movable[i]->center.x = x;
            movable[i]->center.y = y;
        }

        // 5) 画图 + 打印信息
        PLOTTING::plotPlacement("iteration/init_iter_" + std::to_string(it), db);

        std::cout << "[Init] Iter " << it
                << "  resX=" << solverX.error()
                << "  resY=" << solverY.error()
                << "  alpha=" << alpha
                << "  maxStep=" << maxStep
                << "\n";

        // 6) 如果单步位移已经非常小，可以提前退出
        if (maxStep * alpha < 1e-3) {
            std::cout << "[Init] Movement small, early stop at iter " << it << "\n";
            break;
        }
    }

    std::cout << "[Init] Initial-placement completed.\n";

}