#include "myplace.h"
#include "plot.h"


void MyPlacer::run() {
    std::cout << "=== Global Initial Placement ===\n";
    initialPlacement_Kraftwerk2(50);

}

void MyPlacer::initialPlacement_Kraftwerk2(int maxOuter /*=50*/)
{
    using SpMat = Eigen::SparseMatrix<double>;
    using Vec   = Eigen::VectorXd;
    using Trip  = Eigen::Triplet<double>;

    // ============================================================
    // 1. Collect movable modules
    // ============================================================
    std::vector<Module*> movable;
    std::unordered_map<Module*, int> id;

    for (Module* m : db->Nodes) {
        if (m && !m->isFixed) {
            id[m] = movable.size();
            movable.push_back(m);
        }
    }
    const int N = movable.size();
    if (N == 0) {
        std::cout << "[KW2] No movable modules.\n";
        return;
    }

    // Solution vectors
    Vec X(N), Y(N);
    for (int i = 0; i < N; i++) {
        X[i] = movable[i]->center.x;
        Y[i] = movable[i]->center.y;
    }

    //------------------------------------------------------------
    // ★ Step0: Safe Initialization（保证可见）
    //------------------------------------------------------------
    {
        double llx = db->chipRegion.ll.x;
        double lly = db->chipRegion.ll.y;
        double urx = db->chipRegion.ur.x;
        double ury = db->chipRegion.ur.y;

        double cx = (llx + urx) / 2.0;
        double cy = (lly + ury) / 2.0;

        std::mt19937 rng(123456);
        std::uniform_real_distribution<double> rx(-0.05*(urx-llx), 0.05*(urx-llx));
        std::uniform_real_distribution<double> ry(-0.05*(ury-lly), 0.05*(ury-lly));

        for (Module* m : movable) {
            m->center.x = cx + rx(rng);
            m->center.y = cy + ry(rng);
        }

        std::cout << "[KW2] Safe initialization done.\n";
    }


    // ============================================================
    // 2. Kraftwerk2 Main Loop
    // ============================================================
    double mu = 1.0;             // 初始质量
    const double mu_shrink = 0.92; // 每轮衰减（论文建议 0.90~0.95）

    for (int iter = 1; iter <= maxOuter; iter++)
    {
        std::cout << "\n\n[KW2] ===== OUTER ITER " << iter << " =====\n";

        // ---------------------------------------------------------
        // Step A. Build B2B net model
        // ---------------------------------------------------------
        SpMat Cx, Cy;
        Vec dx, dy;
        buildB2BMatrix(Cx, Cy, dx, dy, id);

        // ---------------------------------------------------------
        // Step B. Compute Move Force (Poisson Potential)
        // ---------------------------------------------------------
        SpMat Cmove;
        Vec mx, my;
        computeMoveForce(Cmove, mx, my, movable);

        // ---------------------------------------------------------
        // Step C. Hold Force（维持数值平稳）
        //     C_hold = mu * I
        // ---------------------------------------------------------
        SpMat Chold(N, N);
        std::vector<Trip> th;
        th.reserve(N);
        for (int i = 0; i < N; i++)
            th.emplace_back(i, i, mu);

        Chold.setFromTriplets(th.begin(), th.end());
        Chold.makeCompressed();

        // ---------------------------------------------------------
        // Step D. Assemble total matrix:
        //     C_tot = C + Cmove + Chold
        // ---------------------------------------------------------
        SpMat Ctot = Cx + Cmove + Chold;

        // RHS:
        //   rhs = -d + m + mu * X_old
        // ---------------------------------------------------------
        Vec rhs_x = -dx + mx + mu * X;
        Vec rhs_y = -dy + my + mu * Y;

        // ---------------------------------------------------------
        // Step E. Solve using BiCGSTAB (one step)
        // ---------------------------------------------------------
        Eigen::BiCGSTAB<SpMat, Eigen::DiagonalPreconditioner<double>> solver;

        solver.compute(Ctot);
        solver.setMaxIterations(1);     // 每次迭代只做一步！（Kraftwerk 核心）
        solver.setTolerance(1e-12);

        X = solver.solveWithGuess(rhs_x, X);
        Y = solver.solveWithGuess(rhs_y, Y);

        // ---------------------------------------------------------
        // Step F. Write back to DB
        // ---------------------------------------------------------
        for (int i = 0; i < N; i++) {
            movable[i]->center.x = X[i];
            movable[i]->center.y = Y[i];
        }

        // ---------------------------------------------------------
        // Step G. Quality control（Kraftwerk Section VII）
        //
        // 控制质量参数 μ（模拟弹簧阻尼）
        //   μ ← μ * shrink
        // ---------------------------------------------------------
        mu *= mu_shrink;

        // ---------------------------------------------------------
        // Step H. Plot iteration
        // ---------------------------------------------------------
        PLOTTING::plotPlacement("iteration/kw2_iter_" + std::to_string(iter), db);

        std::cout << "[KW2] Iter=" << iter 
                  << "   mu=" << mu 
                  << "   solverRes=" << solver.error() << "\n";
    }

    std::cout << "\n[KW2] Initial Placement Completed.\n";
}

// ======================================================================
//  Kraftwerk2: Build Bound-to-Bound (B2B) Net Model
//  输出：Cx, Cy, dx, dy
// ======================================================================
void MyPlacer::buildB2BMatrix(
    Eigen::SparseMatrix<double>& Cx,
    Eigen::SparseMatrix<double>& Cy,
    Eigen::VectorXd& dx,
    Eigen::VectorXd& dy,
    const std::unordered_map<Module*, int>& id
)
{
    using SpMat = Eigen::SparseMatrix<double>;
    using Trip  = Eigen::Triplet<double>;

    const int N = id.size();
    dx = Eigen::VectorXd::Zero(N);
    dy = Eigen::VectorXd::Zero(N);

    std::vector<Trip> tx, ty;
    tx.reserve(N * 20);
    ty.reserve(N * 20);

    // -----------------------------------------------------------
    // For each net: find boundary pins (minX, maxX, minY, maxY)
    // B2B model = connect only boundary <-> boundary or boundary<->inner
    // -----------------------------------------------------------
    for (Net* net : db->Nets)
    {
        if (!net || net->netPins.size() < 2) continue;
        auto& pins = net->netPins;
        int P = pins.size();

        // ----------------------
        // 1. 找 boundary pin
        // ----------------------
        double min_x = 1e99, max_x = -1e99;
        double min_y = 1e99, max_y = -1e99;

        for (Pin* p : pins) {
            if (!p || !p->module) continue;
            double px = p->module->center.x + p->offset.x;
            double py = p->module->center.y + p->offset.y;
            min_x = std::min(min_x, px);
            max_x = std::max(max_x, px);
            min_y = std::min(min_y, py);
            max_y = std::max(max_y, py);
        }

        // ----------------------
        // 2. 标记 boundary pin
        // ----------------------
        struct EndPoint {
            Module* m;
            double  offx, offy;
            double  px, py;
            bool isBoundary;
        };

        std::vector<EndPoint> ep;
        ep.reserve(P);

        for (Pin* p : pins) {
            if (!p || !p->module) continue;
            Module* m = p->module;

            EndPoint e;
            e.m = m;
            e.offx = p->offset.x;
            e.offy = p->offset.y;
            e.px   = m->center.x + p->offset.x;
            e.py   = m->center.y + p->offset.y;

            // boundary 判断（Kraftwerk2）
            e.isBoundary =
                (std::abs(e.px - min_x) < 1e-9) ||
                (std::abs(e.px - max_x) < 1e-9) ||
                (std::abs(e.py - min_y) < 1e-9) ||
                (std::abs(e.py - max_y) < 1e-9);

            ep.push_back(e);
        }

        // ----------------------
        // 3. B2B 权重：只连 boundary→boundary / boundary→inner
        // ----------------------
        for (int a = 0; a < ep.size(); a++)
        for (int b = a + 1; b < ep.size(); b++)
        {
            auto& pa = ep[a];
            auto& pb = ep[b];

            // Skip inner-inner 连接（Kraftwerk2 B2B 核心）
            if (!pa.isBoundary && !pb.isBoundary) continue;

            // 坐标差
            double dx_now = pa.px - pb.px;
            double dy_now = pa.py - pb.py;

            // 权重：1/|x_p - x_q| + 1/|y_p - y_q|
            double wx = 1.0 / std::max(1e-3, std::abs(dx_now));
            double wy = 1.0 / std::max(1e-3, std::abs(dy_now));

            // Two-movable
            bool aMov = (id.count(pa.m) > 0);
            bool bMov = (id.count(pb.m) > 0);

            // ----------------------------------------------------------------
            // case 1: both movable → 2×2 Laplacian block
            // ----------------------------------------------------------------
            if (aMov && bMov)
            {
                int ia = id.at(pa.m);
                int ib = id.at(pb.m);

                // X
                tx.emplace_back(ia, ia, wx);
                tx.emplace_back(ib, ib, wx);
                tx.emplace_back(ia, ib, -wx);
                tx.emplace_back(ib, ia, -wx);

                // RHS_x
                double ddx = (pb.offx - pa.offx);
                dx[ia] += wx * ddx;
                dx[ib] -= wx * ddx;

                // Y
                ty.emplace_back(ia, ia, wy);
                ty.emplace_back(ib, ib, wy);
                ty.emplace_back(ia, ib, -wy);
                ty.emplace_back(ib, ia, -wy);

                double ddy = (pb.offy - pa.offy);
                dy[ia] += wy * ddy;
                dy[ib] -= wy * ddy;
            }
            // ----------------------------------------------------------------
            // case 2: a movable, b fixed
            // ----------------------------------------------------------------
            else if (aMov && !bMov)
            {
                int ia = id.at(pa.m);

                tx.emplace_back(ia, ia, wx);
                ty.emplace_back(ia, ia, wy);

                dx[ia] += wx * (pb.px - pa.offx);
                dy[ia] += wy * (pb.py - pa.offy);
            }
            // ----------------------------------------------------------------
            // case 3: a fixed, b movable
            // ----------------------------------------------------------------
            else if (!aMov && bMov)
            {
                int ib = id.at(pb.m);

                tx.emplace_back(ib, ib, wx);
                ty.emplace_back(ib, ib, wy);

                dx[ib] += wx * (pa.px - pb.offx);
                dy[ib] += wy * (pa.py - pb.offy);
            }
        }
    }

    // -----------------------------------------------------------
    // 4. 构建稀疏矩阵
    // -----------------------------------------------------------
    Cx.resize(N, N);
    Cy.resize(N, N);

    Cx.setFromTriplets(tx.begin(), tx.end(), std::plus<double>());
    Cy.setFromTriplets(ty.begin(), ty.end(), std::plus<double>());

    Cx.makeCompressed();
    Cy.makeCompressed();

    std::cout << "[KW2] B2B matrix built. NonZeros(Cx)=" 
              << Cx.nonZeros() << "\n";
}

// ======================================================================
// Kraftwerk2 Move Force (Poisson Potential)
// ======================================================================
void MyPlacer::computeMoveForce(
    Eigen::SparseMatrix<double>& Cmove,
    Eigen::VectorXd& mx,
    Eigen::VectorXd& my,
    const std::vector<Module*>& movable
)
{
    using SpMat = Eigen::SparseMatrix<double>;
    using Trip  = Eigen::Triplet<double>;
    using Vec   = Eigen::VectorXd;

    const int N = movable.size();

    // -------------------------
    // 1. density grid
    // -------------------------
    const int G = 64;
    double llx = db->chipRegion.ll.x;
    double lly = db->chipRegion.ll.y;
    double urx = db->chipRegion.ur.x;
    double ury = db->chipRegion.ur.y;

    double W = urx - llx;
    double H = ury - lly;

    double gx = W / G;
    double gy = H / G;

    double binArea = gx * gy;

    std::vector<double> density(G*G, 0.0);

    // -------------------------
    // 2. 把模块面积 / binArea 加入 density
    // -------------------------
    for (auto* m : movable)
    {
        double x = m->center.x;
        double y = m->center.y;

        int ix = std::clamp(int((x-llx)/gx), 0, G-1);
        int iy = std::clamp(int((y-lly)/gy), 0, G-1);

        density[iy*G + ix] += m->area / binArea;
    }

    // -------------------------
    // 3. target density = lambda
    // -------------------------
    double lambda = 0.8;  // 目标密度

    // RHS = density - lambda
    Vec rhs(G*G);
    for (int i = 0; i < G*G; i++)
        rhs[i] = density[i] - lambda;

    // -------------------------
    // 4. 构建 Poisson Laplacian
    // -------------------------
    std::vector<Trip> Lt;
    Lt.reserve(G*G*5);

    auto idCell = [&](int r,int c){return r*G+c;};

    for (int r=0;r<G;r++)
    for (int c=0;c<G;c++)
    {
        int id = idCell(r,c);
        int deg = 0;

        auto add = [&](int rr,int cc){
            if(rr<0||rr>=G||cc<0||cc>=G) return;
            Lt.emplace_back(id, idCell(rr,cc), -1.0);
            deg++;
        };

        add(r-1,c);
        add(r+1,c);
        add(r,c-1);
        add(r,c+1);

        Lt.emplace_back(id,id, deg);
    }

    SpMat L(G*G, G*G);
    L.setFromTriplets(Lt.begin(), Lt.end());
    L.makeCompressed();

    // -------------------------
    // 5. solve L * Phi = rhs
    // -------------------------
    Eigen::ConjugateGradient<SpMat> cg;
    cg.setTolerance(1e-6);
    cg.compute(L);

    Vec Phi = cg.solve(rhs);

    // -------------------------
    // 6. compute gradient
    // -------------------------
    std::vector<double> gradX(G*G,0), gradY(G*G,0);

    for (int r=1;r<G-1;r++)
    for (int c=1;c<G-1;c++)
    {
        int id = idCell(r,c);
        gradX[id] = (Phi[idCell(r,c+1)] - Phi[idCell(r,c-1)])/(2*gx);
        gradY[id] = (Phi[idCell(r+1,c)] - Phi[idCell(r-1,c)])/(2*gy);
    }

    // -------------------------
    // 7. Map gradient to module move forces
    // -------------------------
    mx = Vec::Zero(N);
    my = Vec::Zero(N);

    double beta = 5.0;  // move force scale

    for (int i=0;i<N;i++)
    {
        Module* m = movable[i];

        int ix = std::clamp(int((m->center.x - llx)/gx), 0, G-1);
        int iy = std::clamp(int((m->center.y - lly)/gy), 0, G-1);

        int id = iy*G + ix;

        mx[i] = -beta * gradX[id] * m->area;
        my[i] = -beta * gradY[id] * m->area;
    }

    // -------------------------
    // 8. Cmove = βI
    // -------------------------
    std::vector<Trip> tm;
    tm.reserve(N);

    for (int i=0;i<N;i++)
        tm.emplace_back(i,i, beta);

    Cmove.resize(N,N);
    Cmove.setFromTriplets(tm.begin(),tm.end());
    Cmove.makeCompressed();

    std::cout << "[KW2] Move force computed.\n";
}
