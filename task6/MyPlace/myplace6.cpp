#include "myplace.h"
#include "plot.h"

void MyPlacer::initialPlacement()
{
    using SpMat   = Eigen::SparseMatrix<double>;
    using Triplet = Eigen::Triplet<double>;
    using Vec     = Eigen::VectorXd;

    // -------------------- 0. 收集可移动模块并建立索引（只为非终端建变量） --------------------
    std::vector<Module*> movableMods;
    movableMods.reserve(db->Nodes.size());

    std::unordered_map<Module*, int> modToIndex;
    modToIndex.reserve(db->Nodes.size());

    for (Module* m : db->Nodes) {
        if (!m) continue;
        if (m->isFixed) continue;  // 终端、暗节点等不进入未知数

        int idx = static_cast<int>(movableMods.size());
        movableMods.push_back(m);
        modToIndex[m] = idx;
    }

    const int N = static_cast<int>(movableMods.size());
    if (N == 0) {
        std::cout << "[Init] No movable modules. Skip.\n";
        return;
    }

    // -------------------- 1. 初始化：把所有可移动单元中心移到 CoreRegion 中心 --------------------
    const double core_llx = db->chipRegion.ll.x;
    const double core_lly = db->chipRegion.ll.y;
    const double core_urx = db->chipRegion.ur.x;
    const double core_ury = db->chipRegion.ur.y;
    const double core_cx  = 0.5 * (core_llx + core_urx);
    const double core_cy  = 0.5 * (core_lly + core_ury);

    for (Module* m : movableMods) {
        m->center.x = static_cast<float>(core_cx);
        m->center.y = static_cast<float>(core_cy);
    }

    // -------------------- 2. 迭代求解：w 中包含 x，需要多次迭代 --------------------
    const int    maxOuterIters = 200;       // 可按需要调小/调大；PDF 中也是“迭代直到误差足够小或到上界”
    const double distMin       = 25.0;    // distanceX / distanceY 的下界（参考 PDF 中 distanceX<25→25）
    const double lambdaDiag    = 1e-8;    // 小对角正则，保证 A 数值稳定
    const double cgTol         = 1e-8;    // BiCGSTAB 收敛阈值

    struct PinInfo {
        Pin*    pin;
        Module* mod;
        bool    movable;
        int     varIdx;   // 若 movable==true，则为未知数下标；否则为 -1
        double  absX;
        double  absY;
        double  offX;
        double  offY;
        bool    boundX;
        bool    boundY;
    };

    for (int iter = 0; iter < maxOuterIters; ++iter) {
        std::cout << "------------------------ InitPlacement Iteration "
                  << (iter + 1) << " ------------------------\n";

        // 2.1 为当前迭代构造 A_x, A_y 以及 RHS b_x, b_y
        std::vector<Triplet> tripX, tripY;
        tripX.reserve(N * 16);
        tripY.reserve(N * 16);

        Vec rhs_x = Vec::Zero(N);  // 这里直接构造的是等式 A x = rhs_x（已等价于 -b/2 的形式）
        Vec rhs_y = Vec::Zero(N);

        std::size_t totalPairs = 0;

        // 遍历每一条网
        for (Net* net : db->Nets) {
            if (!net) continue;
            const auto& netPins = net->netPins;
            const int P = static_cast<int>(netPins.size());
            if (P < 2) continue; 

            // 2.1.1 收集本 net 的所有 pin 信息
            std::vector<PinInfo> pins;
            pins.reserve(P);

            for (Pin* p : netPins) {
                if (!p) continue;
                Module* m = p->module;
                if (!m) continue;

                PinInfo info;
                info.pin     = p;
                info.mod     = m;
                info.movable = !m->isFixed;
                if (info.movable) {
                    auto it = modToIndex.find(m);
                    if (it == modToIndex.end()) {
                        // 不在 movable 列表里，当作固定端处理
                        info.movable = false;
                        info.varIdx  = -1;
                    } else {
                        info.varIdx = it->second;
                    }
                } else {
                    info.varIdx = -1;
                }

                info.offX = p->offset.x;
                info.offY = p->offset.y;

                // pin 绝对坐标 = 模块中心 + offset（PDF 中强调的计算方式）
                const double cx = m->center.x;
                const double cy = m->center.y;
                info.absX = cx + info.offX;
                info.absY = cy + info.offY;

                info.boundX = false;
                info.boundY = false;

                pins.push_back(info);
            }

            const int P2 = static_cast<int>(pins.size());
            if (P2 < 2) continue;

            // 2.1.2 找出 x / y 方向的边界 pin（Bound2Bound 模型）
            double minX = pins[0].absX, maxX = pins[0].absX;
            double minY = pins[0].absY, maxY = pins[0].absY;
            for (int k = 1; k < P2; ++k) {
                if (pins[k].absX < minX) minX = pins[k].absX;
                if (pins[k].absX > maxX) maxX = pins[k].absX;
                if (pins[k].absY < minY) minY = pins[k].absY;
                if (pins[k].absY > maxY) maxY = pins[k].absY;
            }
            const double epsBound = 1e-6;
            for (int k = 0; k < P2; ++k) {
                if (std::fabs(pins[k].absX - minX) <= epsBound ||
                    std::fabs(pins[k].absX - maxX) <= epsBound) {
                    pins[k].boundX = true;
                }
                if (std::fabs(pins[k].absY - minY) <= epsBound ||
                    std::fabs(pins[k].absY - maxY) <= epsBound) {
                    pins[k].boundY = true;
                }
            }

            // 2.1.3 枚举该 net 中的所有 pin 对 (p,q)，构造 Bound2Bound 二次项
            const double denomBase = static_cast<double>(P2 - 1);
            if (denomBase <= 0.0) continue;

            for (int a = 0; a < P2; ++a) {
                const PinInfo& pa = pins[a];
                for (int b = a + 1; b < P2; ++b) {
                    const PinInfo& pb = pins[b];

                    // X 方向权重：w_x = 1 / ((P-1) * max(|xp-xq|, 25))
                    double w_x = 0.0;
                    {
                        double distX = std::fabs(pa.absX - pb.absX);
                        if (distX < distMin) distX = distMin;
                        if (pa.boundX || pb.boundX) {
                            w_x = 1.0 / (denomBase * distX);
                        } else {
                            // 两端都是 inner pin，不参与 Bound2Bound（权重为 0）
                            w_x = 0.0;
                        }
                    }

                    // Y 方向权重：同理
                    double w_y = 0.0;
                    {
                        double distY = std::fabs(pa.absY - pb.absY);
                        if (distY < distMin) distY = distMin;
                        if (pa.boundY || pb.boundY) {
                            w_y = 1.0 / (denomBase * distY);
                        } else {
                            w_y = 0.0;
                        }
                    }

                    const bool aMov = pa.movable;
                    const bool bMov = pb.movable;
                    const int  ia   = pa.varIdx;
                    const int  ib   = pb.varIdx;

                    // --------- 三种情形分别填充 A 和 b（x / y 两个方向独立） ---------

                    // ① p、q 均为可移动模块
                    if (aMov && bMov) {
                        // X 方向
                        if (w_x > 0.0) {
                            // A[i][i]+=w, A[j][j]+=w, A[i][j]-=w, A[j][i]-=w
                            tripX.emplace_back(ia, ia,  w_x);
                            tripX.emplace_back(ib, ib,  w_x);
                            tripX.emplace_back(ia, ib, -w_x);
                            tripX.emplace_back(ib, ia, -w_x);

                            // RHS（等价于 -b/2 的形式）：考虑 pin offset
                            const double ddx = pa.offX - pb.offX;
                            rhs_x(ia) -= w_x * ddx;
                            rhs_x(ib) += w_x * ddx;
                        }
                        // Y 方向
                        if (w_y > 0.0) {
                            tripY.emplace_back(ia, ia,  w_y);
                            tripY.emplace_back(ib, ib,  w_y);
                            tripY.emplace_back(ia, ib, -w_y);
                            tripY.emplace_back(ib, ia, -w_y);

                            const double ddy = pa.offY - pb.offY;
                            rhs_y(ia) -= w_y * ddy;
                            rhs_y(ib) += w_y * ddy;
                        }
                    }
                    // ② p 可移动，q 为 terminal（固定）
                    else if (aMov && !bMov) {
                        // X 方向
                        if (w_x > 0.0) {
                            tripX.emplace_back(ia, ia, w_x);
                            const double c = pb.absX;  // terminal 端 pin 的绝对 x
                            rhs_x(ia) += w_x * (c - pa.offX);
                        }
                        // Y 方向
                        if (w_y > 0.0) {
                            tripY.emplace_back(ia, ia, w_y);
                            const double c = pb.absY;  // terminal 端 pin 的绝对 y
                            rhs_y(ia) += w_y * (c - pa.offY);
                        }
                    }
                    // ③ p 为 terminal，q 可移动
                    else if (!aMov && bMov) {
                        // X 方向
                        if (w_x > 0.0) {
                            tripX.emplace_back(ib, ib, w_x);
                            const double c = pa.absX;
                            rhs_x(ib) += w_x * (c - pb.offX);
                        }
                        // Y 方向
                        if (w_y > 0.0) {
                            tripY.emplace_back(ib, ib, w_y);
                            const double c = pa.absY;
                            rhs_y(ib) += w_y * (c - pb.offY);
                        }
                    }
                    // 两端都是 terminal：对未知数无贡献，忽略

                    ++totalPairs;
                }
            }
        } // end for each net

        std::cout << "[Init] Iteration " << (iter + 1)
                  << " assembled " << totalPairs << " pin pairs.\n";

        // 2.2 构建稀疏矩阵 A_x, A_y，并按 PDF 要求加微小对角项
        SpMat A_x(N, N), A_y(N, N);
        A_x.setFromTriplets(tripX.begin(), tripX.end(), std::plus<double>());
        A_y.setFromTriplets(tripY.begin(), tripY.end(), std::plus<double>());

        A_x.makeCompressed();
        A_y.makeCompressed();

        if (lambdaDiag > 0.0) {
            for (int i = 0; i < N; ++i) {
                A_x.coeffRef(i, i) += lambdaDiag;
                A_y.coeffRef(i, i) += lambdaDiag;
            }
        }

        // 2.3 使用 BiCGSTAB 求解 A_x x = rhs_x 和 A_y y = rhs_y
        Eigen::BiCGSTAB<SpMat, Eigen::DiagonalPreconditioner<double>> solver;
        solver.setTolerance(cgTol);
        solver.setMaxIterations(2000);

        // 解 X
        solver.compute(A_x);
        if (solver.info() != Eigen::Success) {
            std::cerr << "[Init] BiCGSTAB factorization failed for A_x.\n";
            break;
        }
        Vec sol_x = solver.solve(rhs_x);
        if (solver.info() != Eigen::Success) {
            std::cerr << "[Init] BiCGSTAB solve failed for X. iter = "
                      << solver.iterations() << ", err = " << solver.error() << "\n";
            break;
        }

        // 解 Y
        solver.compute(A_y);
        if (solver.info() != Eigen::Success) {
            std::cerr << "[Init] BiCGSTAB factorization failed for A_y.\n";
            break;
        }
        Vec sol_y = solver.solve(rhs_y);
        if (solver.info() != Eigen::Success) {
            std::cerr << "[Init] BiCGSTAB solve failed for Y. iter = "
                      << solver.iterations() << ", err = " << solver.error() << "\n";
            break;
        }

        // 2.4 把解回写到模块中心，并按 PDF 给的代码进行 CoreRegion 边界裁剪
        const double EPS = 1e-3;
        for (int i = 0; i < N; ++i) {
            Module* m = movableMods[i];
            double  x = sol_x[i];
            double  y = sol_y[i];

            if (x - 0.5 * m->width < core_llx) {
                x = core_llx + 0.5 * m->width + EPS;
            }
            if (x + 0.5 * m->width > core_urx) {
                x = core_urx - 0.5 * m->width - EPS;
            }
            if (y - 0.5 * m->height < core_lly) {
                y = core_lly + 0.5 * m->height + EPS;
            }
            if (y + 0.5 * m->height > core_ury) {
                y = core_ury - 0.5 * m->height - EPS;
            }

            m->center.x = static_cast<float>(x);
            m->center.y = static_cast<float>(y);
        }

        // 如需完全对齐参考 PDF 的 BMP 输出，可以在这里调用一次绘图：
        // PLOTTING::plotPlacement("Initial_placement_iteration_" + std::to_string(iter + 1), db);
    }

    std::cout << "[Init] initialPlacement finished.\n";
}