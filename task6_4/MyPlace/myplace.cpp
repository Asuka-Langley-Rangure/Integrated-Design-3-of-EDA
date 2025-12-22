#include "myplace.h"

void MyPlacer::initialPlacement()
{
    using SpMat   = Eigen::SparseMatrix<double>;
    using Triplet = Eigen::Triplet<double>;
    using Vec     = Eigen::VectorXd;

    // -------------------- 0. 收集可移动模块并建立 idx→变量号 映射 --------------------
    const int numNodes = static_cast<int>(db->Nodes.size());

    std::vector<Module*> movableMods;
    movableMods.reserve(numNodes);

    // 用 Module::idx 直接映射到变量下标，替代 unordered_map<Module*, int>
    std::vector<int> node2varIdx(numNodes, -1);

    for (Module* m : db->Nodes) {
        if (!m) continue;
        if (m->isFixed) continue;  // 终端/固定模块不做变量

        int varIdx = static_cast<int>(movableMods.size());
        movableMods.push_back(m);

        // 假设 m->idx 在 [0, numNodes)
        if (m->idx >= 0 && m->idx < numNodes) {
            node2varIdx[m->idx] = varIdx;
        }
    }

    const int N = static_cast<int>(movableMods.size());
    if (N == 0) {
        std::cout << "[Init] No movable modules. Skip.\n";
        return;
    }

    // -------------------- CoreRegion info --------------------
    // Use coreRegion (density grid) instead of chipRegion to keep coordinates consistent.
    const double core_llx = db->coreRegion.ll.x;
    const double core_lly = db->coreRegion.ll.y;
    const double core_urx = db->coreRegion.ur.x;
    const double core_ury = db->coreRegion.ur.y;
    const double core_cx  = 0.5 * (core_llx + core_urx);
    const double core_cy  = 0.5 * (core_lly + core_ury);

    // -------------------- 1. 初始化：把所有可移动单元放到 CoreRegion 中心 --------------------
    for (Module* m : movableMods) {
        m->center.x = static_cast<float>(core_cx);
        m->center.y = static_cast<float>(core_cy);
    }

    // -------------------- HPWL 计算函数（用于打印） --------------------
    auto computeHPWL = [&](PlaceData* dbPtr) -> double {
        double total = 0.0;
        for (Net* net : dbPtr->Nets) {
            if (!net) continue;
            const auto& netPins = net->netPins;
            const int P = static_cast<int>(netPins.size());
            if (P < 2) continue;

            double minX = 0.0, maxX = 0.0;
            double minY = 0.0, maxY = 0.0;
            bool   first = true;

            for (Pin* p : netPins) {
                if (!p || !p->module) continue;
                Module* m = p->module;
                double cx = m->center.x;
                double cy = m->center.y;
                double px = cx + p->offset.x;
                double py = cy + p->offset.y;

                if (first) {
                    minX = maxX = px;
                    minY = maxY = py;
                    first = false;
                } else {
                    if (px < minX) minX = px;
                    if (px > maxX) maxX = px;
                    if (py < minY) minY = py;
                    if (py > maxY) maxY = py;
                }
            }
            if (!first) {
                total += (maxX - minX) + (maxY - minY);
            }
        }
        return total;
    };

    // -------------------- 2. 迭代求解参数 --------------------
    const int    maxOuterIters = 30;      // 外层最多迭代次数
    const double distMin       = 25.0;    // Bound2Bound 距离下界
    const double lambdaDiag    = 1e-8;    // 小对角正则
    const double cgTol         = 1e-8;    // 迭代解收敛阈值
    const double moveTol       = 1e-2;    // 最大位移收敛阈值
    const double hpwlRelTol    = 1e-3;    // HPWL 相对变化收敛阈值

    struct PinInfo {
        Pin*    pin        = nullptr;
        Module* mod        = nullptr;
        bool    movable    = false;
        int     varIdx     = -1;
        double  absX       = 0.0;
        double  absY       = 0.0;
        double  offX       = 0.0;
        double  offY       = 0.0;
        bool    boundX     = false;
        bool    boundY     = false;
    };

    // 用于收敛判据
    std::vector<double> prevX(N), prevY(N);
    for (int i = 0; i < N; ++i) {
        prevX[i] = movableMods[i]->center.x;
        prevY[i] = movableMods[i]->center.y;
    }
    double prevHPWL = computeHPWL(db);

    // ======= 预分配&复用：矩阵、RHS、triplets、pin缓冲、solver =======
    SpMat A_x(N, N), A_y(N, N);

    Vec rhs_x(N), rhs_y(N);
    rhs_x.setZero();
    rhs_y.setZero();

    std::vector<Triplet> tripX, tripY;
    tripX.reserve(N * 16);
    tripY.reserve(N * 16);

    std::vector<PinInfo> pins;
    pins.reserve(128);  // 初始值，下面会根据 net 大小自动扩容一次

    // 保持原来 BiCGSTAB 的求解流程，只是把 solver 挪到循环外重用内部缓冲
    Eigen::BiCGSTAB<SpMat, Eigen::DiagonalPreconditioner<double>> solver;
    solver.setTolerance(cgTol);
    solver.setMaxIterations(5000);

    for (int iter = 0; iter < maxOuterIters; ++iter) {
        std::cout << "------------------------ InitPlacement Iteration "
                  << (iter + 1) << " ------------------------\n";

        // 每轮清零并复用内存
        tripX.clear();
        tripY.clear();
        rhs_x.setZero();
        rhs_y.setZero();

        std::size_t totalPairs = 0;

        // 遍历每一条网
        for (Net* net : db->Nets) {
            if (!net) continue;
            const auto& netPins = net->netPins;
            const int P = static_cast<int>(netPins.size());
            if (P < 2) continue;

            // 2.1.1 收集本 net 的所有 pin 信息（复用 pins 缓冲）
            pins.clear();
            pins.reserve(P);

            for (Pin* p : netPins) {
                if (!p) continue;
                Module* m = p->module;
                if (!m) continue;

                PinInfo info;
                info.pin  = p;
                info.mod  = m;
                info.offX = p->offset.x;
                info.offY = p->offset.y;

                // 可移动性 & 变量下标
                info.movable = !m->isFixed;
                if (info.movable) {
                    int midx = m->idx;
                    if (midx >= 0 && midx < numNodes) {
                        info.varIdx = node2varIdx[midx];
                    } else {
                        info.varIdx = -1;
                    }
                    if (info.varIdx < 0) {
                        info.movable = false;
                    }
                } else {
                    info.varIdx = -1;
                }

                double cx = m->center.x;
                double cy = m->center.y;
                info.absX = cx + info.offX;
                info.absY = cy + info.offY;

                info.boundX = false;
                info.boundY = false;

                pins.push_back(info);
            }

            const int P2 = static_cast<int>(pins.size());
            if (P2 < 2) continue;

            // 2.1.2 找出 x / y 方向的边界 pin（Bound2Bound）
            double minX = pins[0].absX, maxX = pins[0].absX;
            double minY = pins[0].absY, maxY = pins[0].absY;
            for (int k = 1; k < P2; ++k) {
                if (pins[k].absX < minX) minX = pins[k].absX;
                if (pins[k].absX > maxX) maxX = pins[k].absX;
                if (pins[k].absY < minY) minY = pins[k].absY;
                if (pins[k].absY > maxY) maxY = pins[k].absY;
            }
            const double epsBound = 1e-3; // 稍微宽一点，避免浮点误差
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

            const double denomBase = static_cast<double>(P2 - 1);
            if (denomBase <= 0.0) continue;

            // 2.1.3 枚举该 net 中的所有 pin 对 (p,q)，构造 Bound2Bound 二次项
            for (int a = 0; a < P2; ++a) {
                const PinInfo& pa = pins[a];
                for (int b = a + 1; b < P2; ++b) {
                    const PinInfo& pb = pins[b];

                    double w_x = 0.0;
                    {
                        double distX = std::fabs(pa.absX - pb.absX);
                        if (distX < distMin) distX = distMin;
                        if (pa.boundX || pb.boundX) {
                            w_x = 1.0 / (denomBase * distX);
                        }
                    }

                    double w_y = 0.0;
                    {
                        double distY = std::fabs(pa.absY - pb.absY);
                        if (distY < distMin) distY = distMin;
                        if (pa.boundY || pb.boundY) {
                            w_y = 1.0 / (denomBase * distY);
                        }
                    }

                    const bool aMov = pa.movable;
                    const bool bMov = pb.movable;
                    const int  ia   = pa.varIdx;
                    const int  ib   = pb.varIdx;

                    // --------- 三种情形分别填充 A 和 rhs（x / y 两个方向独立） ---------

                    // ① p、q 均为可移动模块
                    if (aMov && bMov) {
                        if (w_x > 0.0) {
                            tripX.emplace_back(ia, ia,  w_x);
                            tripX.emplace_back(ib, ib,  w_x);
                            tripX.emplace_back(ia, ib, -w_x);
                            tripX.emplace_back(ib, ia, -w_x);
                            double ddx = pa.offX - pb.offX;
                            rhs_x(ia) -= w_x * ddx;
                            rhs_x(ib) += w_x * ddx;
                        }
                        if (w_y > 0.0) {
                            tripY.emplace_back(ia, ia,  w_y);
                            tripY.emplace_back(ib, ib,  w_y);
                            tripY.emplace_back(ia, ib, -w_y);
                            tripY.emplace_back(ib, ia, -w_y);
                            double ddy = pa.offY - pb.offY;
                            rhs_y(ia) -= w_y * ddy;
                            rhs_y(ib) += w_y * ddy;
                        }
                    }
                    // ② p 可移动，q 为 terminal（固定）
                    else if (aMov && !bMov) {
                        if (w_x > 0.0) {
                            tripX.emplace_back(ia, ia, w_x);
                            double c = pb.absX; // terminal pin 绝对 x
                            rhs_x(ia) += w_x * (c - pa.offX);
                        }
                        if (w_y > 0.0) {
                            tripY.emplace_back(ia, ia, w_y);
                            double c = pb.absY; // terminal pin 绝对 y
                            rhs_y(ia) += w_y * (c - pa.offY);
                        }
                    }
                    // ③ p 为 terminal，q 可移动
                    else if (!aMov && bMov) {
                        if (w_x > 0.0) {
                            tripX.emplace_back(ib, ib, w_x);
                            double c = pa.absX;
                            rhs_x(ib) += w_x * (c - pb.offX);
                        }
                        if (w_y > 0.0) {
                            tripY.emplace_back(ib, ib, w_y);
                            double c = pa.absY;
                            rhs_y(ib) += w_y * (c - pb.offY);
                        }
                    }
                    // 两端都是 terminal：忽略

                    ++totalPairs;
                }
            }
        } // end for each net

        std::cout << "[Init] Iteration " << (iter + 1)
                  << " assembled " << totalPairs << " pin pairs.\n";

        // 2.2 构建稀疏矩阵 A_x, A_y，并加微小对角项
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
        // ---- 解 X ----
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

        // ---- 解 Y ----
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

        // 2.4 回写坐标前，先检查越界节点（记录原始解）
        std::cout << "[Init] Out-of-core nodes BEFORE clipping (iter "
                  << (iter + 1) << "):\n";
        for (int i = 0; i < N; ++i) {
            Module* m  = movableMods[i];
            double  w  = m->width;
            double  h  = m->height;
            double  x0 = sol_x[i];
            double  y0 = sol_y[i];

            bool out = false;
            if (x0 - 0.5 * w < core_llx || x0 + 0.5 * w > core_urx ||
                y0 - 0.5 * h < core_lly || y0 + 0.5 * h > core_ury) {
                out = true;
            }

            if (out) {
                std::cout << "  idx=" << i
                          << " rawCenter=(" << x0 << "," << y0 << ")\n";
            }
        }

        // 2.5 把解回写到模块中心，并按 CoreRegion 边界裁剪
        for (int i = 0; i < N; ++i) {
            Module* m = movableMods[i];
            double  w = m->width;
            double  h = m->height;
            double  x = sol_x[i];
            double  y = sol_y[i];

            if (x - 0.5 * w < core_llx) {
                x = core_llx + 0.5 * w + EPS;
            }
            if (x + 0.5 * w > core_urx) {
                x = core_urx - 0.5 * w - EPS;
            }
            if (y - 0.5 * h < core_lly) {
                y = core_lly + 0.5 * h + EPS;
            }
            if (y + 0.5 * h > core_ury) {
                y = core_ury - 0.5 * h - EPS;
            }

            m->center.x = static_cast<float>(x);
            m->center.y = static_cast<float>(y);
        }

        // 2.6 计算当前 HPWL，打印
        double curHPWL = computeHPWL(db);
        std::cout << "[Init] Iter " << (iter + 1)
                  << " HPWL = " << curHPWL << "\n";

        // 2.7 保存当前布局图片（这里本身就很耗时，可视情况减少次数或关闭）
        {
            std::string imgName = "iteration/iter_" + std::to_string(iter + 1);
            PLOTTING::plotPlacement(imgName, db);
        }

        // 2.8 计算最大位移，用于收敛判据
        double maxDelta = 0.0;
        for (int i = 0; i < N; ++i) {
            double nx = movableMods[i]->center.x;
            double ny = movableMods[i]->center.y;
            double dx = nx - prevX[i];
            double dy = ny - prevY[i];
            double d  = std::sqrt(dx * dx + dy * dy);
            if (d > maxDelta) maxDelta = d;

            prevX[i] = nx;
            prevY[i] = ny;
        }

        double relHPWLChange = (prevHPWL > 0.0)
                             ? std::fabs(curHPWL - prevHPWL) / prevHPWL
                             : 0.0;
        prevHPWL = curHPWL;

        std::cout << "[Init] Iter " << (iter + 1)
                  << " maxDelta = " << maxDelta
                  << ", relHPWLChange = " << relHPWLChange << "\n";

        if (maxDelta < moveTol && relHPWLChange < hpwlRelTol) {
            std::cout << "[Init] Converged at iter " << (iter + 1) << ".\n";
            break;
        }
    }

    lastHPWL = db ? db->calcHPWL() : 0.0;
    std::cout << "[Init] initialPlacement finished.\n";
}

void MyPlacer::Init()
{
    FillerInit();
    BinInit();
    gradientVectorInitialization();
    GetTotalGradient();
}

void MyPlacer::penaltyFactorInitilization()
{
    float denominator = 0;
    float numerator = 0;

    int nodeCount = wirelengthGradient.size();
    int nodeAndFillerCount = densityGradient.size();

    for (int i = 0; i < nodeCount; i++)
    {
        numerator += fabs(wirelengthGradient[i].x);
        numerator += fabs(wirelengthGradient[i].y);
        denominator += fabs(densityGradient[i].x);
        denominator += fabs(densityGradient[i].y);
    }
    for (int i = nodeCount; i < nodeAndFillerCount; i++)
    {
        denominator += fabs(densityGradient[i].x);
        denominator += fabs(densityGradient[i].y);
    }

    lambda = numerator/denominator;
}

void MyPlacer::updatePenaltyFactor()
{
    // printf("penalty factor = %.10f\n", lambda);
    double curHPWL = db->calcHPWL();
    float multiplier;
    double deltaHPWL = curHPWL - lastHPWL;
    if ((deltaHPWL) < 0.0) //?? what if (curHPWL - lastHPWL)<0????? never considered before 2024.5.19
    {
        // cout << "multiplier is: " << pow(PENALTY_MULTIPLIER_BASE, (-(deltaHPWL) / DELTA_HPWL_REF + 1.0)) << " when < 0" << endl;
        multiplier = PENALTY_MULTIPLIER_UPPERBOUND;
    }
    else
    {
        multiplier = pow(PENALTY_MULTIPLIER_BASE, (-(deltaHPWL) / DELTA_HPWL_REF + 1.0)); // see ePlace-3D code opt.cpp line 1523
    }

    if (multiplier>PENALTY_MULTIPLIER_UPPERBOUND)
    {
        multiplier = PENALTY_MULTIPLIER_UPPERBOUND;
    }
    if (float_less(multiplier, PENALTY_MULTIPLIER_LOWERBOUND))
    {
        multiplier = PENALTY_MULTIPLIER_LOWERBOUND;
    }
    lambda *= multiplier;

    lastHPWL = curHPWL;
}

std::vector<VECTOR_3D> MyPlacer::getModulePositions(const std::vector<Module *> &modules)
{
    std::vector<VECTOR_3D> res;
    res.reserve(modules.size());

    for (Module *m : modules)
    {
        VECTOR_3D p{};
        if (!m)
        {
            p.x = 0; p.y = 0; p.z = 0;
        }
        else
        {
            p.x = m->center.x;
            p.y = m->center.y;
            p.z = 0.0; 
        }
        res.push_back(p);
    }
    return res;
}

std::vector<VECTOR_3D> MyPlacer::getPosition()
{
    return getModulePositions(NodesAndFillers);
}

void MyPlacer::gradientVectorInitialization()
{
    wirelengthGradient.resize(db->Nodes.size()); 
    densityGradient.resize(NodesAndFillers.size());
    totalGradient.resize(NodesAndFillers.size());

    cGPGradient.resize(CellsAndFillers.size());
    fillerGradient.resize(Fillers.size());
}

void MyPlacer::FillerInit()
{
    //////// calculate whitespace area
    float whitespaceArea = 0;
    float totalOverLapArea = 49164072; // overlap area between placement rows and terminals

    for (Module *curTerminal : db->Terminals)
    {

        CRect terminal;
        terminal.ll = curTerminal->getLL_2D();
        terminal.ur = curTerminal->getUR_2D();

        for (SiteRow curRow : db->SiteRows)
        {
            CRect placementRow;
            placementRow.ll = curRow.getLL_2D();
            placementRow.ur = curRow.getUR_2D();
            totalOverLapArea += getOverlapArea_2D(terminal, placementRow);
        }
    }

    whitespaceArea = db->totalRowArea - totalOverLapArea;

   
    //////// calculate node area
  
    float nodeAreaScaled = 0; // node = std cells + movable macros
    float stdcellArea = 0;
    float macroArea = 0;

    for (Module *curNode : db->Nodes)
    {
        assert(curNode->getArea() > 0);
        if (curNode->isMacro)
        {
            macroArea += curNode->getArea();
        }
        else
        {
            stdcellArea += curNode->getArea();
        }
    }

    StdCellArea = stdcellArea;
    stdcellArea = 37286292.0;
    MacroArea = macroArea;
    
    nodeAreaScaled = stdcellArea + macroArea * targetDensity; 

    //''' calculate filler area


    float totalFillerArea = 0;                                         
    totalFillerArea = whitespaceArea * targetDensity - nodeAreaScaled; 

    int nodeCount = db->Nodes.size();

    vector<float> nodeArea; 
    nodeArea.resize(nodeCount);

    for (int i = 0; i < nodeCount; i++)
    {
        nodeArea[i] = db->Nodes[i]->getArea();

    }

    sort(nodeArea.begin(), nodeArea.end()); 

    float avg80TotalArea = 0;
    float avg80NodeArea = 0;
    int minIdx = (int)(0.05 * (float)nodeCount); 
    int maxIdx = (int)(0.95 * (float)nodeCount);

    for (int i = minIdx; i < maxIdx; i++)
    {
        avg80TotalArea += nodeArea[i];
    }

    avg80NodeArea = avg80TotalArea / ((float)(maxIdx - minIdx));

    float fillerArea = avg80NodeArea; 
    float fillerHeight = db->commonRowHeight;
    float fillerWidth = fillerArea/fillerHeight;

   
    ////// add fillers and set filler locations randomly

    int fillerCount = (int)(totalFillerArea / fillerArea + 0.5); 
    fillerCount = 122545;
    Fillers.resize(fillerCount);

    float leftMost;
    float rightMost;

    for (int i = 0; i < fillerCount; i++)
    {
        string name = "f" + std::to_string(i);
        Module *curFiller = new Module(i + nodeCount, name, fillerWidth, fillerHeight, false, false);
        curFiller->isFiller = true; //!
        Fillers[i] = curFiller;

        db->setModuleLocation_2D_random(curFiller);
    }

    NodesAndFillers = db->Nodes;
    NodesAndFillers.insert(NodesAndFillers.end(), Fillers.begin(), Fillers.end()); 

    for (Module *curCellOrFiller : NodesAndFillers)
    {
        if (curCellOrFiller->isFiller)
        {
            CellsAndFillers.push_back(curCellOrFiller);
        }
        else if (!curCellOrFiller->isMacro)
        {
            CellsAndFillers.push_back(curCellOrFiller);
        }
    }
}

void MyPlacer::BinInit()
{
 
    //////// calculate bin dimension and size
    int nodeCount = db->Nodes.size();
    float nodeArea = StdCellArea + MacroArea;

    float coreRegionWidth = db->coreRegion.getWidth();
    float coreRegionHeight = db->coreRegion.getHeight();
    float coreRegionArea = coreRegionWidth*coreRegionHeight;

    float averageNodeArea = 1.0 * nodeArea / nodeCount;
    float idealBinArea = averageNodeArea / targetDensity;

    int idealBinCount = INT_CONVERT(coreRegionArea / idealBinArea);

    bool isUpdate = false;

    for (int i = 1; i < 10; i++)
    { //! 4*4,8*8,16*16,32*32..., 1024*1024
        if ((2 << i) * (2 << i) <= idealBinCount &&
            (2 << (i + 1)) * (2 << (i + 1)) > idealBinCount)
        {
            binDimension.x = binDimension.y = 2 << i;
            isUpdate = true;
            break;
        }
    }
    if (!isUpdate)
    {
        binDimension.x = binDimension.y = 1024; 
    }

    binStep.x = coreRegionWidth/binDimension.x;     // binStep.x表示单个网格宽度
    binStep.y = coreRegionHeight/binDimension.y;      // binStep.y表示单个网格高度


    bins.resize(binDimension.x);

    double addBinTime;
    double terminalDensityTime;
    double baseDensityTime;

    for (int i = 0; i < binDimension.x; i++)
    {
        bins[i].resize(binDimension.y);
        for (int j = 0; j < binDimension.y; j++)
        {
            bins[i][j] = new Bin_2D();

            bins[i][j]->ll.x = i * binStep.x + db->coreRegion.ll.x;
            bins[i][j]->ll.y = j * binStep.y + db->coreRegion.ll.y;

            bins[i][j]->width = binStep.x;
            bins[i][j]->height = binStep.y;

            bins[i][j]->ur.x = bins[i][j]->ll.x + bins[i][j]->width;
            bins[i][j]->ur.y = bins[i][j]->ll.y + bins[i][j]->height;

            bins[i][j]->area = binStep.x * binStep.y;

            bins[i][j]->center.x = bins[i][j]->ll.x + (float)0.5 * bins[i][j]->width;
            bins[i][j]->center.y = bins[i][j]->ll.y + (float)0.5 * bins[i][j]->height;
        }
    }



    //////// terminal density calculation
    VECTOR_2D_INT binStartIdx;
    VECTOR_2D_INT binEndIdx;


    for (Module *curTerminal : db->Terminals)
    {
        if (curTerminal->getLL_2D().x < db->coreRegion.ll.x || curTerminal->getLL_2D().x + curTerminal->getWidth() > db->coreRegion.ur.x)
        {
            continue;
        }
        if (curTerminal->getLL_2D().y < db->coreRegion.ll.y || curTerminal->getLL_2D().y + curTerminal->getHeight() > db->coreRegion.ur.y)
        {
            continue;
        }

        binStartIdx.x = INT_DOWN((curTerminal->getLL_2D().x - db->coreRegion.ll.x) / binStep.x);
        binEndIdx.x = INT_DOWN((curTerminal->getUR_2D().x - db->coreRegion.ll.x) / binStep.x);

        binStartIdx.y = INT_DOWN((curTerminal->getLL_2D().y - db->coreRegion.ll.y) / binStep.y);
        binEndIdx.y = INT_DOWN((curTerminal->getUR_2D().y - db->coreRegion.ll.y) / binStep.y);

        if (binEndIdx.y >= binDimension.y)
        {
            binEndIdx.y = binDimension.y - 1;
        }

        if (binEndIdx.x >= binDimension.x)
        {
            binEndIdx.x = binDimension.x - 1;
        }

        for (int i = binStartIdx.x; i <= binEndIdx.x; i++)
        {
            for (int j = binStartIdx.y; j <= binEndIdx.y; j++)
            {
                //! beware: density scaling!    // 计算终端单元密度
                bins[i][j]->terminalDensity += targetDensity * getOverlapArea_2D(bins[i][j]->ll, bins[i][j]->ur, curTerminal->getLL_2D(), curTerminal->getUR_2D());
            }
        }
    }

    ////// Dark density calculation
    for (int i = 0; i < binDimension.x; i++)
    {
        for (int j = 0; j < binDimension.y; j++)
        {
            float curBinAvailableArea = 0; // overlap area between current bin and placement rows
            for (SiteRow curRow : db->SiteRows)
            {
                curBinAvailableArea += getOverlapArea_2D(bins[i][j]->ll, bins[i][j]->ur, curRow.getLL_2D(), curRow.getUR_2D());
            }

            if (float_equal(bins[i][j]->area, curBinAvailableArea))
            {
                bins[i][j]->darkDensity = 0;
            }
            else
            {
                bins[i][j]->darkDensity = targetDensity * (bins[i][j]->area - curBinAvailableArea); 
            }
        }
    }
}

void MyPlacer::GetWirelengthGradient()
{
    // wirelengthGradient 只对 db->Nodes（真实可放置单元，不含 filler）开
    // 你在 gradientVectorInitialization() 里也是 wirelengthGradient.resize(db->Nodes.size())
    const int numNodes = static_cast<int>(db->Nodes.size());

    // 1) 清零
    for (int i = 0; i < numNodes; ++i)
        wirelengthGradient[i].SetZero();

    // 2) 平滑参数 gamma（可按 pdf 的 (6) 用 overflow 调，先给一个稳健版本）
    // pdf: gamma = 8.0 * wb * 10^(20/9) * (tau - 0.1)^(-1)
    // 这里 wb = binStep.x（你 BinInit 已经算了），tau 用 globalDensityOverflow（若有）
    double wb  = (binStep.x > 1e-9f) ? static_cast<double>(binStep.x) : 1.0;
    double tau = 0.2; // fallback

    // 如果你有 globalDensityOverflow（NesterovOpt 里在打印 placer->globalDensityOverflow）
    // 这里假设 MyPlacer 成员里有 globalDensityOverflow
    // 没有的话就保持 tau=0.2 也能跑
#ifdef HAS_GLOBAL_OVERFLOW
    tau = std::max(0.100001, static_cast<double>(globalDensityOverflow));
#endif

    // 按 pdf 的形式给一个 gamma（并做 clamp，防止 tau 接近 0.1 发散）
    double denom = std::max(1e-6, (tau - 0.1));
    double gamma = 8.0 * wb * std::pow(10.0, 20.0 / 9.0) / denom;

    // 过大/过小都会数值不稳，做一下裁剪
    if (gamma < 1e-3) gamma = 1e-3;
    if (gamma > 1e6)  gamma = 1e6;

    const double invGamma = 1.0 / gamma;
    //const double EPS      = 1e-30;

    // 3) 遍历每条 net，计算平滑 HPWL 梯度并累加到 cell
    for (Net* net : db->Nets)
    {
        if (!net) continue;
        const auto& pins = net->netPins;
        const int P = static_cast<int>(pins.size());
        if (P < 2) continue;

        // --- 先收集 pin 的绝对坐标，顺便找 max/min 做数值稳定 ---
        // 这里用局部数组，避免频繁分配可用 static thread_local 或成员复用优化
        std::vector<double> px(P), py(P);
        std::vector<Module*> mods(P, nullptr);

        double maxX = -1e100, minX = 1e100;
        double maxY = -1e100, minY = 1e100;

        int validP = 0;
        for (int k = 0; k < P; ++k)
        {
            Pin* p = pins[k];
            if (!p || !p->module) { px[k] = py[k] = 0.0; mods[k] = nullptr; continue; }

            Module* m = p->module;
            mods[k] = m;

            double x = static_cast<double>(m->center.x) + static_cast<double>(p->offset.x);
            double y = static_cast<double>(m->center.y) + static_cast<double>(p->offset.y);

            px[k] = x; py[k] = y;

            if (x > maxX) maxX = x;
            if (x < minX) minX = x;
            if (y > maxY) maxY = y;
            if (y < minY) minY = y;

            ++validP;
        }
        if (validP < 2) continue;

        // --- 计算 sum(exp(+x/g)), sum(exp(-x/g))，用 shift 防溢出 ---
        // 令 a_i = exp((x_i - maxX)/g), b_i = exp((minX - x_i)/g) 等价于 exp(-x/g) 的稳定形式
        double sumPosX = 0.0, sumNegX = 0.0;
        double sumPosY = 0.0, sumNegY = 0.0;

        // x 方向：正项 shift 用 maxX；负项 shift 用 minX（等价稳定）
        for (int k = 0; k < P; ++k)
        {
            if (!mods[k]) continue;

            sumPosX += std::exp((px[k] - maxX) * invGamma);
            sumNegX += std::exp((minX - px[k]) * invGamma);

            sumPosY += std::exp((py[k] - maxY) * invGamma);
            sumNegY += std::exp((minY - py[k]) * invGamma);
        }

        sumPosX = std::max(sumPosX, EPS);
        sumNegX = std::max(sumNegX, EPS);
        sumPosY = std::max(sumPosY, EPS);
        sumNegY = std::max(sumNegY, EPS);

        // --- 对每个 pin 求梯度：dWL/dx_pin = p_i^+ - p_i^- ---
        for (int k = 0; k < P; ++k)
        {
            Module* m = mods[k];
            if (!m) continue;
            if (m->isFixed) continue;         // 固定模块不累加梯度
            if (m->idx < 0 || m->idx >= numNodes) continue;

            // p^+ = exp((x-maxX)/g)/sumPos
            double pposX = std::exp((px[k] - maxX) * invGamma) / sumPosX;
            // p^- = exp((minX-x)/g)/sumNeg  (等价于 exp(-x/g) 的归一化)
            double pnegX = std::exp((minX - px[k]) * invGamma) / sumNegX;

            double pposY = std::exp((py[k] - maxY) * invGamma) / sumPosY;
            double pnegY = std::exp((minY - py[k]) * invGamma) / sumNegY;

            // 注意：这是 ∂WL/∂x（“上升方向”）
            // 你在 GetTotalGradient() 里用了 (lambda*density - wirelength)，相当于后面走下降
            wirelengthGradient[m->idx].x += static_cast<float>(pposX - pnegX);
            wirelengthGradient[m->idx].y += static_cast<float>(pposY - pnegY);
        }
    }
}

void MyPlacer::GetBinDensity()
{
    // 防御：若 bin 未初始化，直接返回避免除零/越界
    if (binDimension.x <= 0 || binDimension.y <= 0) {
        std::cerr << "[GetBinDensity] binDimension is zero. Call BinInit() first.\n";
        globalDensityOverflow = 0.0f;
        return;
    }

    // 1. 清零每个 bin 的 node/filler 密度
    for (int i = 0; i < binDimension.x; i++)
    {
        for (int j = 0; j < binDimension.y; j++)
        {
            bins[i][j]->nodeDensity   = 0;
            bins[i][j]->fillerDensity = 0;
        }
    }

    // 2. 遍历所有 node + filler
    for (Module *curNode : NodesAndFillers)
    {
        VECTOR_2D localSmoothLengthScale;
        localSmoothLengthScale.x = 1.0f;
        localSmoothLengthScale.y = 1.0f;

        CRect rectForCurNode;
        rectForCurNode.ll = curNode->getLL_2D();
        rectForCurNode.ur = curNode->getUR_2D();

        POS_3D cellCenter = curNode->getCenter();

        // 小单元扩到一个 bin 大小（Kraftwerk2 的 smoothing）
        if (float_less(curNode->getWidth(), binStep.x))
        {
            localSmoothLengthScale.x = curNode->getWidth() / binStep.x;
            rectForCurNode.ll.x = cellCenter.x - 0.5f * binStep.x;
            rectForCurNode.ur.x = cellCenter.x + 0.5f * binStep.x;
        }
        if (float_less(curNode->getHeight(), binStep.y))
        {
            localSmoothLengthScale.y = curNode->getHeight() / binStep.y;
            rectForCurNode.ll.y = cellCenter.y - 0.5f * binStep.y;
            rectForCurNode.ur.y = cellCenter.y + 0.5f * binStep.y;
        }

        // 如果整个 rect 完全在 core 外面，直接跳过
        if (rectForCurNode.ur.x <= db->coreRegion.ll.x ||
            rectForCurNode.ll.x >= db->coreRegion.ur.x ||
            rectForCurNode.ur.y <= db->coreRegion.ll.y ||
            rectForCurNode.ll.y >= db->coreRegion.ur.y)
        {
            continue;
        }

        VECTOR_2D_INT binStartIdx;
        VECTOR_2D_INT binEndIdx;

        binStartIdx.x = INT_DOWN((rectForCurNode.ll.x - db->coreRegion.ll.x) / binStep.x);
        binEndIdx.x   = INT_DOWN((rectForCurNode.ur.x - db->coreRegion.ll.x) / binStep.x);

        binStartIdx.y = INT_DOWN((rectForCurNode.ll.y - db->coreRegion.ll.y) / binStep.y);
        binEndIdx.y   = INT_DOWN((rectForCurNode.ur.y - db->coreRegion.ll.y) / binStep.y);

        // ---- 关键：对索引做边界裁剪 ----
        // 如果结束索引都在 0 左边，或者起始索引都在最大右边，说明完全不在 core 内
        if (binEndIdx.x < 0 || binEndIdx.y < 0) continue;
        if (binStartIdx.x >= binDimension.x || binStartIdx.y >= binDimension.y) continue;

        if (binStartIdx.x < 0)             binStartIdx.x = 0;
        if (binStartIdx.y < 0)             binStartIdx.y = 0;
        if (binEndIdx.x   >= binDimension.x) binEndIdx.x = binDimension.x - 1;
        if (binEndIdx.y   >= binDimension.y) binEndIdx.y = binDimension.y - 1;

        // 防御：如果裁剪完后 start > end，说明确实没交集
        if (binStartIdx.x > binEndIdx.x || binStartIdx.y > binEndIdx.y)
            continue;

        // 3. 累加密度
        for (int i = binStartIdx.x; i <= binEndIdx.x; i++)
        {
            for (int j = binStartIdx.y; j <= binEndIdx.y; j++)
            {
                float overlapArea = getOverlapArea_2D(
                    bins[i][j]->ll, bins[i][j]->ur,
                    rectForCurNode.ll, rectForCurNode.ur);

                if (overlapArea <= 0.0f)
                    continue;

                float coeff = localSmoothLengthScale.x * localSmoothLengthScale.y;

                if (curNode->isMacro)   // 宏单元：系上 targetDensity
                {
                    bins[i][j]->nodeDensity += coeff * targetDensity * overlapArea;
                }
                else
                {
                    if (curNode->isFiller)  // 填充单元
                    {
                        bins[i][j]->fillerDensity += coeff * overlapArea;
                    }
                    else                    // 标准单元
                    {
                        bins[i][j]->nodeDensity += coeff * overlapArea;
                    }
                }
            }
        }
    }

    // 4. 计算全局 density overflow（>0 表示超出 targetDensity 的比例）
    globalDensityOverflow = 0.0f;
    for (int i = 0; i < binDimension.x; ++i)
    {
        for (int j = 0; j < binDimension.y; ++j)
        {
            Bin_2D* b = bins[i][j];
            if (!b) continue;

            float totalArea = b->nodeDensity + b->fillerDensity + b->terminalDensity + b->darkDensity;
            float targetArea = b->area * targetDensity;
            if (targetArea <= 0.0f) continue;

            float overflow = totalArea / targetArea - 1.0f;
            if (overflow < 0.0f) overflow = 0.0f;
            if (overflow > globalDensityOverflow) globalDensityOverflow = overflow;
        }
    }
}


void MyPlacer::GetDensityGradient()
{
    if (binDimension.x <= 0 || binDimension.y <= 0 || binStep.x <= 0.0f || binStep.y <= 0.0f) {
        std::cerr << "[GetDensityGradient] bin/grid not initialized. Call BinInit() first.\n";
        return;
    }

    myplace::FFT_2D fft(binDimension.x, binDimension.y, binStep.x, binStep.y);
    float invertedBinArea = 1.0 / (binStep.x * binStep.y);  // invertedBinArea表示单个网格面积的倒数
    for (int i = 0; i < binDimension.x; i++)
    {
        for (int j = 0; j < binDimension.y; j++)
        {   
            float eDensity = bins[i][j]->nodeDensity + bins[i][j]->darkDensity + bins[i][j]->fillerDensity + bins[i][j]->terminalDensity; // consider filler area(density) here
            eDensity *= invertedBinArea;    // eDensity表示单个网格内单元所占面积比例，即网格密度
            fft.updateDensity(i, j, eDensity);  // 更新密度分布
        }
    }
    fft.doFFT();    // FFT计算电势和电势梯度
    for (int i = 0; i < binDimension.x; i++)
    {
        for (int j = 0; j < binDimension.y; j++)
        {
            auto eForcePair = fft.getElectroForce(i, j);    
            bins[i][j]->E.x = eForcePair.first;     // E表示电场力
            bins[i][j]->E.y = eForcePair.second;
            
            float electroPhi = fft.getElectroPhi(i, j);
            bins[i][j]->phi = electroPhi;       // Phi表示电势,虽然计算了但是之后没有用到
        }
        
    }

   
    /////////////////////calculate density(potential) gradient
    int nodeCount = db->Nodes.size();
    int index = 0;
    for (Module *curNode : NodesAndFillers)
    {
        assert(index == curNode->idx);
        //! clear before updating
        densityGradient[index].SetZero();

        VECTOR_2D localSmoothLengthScale; 
        localSmoothLengthScale.x = 1.0;
        localSmoothLengthScale.y = 1.0;

        CRect rectForCurNode;
        rectForCurNode.ll = curNode->getLL_2D();
        rectForCurNode.ur = curNode->getUR_2D();

             POS_3D cellCenter = curNode->getCenter();


        if (float_less(curNode->getWidth(), binStep.x))
        {
            localSmoothLengthScale.x = curNode->getWidth() / binStep.x;
            rectForCurNode.ll.x = cellCenter.x - 0.5 * binStep.x;
            rectForCurNode.ur.x = cellCenter.x + 0.5 * binStep.x;
        }
        if (float_less(curNode->getHeight(), binStep.y))
        {
            localSmoothLengthScale.y = curNode->getHeight() / binStep.y;
            rectForCurNode.ll.y = cellCenter.y - 0.5 * binStep.y;
            rectForCurNode.ur.y = cellCenter.y + 0.5 * binStep.y;
        }
        // }

        VECTOR_2D_INT binStartIdx; 
        VECTOR_2D_INT binEndIdx;
        binStartIdx.x = INT_DOWN((rectForCurNode.ll.x - db->coreRegion.ll.x) / binStep.x);
        binEndIdx.x = INT_DOWN((rectForCurNode.ur.x - db->coreRegion.ll.x) / binStep.x);

        binStartIdx.y = INT_DOWN((rectForCurNode.ll.y - db->coreRegion.ll.y) / binStep.y);
        binEndIdx.y = INT_DOWN((rectForCurNode.ur.y - db->coreRegion.ll.y) / binStep.y);

        // bin 索引裁剪到合法范围，避免越界
        if (binEndIdx.x < 0 || binEndIdx.y < 0) { index++; continue; }
        if (binStartIdx.x >= binDimension.x || binStartIdx.y >= binDimension.y) { index++; continue; }

        if (binStartIdx.x < 0) binStartIdx.x = 0;
        if (binStartIdx.y < 0) binStartIdx.y = 0;

        if (binEndIdx.y >= binDimension.y)
        {
            binEndIdx.y = binDimension.y - 1;
        }

        if (binEndIdx.x >= binDimension.x)
        {
            binEndIdx.x = binDimension.x - 1;
        }

        if (binStartIdx.x > binEndIdx.x || binStartIdx.y > binEndIdx.y) { index++; continue; }


        for (int i = binStartIdx.x; i <= binEndIdx.x; i++)
        {
            for (int j = binStartIdx.y; j <= binEndIdx.y; j++)
            {
                // overlapArea表示网格中单元所占面积（即公式中的qi）
                float overlapArea = localSmoothLengthScale.x * localSmoothLengthScale.y * getOverlapArea_2D(bins[i][j]->ll, bins[i][j]->ur, rectForCurNode.ll, rectForCurNode.ur);
                densityGradient[index].x += overlapArea * bins[i][j]->E.x;
                densityGradient[index].y += overlapArea * bins[i][j]->E.y;
                // 密度梯度 = qi * 电势梯度（求的是单元的密度梯度？）
            }
        }

        index++;
    }
}


void MyPlacer::GetTotalGradient()
{
    GetBinDensity();
    GetDensityGradient();
    GetWirelengthGradient();

    // ✅ totalGradient 必须是“紧凑的”，长度等于 NodesAndFillers.size()
    totalGradient.resize(NodesAndFillers.size());

    int cGPindex = 0;
    int fillerOnlyIndex = 0;

    for (size_t local = 0; local < NodesAndFillers.size(); ++local)
    {
        Module* m = NodesAndFillers[local];
        if (!m) continue;

        totalGradient[local].SetZero();

        // ✅ 用全局 idx 去索引 density/wl
        const int gid = m->idx;

        float connectedNetNum = (float)m->modulePins.size();
        float charge = m->getArea();
        // |E_i| 用 densityGradient 的模长来近似
        float e_mag = std::sqrt(densityGradient[gid].x * densityGradient[gid].x +
                                densityGradient[gid].y * densityGradient[gid].y);
        float denom = std::max(1e-3f, std::fabs(e_mag) + lambda * charge);
        float preconditioner = 1.0f / denom;

        if (m->isFiller)
        {
            totalGradient[local].x = preconditioner * lambda * densityGradient[gid].x;
            totalGradient[local].y = preconditioner * lambda * densityGradient[gid].y;

            fillerGradient[fillerOnlyIndex++] = totalGradient[local];
            cGPGradient[cGPindex++] = totalGradient[local];
        }
        else
        {
            totalGradient[local].x = preconditioner * (lambda * densityGradient[gid].x - wirelengthGradient[gid].x);
            totalGradient[local].y = preconditioner * (lambda * densityGradient[gid].y - wirelengthGradient[gid].y);

            if (!m->isMacro)
            {
                cGPGradient[cGPindex++] = totalGradient[local];
            }
        }
    }
}



void MyPlacer::setTargetDensity(float target)
{
    targetDensity = target;
}


void MyPlacer::setPosition(const std::vector<VECTOR_3D>& modulePositions)
{
    const size_t n = NodesAndFillers.size();
    if (modulePositions.size() != n)
    {
        printf("[ERR] setPosition size mismatch: pos=%zu nodes=%zu\n",
               modulePositions.size(), n);
        return;
    }

    for (size_t i = 0; i < n; ++i)
    {
        Module* m = NodesAndFillers[i];
        if (!m) continue;
        // 固定/终端模块保持原位，不写回
        if (m->isFixed || m->isTerminal) continue;
        db->setModuleCenter_2D(m, modulePositions[i]);
    }
}















