#include "myplace.h"

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
        if (m->isFixed) continue;  // 终端/固定模块不做变量

        int idx = static_cast<int>(movableMods.size());
        movableMods.push_back(m);
        modToIndex[m] = idx;
    }

    const int N = static_cast<int>(movableMods.size());
    if (N == 0) {
        std::cout << "[Init] No movable modules. Skip.\n";
        return;
    }

    // -------------------- CoreRegion 信息 --------------------
    const double core_llx = db->chipRegion.ll.x;
    const double core_lly = db->chipRegion.ll.y;
    const double core_urx = db->chipRegion.ur.x;
    const double core_ury = db->chipRegion.ur.y;
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

    // -------------------- 2. 迭代求解：w 中包含 x，需要多次迭代 --------------------
    const int    maxOuterIters = 30;      // 外层最多迭代次数（逻辑不变，只给足一点余量）
    const double distMin       = 25.0;    // Bound2Bound 距离下界
    const double lambdaDiag    = 1e-8;    // 小对角正则
    const double cgTol         = 1e-8;    // BiCGSTAB 收敛阈值
    const double moveTol       = 1e-2;    // 最大位移收敛阈值
    const double hpwlRelTol    = 1e-3;    // HPWL 相对变化收敛阈值

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

    // 用于收敛判据
    std::vector<double> prevX(N), prevY(N);
    for (int i = 0; i < N; ++i) {
        prevX[i] = movableMods[i]->center.x;
        prevY[i] = movableMods[i]->center.y;
    }
    double prevHPWL = computeHPWL(db);

    for (int iter = 0; iter < maxOuterIters; ++iter) {
        std::cout << "------------------------ InitPlacement Iteration "
                  << (iter + 1) << " ------------------------\n";

        // 2.1 为当前迭代构造 A_x, A_y 以及 RHS b_x, b_y
        std::vector<Triplet> tripX, tripY;
        tripX.reserve(N * 16);
        tripY.reserve(N * 16);

        Vec rhs_x = Vec::Zero(N);
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

                // pin 绝对坐标 = 模块中心 + offset
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
                        } else {
                            w_x = 0.0;
                        }
                    }

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

                    // --------- 三种情形分别填充 A 和 rhs（x / y 两个方向独立） ---------

                    // ① p、q 均为可移动模块
                    if (aMov && bMov) {
                        if (w_x > 0.0) {
                            // A_x
                            tripX.emplace_back(ia, ia,  w_x);
                            tripX.emplace_back(ib, ib,  w_x);
                            tripX.emplace_back(ia, ib, -w_x);
                            tripX.emplace_back(ib, ia, -w_x);
                            // rhs_x（考虑 offset）
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
        solver.setMaxIterations(5000);

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

        // 2.4 回写坐标前，先检查越界节点（记录原始解）
        //const double EPS = 1e-3;
        std::cout << "[Init] Out-of-core nodes BEFORE clipping (iter "
                  << (iter + 1) << "):\n";
        for (int i = 0; i < N; ++i) {
            Module* m = movableMods[i];
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
            double  w  = m->width;
            double  h  = m->height;
            double  x  = sol_x[i];
            double  y  = sol_y[i];

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

        // 2.7 保存当前布局图片
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

        // 简单收敛判据：位移足够小 或 HPWL 变化足够小
        if (maxDelta < moveTol && relHPWLChange < hpwlRelTol) {
            std::cout << "[Init] Converged at iter " << (iter + 1) << ".\n";
            break;
        }
    }

    std::cout << "[Init] initialPlacement finished.\n";
}

void MyPlacer::Init()
{
    FillerInit();
    BinInit();
    gradientVectorInitialization();
    GetTotalGradient();
}

vector<VECTOR_3D> MyPlacer::getModulePositions(vector<Module *> modules)
{
    int moduleCount = modules.size();
    vector<VECTOR_3D> res;
    res.resize(moduleCount);

    for (int i = 0; i < modules.size(); i++)
    {
        res[i] = modules[i]->getCenter();
    }
    return res;
}

vector<VECTOR_3D> MyPlacer::getPosition()
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
        string name = "f" + to_string(i);
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
    
}
void MyPlacer::GetBinDensity()     // 计算每个网格的密度
{                                          
    for (int i = 0; i < binDimension.x; i++)
    {
        for (int j = 0; j < binDimension.y; j++)
        {
            bins[i][j]->nodeDensity = 0;
            bins[i][j]->fillerDensity = 0;
        }
    }

    for (Module *curNode : NodesAndFillers) // （node(标准单元、宏单元)+填充单元）
    {        
       
        VECTOR_2D localSmoothLengthScale; 
        localSmoothLengthScale.x = 1;
        localSmoothLengthScale.y = 1;

        CRect rectForCurNode;
        rectForCurNode.ll = curNode->getLL_2D();
        rectForCurNode.ur = curNode->getUR_2D();

        POS_3D cellCenter = curNode->getCenter();

        if (float_less(curNode->getWidth(), binStep.x))     // 如果当前单元的宽度小于单个网格宽度
        {
            localSmoothLengthScale.x = curNode->getWidth() / binStep.x;
            rectForCurNode.ll.x = cellCenter.x - 0.5 * binStep.x;   // 将单元宽度扩充到一个网格宽度
            rectForCurNode.ur.x = cellCenter.x + 0.5 * binStep.x;   // 计算单元密度时需要单元宽度高度大于单个网格宽高度
        }
        if (float_less(curNode->getHeight(), binStep.y))
        {
            localSmoothLengthScale.y = curNode->getHeight() / binStep.y;
            rectForCurNode.ll.y = cellCenter.y - 0.5 * binStep.y;
            rectForCurNode.ur.y = cellCenter.y + 0.5 * binStep.y;
        }

        VECTOR_2D_INT binStartIdx; 
        VECTOR_2D_INT binEndIdx;
        binStartIdx.x = INT_DOWN((rectForCurNode.ll.x - db->coreRegion.ll.x) / binStep.x); // binStartIdx.x表示节点起始位置对应网格index
        binEndIdx.x = INT_DOWN((rectForCurNode.ur.x - db->coreRegion.ll.x) / binStep.x);

        binStartIdx.y = INT_DOWN((rectForCurNode.ll.y - db->coreRegion.ll.y) / binStep.y);
        binEndIdx.y = INT_DOWN((rectForCurNode.ur.y - db->coreRegion.ll.y) / binStep.y);

        if (binEndIdx.y >= binDimension.y)      // 如果节点分布在核心区域外
        {
            binEndIdx.y = binDimension.y - 1;   // 向区域内移动一个网格
        }

        if (binEndIdx.x >= binDimension.x)
        {
            binEndIdx.x = binDimension.x - 1;
        }


        for (int i = binStartIdx.x; i <= binEndIdx.x; i++)
        {
            for (int j = binStartIdx.y; j <= binEndIdx.y; j++)  // 遍历单元包含的所有网格
            {
                // overlapArea为单个网格中的单元所占面积
                float overlapArea = getOverlapArea_2D(bins[i][j]->ll, bins[i][j]->ur, rectForCurNode.ll, rectForCurNode.ur);
                if (curNode->isMacro)   // 若当前节点是宏单元
                {                       // qi=单元面积*targetDensity
                    bins[i][j]->nodeDensity += localSmoothLengthScale.x * localSmoothLengthScale.y * targetDensity * overlapArea;
                }
                else
                {
                    if (curNode->isFiller)  // 若当前节点是填充单元
                    {
                       
                        bins[i][j]->fillerDensity += localSmoothLengthScale.x * localSmoothLengthScale.y * overlapArea;
                    }
                    else        // 若当前节点是标准单元
                    {           // qi = 单元面积
                        bins[i][j]->nodeDensity += localSmoothLengthScale.x * localSmoothLengthScale.y * overlapArea;
                    }
                }
            }
        }
    }
}


void MyPlacer::GetDensityGradient()
{
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

        assert(binStartIdx.x >= 0);
        assert(binEndIdx.x >= 0);
        assert(binStartIdx.y >= 0);
        assert(binEndIdx.y >= 0);

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

    int index = 0;
    int cGPindex = 0;
    int fillerOnlyIndex = 0;
    for (Module *curNodeOrFiller : NodesAndFillers) 
    {
        totalGradient[index].SetZero();
        assert(index == curNodeOrFiller->idx);


        float connectedNetNum = curNodeOrFiller->modulePins.size();

        float charge = curNodeOrFiller->getArea();
        float preconditioner = 1 / max(1.0f, (connectedNetNum + lambda * charge));

        if (curNodeOrFiller->isFiller)  // 若当前单元是填充单元
        {
    
            totalGradient[index].x = preconditioner * lambda * densityGradient[index].x;
            totalGradient[index].y = preconditioner * lambda * densityGradient[index].y;

            fillerGradient[fillerOnlyIndex] = totalGradient[index];
            fillerOnlyIndex++;

            cGPGradient[cGPindex] = totalGradient[index];
            cGPindex++;
        }
        else    // 若当前单元非填充单元
        {   
            // 梯度下降法中要求梯度指向函数增大的方向，后面计算需要对梯度取负号，因此在这一步取负号，梯度下降时为正
            totalGradient[index].x = preconditioner * (lambda * densityGradient[index].x - wirelengthGradient[index].x);
            totalGradient[index].y = preconditioner * (lambda * densityGradient[index].y - wirelengthGradient[index].y);
            if (!curNodeOrFiller->isMacro)
            {
                cGPGradient[cGPindex] = totalGradient[index];
                cGPindex++;
            }
        }

        index++;
    }
}

void MyPlacer::setTargetDensity(float target)
{
    targetDensity = target;
}


void MyPlacer::setPosition(vector<VECTOR_3D> modulePositions)
{
    int moduleCount;
    moduleCount = NodesAndFillers.size();
    for (int i = 0; i < moduleCount; i++)
        {
            db->setModuleCenter_2D(NodesAndFillers[i], modulePositions[i]);
        }
}














