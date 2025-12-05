#include "myplace.h"

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

        // 计算索引
        binStartIdx.x = INT_DOWN((rectForCurNode.ll.x - db->coreRegion.ll.x) / binStep.x);
        binEndIdx.x   = INT_DOWN((rectForCurNode.ur.x - db->coreRegion.ll.x) / binStep.x);

        binStartIdx.y = INT_DOWN((rectForCurNode.ll.y - db->coreRegion.ll.y) / binStep.y);
        binEndIdx.y   = INT_DOWN((rectForCurNode.ur.y - db->coreRegion.ll.y) / binStep.y);

        // 上下界裁剪
        binStartIdx.x = max(0, std::min(binStartIdx.x, binDimension.x - 1));
        binEndIdx.x   = max(0, std::min(binEndIdx.x,   binDimension.x - 1));

        binStartIdx.y = max(0, std::min(binStartIdx.y, binDimension.y - 1));
        binEndIdx.y   = max(0, std::min(binEndIdx.y,   binDimension.y - 1));
        
        // 如果完全在核心区外，直接跳过
        if (binStartIdx.x > binEndIdx.x || binStartIdx.y > binEndIdx.y)
            continue;


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














