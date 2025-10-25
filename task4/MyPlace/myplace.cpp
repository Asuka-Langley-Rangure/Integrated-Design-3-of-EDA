#include "myplace.h"
#include "../eigen3/Eigen/Sparse"
#include <random>
void MyPlacer::initialPlacement()
{
    // 1. 收集所有可移动模块
    std::vector<Module*> movableMods;
    std::map<Module*, int> modToIndex;

    for(Module* mod : db->Nodes){
        if(!mod -> isFixed){
            modToIndex[mod] = movableMods.size();
            movableMods.push_back(mod);
        }
    }
    int N = movableMods.size();
    if ( N == 0) return;

    // 2. 初始化稀疏矩阵 A 和向量b
    Eigen::SparseMatrix<double> A_x(N, N), A_y(N, N);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(N);

    for(Net* net : db->Nets){
        if(net->netPins.size() < 2) continue;

        double weight = 1.0 / (net->netPins.size() - 1);

        for(int i = 0; i < net->netPins.size(); i++) {
            Pin* p = net->netPins[i];
            Module* m = p -> module;
            bool mFixed = m->isFixed;
            double px_fixed = m->center.x; // 若固定，这是已知坐标
            double py_fixed = m->center.y;
            double dx = p->offset.x;
            double dy = p->offset.y;

            for (int j = i + 1; j < net->netPins.size(); j++) {
                Pin* q = net->netPins[j];
                Module* n = q->module;
                bool nFixed = n->isFixed;
                double qx_fixed = n->center.x;
                double qy_fixed = n->center.y;
                double ex = q->offset.x;
                double ey = q->offset.y;

                if (!mFixed && !nFixed) { 
                    // 情况1：两个模块都可移动
                    int im = modToIndex[m];
                    int in = modToIndex[n];
                    double ddx = dx - ex;
                    double ddy = dy - ey;

                    A_x.coeffRef(im, im) += weight;
                    A_x.coeffRef(in, in) += weight;
                    A_x.coeffRef(im, in) -= weight;
                    A_x.coeffRef(in, im) -= weight;
                    b_x[im] += weight * ddx;
                    b_x[in] -= weight * ddx;

                    A_y.coeffRef(im, im) += weight;
                    A_y.coeffRef(in, in) += weight;
                    A_y.coeffRef(im, in) -= weight;
                    A_y.coeffRef(in, im) -= weight;
                    b_y[im] += weight * ddy;
                    b_y[in] -= weight * ddy;
                }
                else if (!mFixed && nFixed) { 
                    // 情况2 ： m可以移动，但n固定
                    int im = modToIndex[m];
                    double target_x = (qx_fixed + ex) - dx;
                    double target_y = (qy_fixed + ey) - dy;

                    A_x.coeffRef(im, im) += weight;
                    b_x[im] += weight * target_x;

                    A_y.coeffRef(im, im) += weight;
                    b_y[im] += weight * target_y;
                }
                else if (mFixed && !nFixed) { 
                    // 情况3 ： m固定，n可以移动
                    int in = modToIndex[n];
                    double target_x = (px_fixed + dx) - ex;
                    double target_y = (py_fixed + dy) - ey;

                    A_x.coeffRef(in, in) += weight;
                    b_x[in] += weight * target_x;

                    A_y.coeffRef(in, in) += weight;
                    b_y[in] += weight * target_y;
                }
            }
        } 
    }
    // Step 4: 求解线性方程组 A_x * X = b_x, A_y * Y = b_y
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver_x, solver_y;
    solver_x.compute(A_x);
    solver_y.compute(A_y);

    Eigen::VectorXd X = solver_x.solve(b_x);
    Eigen::VectorXd Y = solver_y.solve(b_y);

    // Step 5: 更新模块中心位置
    for (int i = 0; i < N; i++) {
        movableMods[i]->center.x = X[i];
        movableMods[i]->center.y = Y[i];
    }

    std::cout << "Initial placement completed." << std::endl;
}

void MyPlacer::createfillerCells(){
    double dieLx = std::numeric_limits<double>::max();
    double dieLy = std::numeric_limits<double>::max();
    double dieUx = std::numeric_limits<double>::lowest();
    double dieUy = std::numeric_limits<double>::lowest();

    
    double rowArea = 0.0;
    for(const auto& row : db->SiteRows){
        dieLx = std::min<double>(dieLx, row.start.x);
        dieLy = std::min<double>(dieLy, row.start.y); // 或 row.bottom
        dieUx = std::max<double>(dieUx, row.end.x);
        dieUy = std::max<double>(dieUy, row.bottom + row.height);

        double width = row.end.x - row.start.x;
        rowArea = rowArea + width * row.height;
    }

    double fixedArea = 0.0;
    std::vector<double> movableAreas;
    double movableTotalArea = 0.0;

    for(Module* mod : db->Nodes){
        if(mod -> isFixed){
            fixedArea += mod->area;
        }
        else if(mod ->isFiller){
            movableAreas.push_back(mod->area);
            movableTotalArea += mod->area;
        }
    }
    


    if (movableAreas.empty()) {
        std::cout << "No movable cells. Skip filler creation." << std::endl;
        return;
    }

    // 排序并取中间 90%
    std::sort(movableAreas.begin(), movableAreas.end());
    int n = movableAreas.size();
    int start = static_cast<int>(n * 0.05);
    int end = static_cast<int>(n * 0.95);
    if (start >= end) {
        start = 0;
        end = n;
    }

    double sum = 0.0;
    for (int i = start; i < end; ++i) {
        sum += movableAreas[i];
    }
    double avgMovableArea = sum / (end - start);

    double targetDensity = 0.8; // 可配置为参数
    double availableArea = rowArea - fixedArea;//空白区域
    double requiredMovableArea = targetDensity * availableArea;
    double x = 0.0;//（标准单元面积 + 宏块面积* targetDensity）
    for(Module* mod : db->Nodes){
        if(!mod -> isFixed){
            if(mod->isMacro) x += mod->area * targetDensity;
            else x += mod->area;
        }
    }
    double fillerArea = requiredMovableArea - x;//总填充面积

    if (fillerArea <= 0) {
        std::cout << "No filler needed (density already satisfied)." << std::endl;
        return;
    }

    double fillerHeight = db->siteHeight; // 从 .scl 读取
    double fillerWidth = avgMovableArea / fillerHeight;
    if (fillerWidth <= 0) {
        fillerWidth = 1.0; // 安全兜底
    }

    int numFillers = static_cast<int>(fillerArea / avgMovableArea);
    if (numFillers <= 0) return;

    // === Step 6: 创建 filler 模块 ===
    std::cout << "Creating " << numFillers << " filler cells..." << std::endl;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(dieLx, dieUx);
    std::uniform_real_distribution<> dis_y(dieLy, dieUy);

    for (int i = 0; i < numFillers; ++i) {
        Module* filler = new Module();
        filler->name = "FILLER_" + std::to_string(i);
        filler->width = static_cast<float>(fillerWidth);
        filler->height = static_cast<float>(fillerHeight);
        filler->area = static_cast<float>(fillerWidth * fillerHeight);
        filler->isFixed = false;
        filler->isFiller = true; // 标记为 filler
        filler->center.x = static_cast<float>(dis_x(gen));
        filler->center.y = static_cast<float>(dis_y(gen));
        // netPins 为空（默认）
        db->Nodes.push_back(filler);
    }

    std::cout << "Added " << numFillers << " filler cells (each "
              << fillerWidth << " x " << fillerHeight << ")." << std::endl;
}

// 返回 SiteRow 区域的 bounding box
std::pair<POS_2D, POS_2D> MyPlacer::getSiteRowBoundingBox() {
    if (db->SiteRows.empty()) {
        return {{0, 0}, {1000, 1000}}; // fallback
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
    // === Step 1: 计算节点总面积和数量 ===
    double totalArea = 0.0;
    int nodeCount = 0;

    for (Module* mod : db->Nodes) {
        // 包括标准单元、宏、filler（但 filler 面积小，不影响）
        totalArea += mod->width * mod->height;
        nodeCount++;
    }

    if (nodeCount == 0 || totalArea == 0) {
        std::cerr << "Warning: No modules to compute bin grid." << std::endl;
        return;
    }

    double avgNodeArea = totalArea / nodeCount;
    double expectedBinArea = avgNodeArea / targetDensity;

    // === Step 2: 获取布局区域（用 SiteRow bounding box）===
    auto [ll_site, ur_site] = getSiteRowBoundingBox();
    double layoutArea = (ur_site.x - ll_site.x) * (ur_site.y - ll_site.y);

    // === Step 3: 计算期望 bin 数目 M ===
    double M_float = layoutArea / expectedBinArea;
    int M = static_cast<int>(std::ceil(M_float));

    // === Step 4: 确定 bin 网格维度（2^n × 2^n）===
    int n = static_cast<int>(std::floor(std::log2(std::sqrt(M))));
    int binDim = 1 << n; // 2^n

    // 如果太小，尝试 2^(n+1)
    if (binDim * binDim < M) {
        binDim = 1 << (n + 1);
    }

    db->binRows = db->binCols = binDim;
    std::cout << "Initialize bins: " << db->binRows << " x " << db->binCols << std::endl;

    // === Step 5: 创建 bins 网格 ===
    db->bins.assign(db->binRows, std::vector<Bin>(db->binCols));

    double binWidth = (ur_site.x - ll_site.x) / db->binCols;
    double binHeight = (ur_site.y - ll_site.y) / db->binRows;

    for (int i = 0; i < db->binRows; i++) {
        for (int j = 0; j < db->binCols; j++) {
            Bin& bin = db->bins[i][j];
            bin.ll.x = ll_site.x + j * binWidth;
            bin.ll.y = ll_site.y + i * binHeight;
            bin.ur.x = bin.ll.x + binWidth;
            bin.ur.y = bin.ll.y + binHeight;
            bin.center.x = (bin.ll.x + bin.ur.x) / 2.0;
            bin.center.y = (bin.ll.y + bin.ur.y) / 2.0;
            bin.width = binWidth;
            bin.height = binHeight;
            bin.terminalDensity = 0.0;
            bin.darkDensity = 0.0;
        }
    }

    // === Step 6: 计算 terminalDensity（固定宏重叠）===
    for (Module* mod : db->Nodes) {
        if (!mod->isFixed) continue; // 只处理固定宏（terminal）

        // 模块边界
        double mod_llx = mod->center.x - mod->width / 2.0;
        double mod_lly = mod->center.y - mod->height / 2.0;
        double mod_urx = mod->center.x + mod->width / 2.0;
        double mod_ury = mod->center.y + mod->height / 2.0;

        // 遍历所有 bins，计算重叠面积
        for (int i = 0; i < db->binRows; i++) {
            for (int j = 0; j < db->binCols; j++) {
                Bin& bin = db->bins[i][j];
                // 计算矩形重叠面积
                double overlap_x = std::max<double>(0.0, std::min<double>(mod_urx, bin.ur.x) - std::max<double>(mod_llx, bin.ll.x));
                double overlap_y = std::max<double>(0.0, std::min<double>(mod_ury, bin.ur.y) - std::max<double>(mod_lly, bin.ll.y));
                double overlapArea = overlap_x * overlap_y;

                if (overlapArea > 0) {
                    bin.terminalDensity += targetDensity * overlapArea;
                }
            }
        }
    }

    // === Step 7: 计算 darkDensity（bin 超出 SiteRow 的部分）===


    for (int i = 0; i < db->binRows; i++) {
        for (int j = 0; j < db->binCols; j++) {
            // bins[i][j].darkDensity = ... ; // 可选
            // 暂不实现，设为 0
        }
    }

    std::cout << "Bin grid initialized." << std::endl;
}
    
