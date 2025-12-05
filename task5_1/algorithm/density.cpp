// // placement/density.cpp
// #include "density.h"
// #include "../eigen3/Eigen/Dense"
// #include <algorithm>
// #include <cmath>
// #include <iostream>

// double DensityModel::computeCost(const Eigen::VectorXd& x,
//                                  const Eigen::VectorXd& y) const {
//     // --- 1. 初始化 bins ---
//     const double dieW = db->die_xh - db->die_xl;
//     const double dieH = db->die_yh - db->die_yl;
//     const double binW = dieW / nCols;
//     const double binH = dieH / nRows;

//     std::vector<double> binDensity(nRows * nCols, 0.0);
//     std::vector<double> binCapacity(nRows * nCols, binW * binH);

//     // --- 2. 应用死区 mask（固定模块区域）---
//     for (const auto& cell : db->cells) {
//         if (cell.fixed) {
//             int bx = std::clamp(int((cell.x - db->die_xl) / binW), 0, nCols - 1);
//             int by = std::clamp(int((cell.y - db->die_yl) / binH), 0, nRows - 1);
//             binCapacity[by * nCols + bx] = 0.0; // 死区
//         }
//     }

//     // --- 3. 计算当前密度分布 ---
//     for (size_t i = 0; i < db->cells.size(); ++i) {
//         if (db->cells[i].fixed) continue;
//         int bx = std::clamp(int((x(i) - db->die_xl) / binW), 0, nCols - 1);
//         int by = std::clamp(int((y(i) - db->die_yl) / binH), 0, nRows - 1);
//         binDensity[by * nCols + bx] += 1.0; // 每个cell计为1个单位面积
//     }

//     // --- 4. 计算代价 ---
//     double totalCost = 0.0;
//     double rho_target = 1.0; // 目标密度（归一化后）
//     for (int i = 0; i < nRows * nCols; ++i) {
//         if (binCapacity[i] <= 1e-9) continue; // 死区跳过
//         double rho = binDensity[i] / binCapacity[i];
//         double diff = rho - rho_target;
//         totalCost += 0.5 * diff * diff;
//     }

//     return totalCost;
// }

// void DensityModel::computeGradient(const Eigen::VectorXd& x,
//                                    const Eigen::VectorXd& y,
//                                    Eigen::VectorXd& grad_x,
//                                    Eigen::VectorXd& grad_y) const {
//     const double dieW = db->die_xh - db->die_xl;
//     const double dieH = db->die_yh - db->die_yl;
//     const double binW = dieW / nCols;
//     const double binH = dieH / nRows;

//     grad_x.setZero();
//     grad_y.setZero();

//     // 使用简单的“向低密度方向移动”规则
//     for (size_t i = 0; i < db->cells.size(); ++i) {
//         if (db->cells[i].fixed) continue;

//         int bx = std::clamp(int((x(i) - db->die_xl) / binW), 0, nCols - 1);
//         int by = std::clamp(int((y(i) - db->die_yl) / binH), 0, nRows - 1);

//         // 计算周围bin的密度差异
//         double gx = 0.0, gy = 0.0;
//         for (int dy = -1; dy <= 1; ++dy)
//             for (int dx = -1; dx <= 1; ++dx) {
//                 int nx = bx + dx;
//                 int ny = by + dy;
//                 if (nx < 0 || ny < 0 || nx >= nCols || ny >= nRows) continue;
//                 double centerX = db->die_xl + (nx + 0.5) * binW;
//                 double centerY = db->die_yl + (ny + 0.5) * binH;
//                 double diffx = x(i) - centerX;
//                 double diffy = y(i) - centerY;
//                 gx += diffx;
//                 gy += diffy;
//             }
//         grad_x(i) = gx * 1e-4;
//         grad_y(i) = gy * 1e-4;
//     }
// }

// double DensityModel::computeOverflow(const Eigen::VectorXd& x,
//                                      const Eigen::VectorXd& y) const {
//     const double dieW = db->die_xh - db->die_xl;
//     const double dieH = db->die_yh - db->die_yl;
//     const double binW = dieW / nCols;
//     const double binH = dieH / nRows;

//     std::vector<double> binDensity(nRows * nCols, 0.0);
//     std::vector<double> binCapacity(nRows * nCols, binW * binH);

//     for (const auto& cell : db->cells) {
//         if (cell.fixed) {
//             int bx = std::clamp(int((cell.x - db->die_xl) / binW), 0, nCols - 1);
//             int by = std::clamp(int((cell.y - db->die_yl) / binH), 0, nRows - 1);
//             binCapacity[by * nCols + bx] = 0.0;
//         }
//     }

//     for (size_t i = 0; i < db->cells.size(); ++i) {
//         if (db->cells[i].fixed) continue;
//         int bx = std::clamp(int((x(i) - db->die_xl) / binW), 0, nCols - 1);
//         int by = std::clamp(int((y(i) - db->die_yl) / binH), 0, nRows - 1);
//         binDensity[by * nCols + bx] += 1.0;
//     }

//     double totalOverflow = 0.0;
//     for (int i = 0; i < nRows * nCols; ++i) {
//         if (binCapacity[i] <= 1e-9) continue;
//         double rho = binDensity[i] / binCapacity[i];
//         if (rho > 1.0) totalOverflow += (rho - 1.0);
//     }
//     return totalOverflow / (nRows * nCols);
// }
// placement/density.cpp
#include "density.h"
#include "../eigen3/Eigen/Dense"
#include <algorithm>
#include <cmath>
#include <iostream>

double DensityModel::computeCost(const Eigen::VectorXd& x,
                                 const Eigen::VectorXd& y) const {
    const double dieW = db->die_xh - db->die_xl;
    const double dieH = db->die_yh - db->die_yl;
    const double binW = dieW / nCols;
    const double binH = dieH / nRows;

    std::vector<double> binDensity(nRows * nCols, 0.0);
    std::vector<double> binCapacity(nRows * nCols, binW * binH);

    // --- 1. 标记死区（fixed cells）---
    for (const auto& cell : db->cells) {
        if (cell.fixed) {
            int bx = std::clamp(int((cell.x - db->die_xl) / binW), 0, nCols - 1);
            int by = std::clamp(int((cell.y - db->die_yl) / binH), 0, nRows - 1);
            binCapacity[by * nCols + bx] = 0.0; // 不可放置
        }
    }

    // --- 2. 统计当前密度 ---
    double totalCellArea = db->cells.size();  // 简化：每个单元面积 = 1
    double totalBinArea  = dieW * dieH;
    double rho_target = totalCellArea / totalBinArea;

    for (size_t i = 0; i < db->cells.size(); ++i) {
        if (db->cells[i].fixed) continue;
        int bx = std::clamp(int((x(i) - db->die_xl) / binW), 0, nCols - 1);
        int by = std::clamp(int((y(i) - db->die_yl) / binH), 0, nRows - 1);
        binDensity[by * nCols + bx] += 1.0; // 面积单位计数
    }

    // --- 3. 计算密度代价（平方误差）---
    double totalCost = 0.0;
    for (int i = 0; i < nRows * nCols; ++i) {
        if (binCapacity[i] <= 1e-9) continue;
        double rho = (binDensity[i] / binCapacity[i]);
        double diff = rho - rho_target;
        totalCost += 0.5 * diff * diff;
    }
    return totalCost;
}


void DensityModel::computeGradient(const Eigen::VectorXd& x,
                                   const Eigen::VectorXd& y,
                                   Eigen::VectorXd& grad_x,
                                   Eigen::VectorXd& grad_y) const {
    const double dieW = db->die_xh - db->die_xl;
    const double dieH = db->die_yh - db->die_yl;
    const double binW = dieW / nCols;
    const double binH = dieH / nRows;

    grad_x.setZero();
    grad_y.setZero();

    std::vector<double> binDensity(nRows * nCols, 0.0);
    std::vector<double> binCapacity(nRows * nCols, binW * binH);

    // --- 1. 统计当前密度 ---
    double totalCellArea = db->cells.size();
    double totalBinArea  = dieW * dieH;
    double rho_target = totalCellArea / totalBinArea;

    for (size_t i = 0; i < db->cells.size(); ++i) {
        if (db->cells[i].fixed) continue;
        int bx = std::clamp(int((x(i) - db->die_xl) / binW), 0, nCols - 1);
        int by = std::clamp(int((y(i) - db->die_yl) / binH), 0, nRows - 1);
        binDensity[by * nCols + bx] += 1.0;
    }

    // --- 2. 计算每个cell受到的“密度压力” ---
    for (size_t i = 0; i < db->cells.size(); ++i) {
        if (db->cells[i].fixed) continue;

        int bx = std::clamp(int((x(i) - db->die_xl) / binW), 0, nCols - 1);
        int by = std::clamp(int((y(i) - db->die_yl) / binH), 0, nRows - 1);
        double rho_here = binDensity[by * nCols + bx] / binCapacity[by * nCols + bx];
        double pressure = (rho_here - rho_target) / std::max(rho_target, 1e-9);

        // 推离中心方向
        double centerX = db->die_xl + (bx + 0.5) * binW;
        double centerY = db->die_yl + (by + 0.5) * binH;
        grad_x(i) = pressure * (x(i) - centerX) * 1e-3;
        grad_y(i) = pressure * (y(i) - centerY) * 1e-3;
    }
}


double DensityModel::computeOverflow(const Eigen::VectorXd& x,
                                     const Eigen::VectorXd& y) const {
    const double dieW = db->die_xh - db->die_xl;
    const double dieH = db->die_yh - db->die_yl;
    const double binW = dieW / nCols;
    const double binH = dieH / nRows;

    std::vector<double> binDensity(nRows * nCols, 0.0);
    std::vector<double> binCapacity(nRows * nCols, binW * binH);

    double totalCellArea = db->cells.size();
    double totalBinArea  = dieW * dieH;
    double rho_target = totalCellArea / totalBinArea;

    for (size_t i = 0; i < db->cells.size(); ++i) {
        if (db->cells[i].fixed) continue;
        int bx = std::clamp(int((x(i) - db->die_xl) / binW), 0, nCols - 1);
        int by = std::clamp(int((y(i) - db->die_yl) / binH), 0, nRows - 1);
        binDensity[by * nCols + bx] += 1.0;
    }

    double totalOverflow = 0.0;
    int validBins = 0;
    for (int i = 0; i < nRows * nCols; ++i) {
        if (binCapacity[i] <= 1e-9) continue;
        double rho = (binDensity[i] / binCapacity[i]);
        double overflow = std::max(0.0, rho - rho_target);
        totalOverflow += overflow;
        validBins++;
    }

    return validBins ? totalOverflow / (rho_target * validBins) : 0.0;
}

