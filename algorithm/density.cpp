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


void DensityModel::computeGradient(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& y,
    Eigen::VectorXd& grad_x,
    Eigen::VectorXd& grad_y) const
{
    const double xl = db->die_xl;
    const double yl = db->die_yl;
    const double dieW = db->die_xh - xl;
    const double dieH = db->die_yh - yl;

    const double binW = dieW / nCols;
    const double binH = dieH / nRows;

    grad_x.setZero();
    grad_y.setZero();

    // ======================================================
    // 1) 计算每个 bin 的电荷密度 rho(b)
    // ======================================================
    std::vector<double> rho(nRows * nCols, 0.0);

    for (size_t i = 0; i < db->cells.size(); ++i) {
        if (db->cells[i].fixed) continue;

        double cx = x(i);
        double cy = y(i);

        // cell 面积（简化版）
        double q = db->cells[i].width * db->cells[i].height;

        double fx = (cx - xl) / binW;
        double fy = (cy - yl) / binH;

        int bx = floor(fx);
        int by = floor(fy);
        double tx = fx - bx;
        double ty = fy - by;

        // Triangle bump: W(t) = 1 - |t|
        double Wx0 = 1.0 - tx;
        double Wx1 = tx;
        double Wy0 = 1.0 - ty;
        double Wy1 = ty;

        // deposit to 4 neighbors
        auto add_rho = [&](int xx, int yy, double w){
            if (xx >= 0 && yy >= 0 && xx < nCols && yy < nRows)
                rho[yy*nCols + xx] += q * w;
        };

        add_rho(bx,   by,   Wx0 * Wy0);
        add_rho(bx+1, by,   Wx1 * Wy0);
        add_rho(bx,   by+1, Wx0 * Wy1);
        add_rho(bx+1, by+1, Wx1 * Wy1);
    }

    // 目标密度（均匀分布）
    double totalCharge = db->cells.size();  // q=1 情况
    double rho_target = totalCharge / (nCols * nRows);

    // ======================================================
    // 2) 求解 Poisson 方程 ∇²ψ = rho - rho_target（Gauss-Seidel relax）
    // ======================================================
    std::vector<double> psi(nRows * nCols, 0.0);
    std::vector<double> psi_new(nRows * nCols, 0.0);

    const int relax_iters = 30;

    for (int it = 0; it < relax_iters; ++it) {
        for (int yb = 1; yb < nRows-1; ++yb)
        for (int xb = 1; xb < nCols-1; ++xb) {
            int id = yb*nCols + xb;

            double rhs = rho[id] - rho_target;

            psi_new[id] =
                0.25 * (psi[id-1] + psi[id+1] +
                        psi[id-nCols] + psi[id+nCols] +
                        rhs);
        }
        psi.swap(psi_new);
    }

    // ======================================================
    // 3) 电场 = -∇ψ
    // ======================================================
    auto Ex = [&](int xb, int yb){
        if (xb <= 0 || xb >= nCols-1) return 0.0;
        return -(psi[yb*nCols + xb+1] - psi[yb*nCols + xb-1]) / (2*binW);
    };

    auto Ey = [&](int xb, int yb){
        if (yb <= 0 || yb >= nRows-1) return 0.0;
        return -(psi[(yb+1)*nCols + xb] - psi[(yb-1)*nCols + xb]) / (2*binH);
    };

    // ======================================================
    // 4) 对每个 cell 计算密度梯度 force = q * electric field
    // ======================================================
    for (size_t i = 0; i < db->cells.size(); ++i) {
        if (db->cells[i].fixed) continue;

        double cx = x(i);
        double cy = y(i);

        double q = db->cells[i].width * db->cells[i].height;

        double fx = (cx - xl) / binW;
        double fy = (cy - yl) / binH;

        int bx = floor(fx);
        int by = floor(fy);
        double tx = fx - bx;
        double ty = fy - by;

        double Wx0 = 1.0 - tx;
        double Wx1 = tx;
        double Wy0 = 1.0 - ty;
        double Wy1 = ty;

        auto add_force = [&](int xx, int yy, double w){
            if (xx >= 0 && yy >= 0 && xx < nCols && yy < nRows) {
                int id = yy*nCols + xx;
                grad_x(i) += q * Ex(xx,yy) * w;
                grad_y(i) += q * Ey(xx,yy) * w;
            }
        };

        add_force(bx,   by,   Wx0 * Wy0);
        add_force(bx+1, by,   Wx1 * Wy0);
        add_force(bx,   by+1, Wx0 * Wy1);
        add_force(bx+1, by+1, Wx1 * Wy1);
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

