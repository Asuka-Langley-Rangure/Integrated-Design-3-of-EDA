// #ifndef PLOT_H
// #define PLOT_H
// #include "common.h"

// #define cimg_display 0
// #include "../Library/CImg/CImg.h"
// #include "../algorithm/db.h"

// using namespace cimg_library;
// namespace PLOTTING
// {
//     const unsigned char Black[] = {0, 0, 0},
//                         Green[] = {0, 150, 0},
//                         Red[] = {255, 0, 0};
//     void plotPlacement(const std::string& imageName, const PlacementDB* db);

// }

// #endif

#ifndef PLOT_H
#define PLOT_H

#include <string>
#include "../algorithm/db.h"

// ======================================================
// SVG 绘图模块：PLOTTING 命名空间
// ======================================================
//
// 主要功能：
//   - 将 PlacementDB 中的单元坐标与芯片区域导出为 SVG 图像
//   - 不依赖 CImg / OpenCV / Eigen，仅使用标准库
//   - 输出文件可直接用浏览器打开查看
//
// ======================================================

namespace PLOTTING {

/// @brief 导出布局为 SVG 图片
/// @param filename 输出 SVG 文件名（例如 "placement.svg"）
/// @param db 指向布图数据库 PlacementDB 的指针
void plotPlacementSVG(const std::string& filename, const PlacementDB* db);

}  // namespace PLOTTING

#endif  // PLOT_H
