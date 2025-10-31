#include "plot.h"
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace PLOTTING;

void PLOTTING::plotPlacement(std::string imageName, PlaceData* db)
{
    using std::cout; using std::endl;

    // ---------------------------
    // 0) 安全检查
    // ---------------------------
    if (!db) { std::cerr << "[ERR] db is null.\n"; return; }
    if (db->Nodes.empty()) { std::cerr << "[WARN] no modules to plot.\n"; }

    // ---------------------------
    // 1) 以模块几何为基准，修正/扩展 chipRegion（5% 留白）
    // ---------------------------
    auto expandChipRegionFromModules = [&](PlaceData* db) {
        double minx =  std::numeric_limits<double>::infinity();
        double miny =  std::numeric_limits<double>::infinity();
        double maxx = -std::numeric_limits<double>::infinity();
        double maxy = -std::numeric_limits<double>::infinity();

        for (Module* m : db->Nodes) {
            if (!m) continue;
            POS_2D ll = m->getLL_2D();
            POS_2D ur = m->getUR_2D();
            minx = std::min(minx, (double)ll.x);
            miny = std::min(miny, (double)ll.y);
            maxx = std::max(maxx, (double)ur.x);
            maxy = std::max(maxy, (double)ur.y);
        }
        if (!std::isfinite(minx) || !std::isfinite(miny) ||
            !std::isfinite(maxx) || !std::isfinite(maxy)) {
            // 兜底：若没有有效模块，保持原 region
        }
        db->chipRegion.ll.x = std::min(db->chipRegion.ll.x, (float)minx);
        db->chipRegion.ll.y = std::min(db->chipRegion.ll.y, (float)miny);
        db->chipRegion.ur.x = std::max(db->chipRegion.ur.x, (float)maxx);
        db->chipRegion.ur.y = std::max(db->chipRegion.ur.y, (float)maxy);

        // 若原 chipRegion 太小或未覆盖，按模块盒子扩展
        double w0 = db->chipRegion.ur.x - db->chipRegion.ll.x;
        double h0 = db->chipRegion.ur.y - db->chipRegion.ll.y;

        double minx_all = (double)db->chipRegion.ll.x;
        double miny_all = (double)db->chipRegion.ll.y;
        double maxx_all = (double)db->chipRegion.ur.x;
        double maxy_all = (double)db->chipRegion.ur.y;

        double w = std::max(1.0, maxx_all - minx_all);
        double h = std::max(1.0, maxy_all - miny_all);
        double pad = 0.05; // 5% padding
        double px = w * pad, py = h * pad;

        db->chipRegion.ll.x = minx_all - px;
        db->chipRegion.ll.y = miny_all - py;
        db->chipRegion.ur.x = maxx_all + px;
        db->chipRegion.ur.y = maxy_all + py;
    };

    expandChipRegionFromModules(db);

    // ---------------------------
    // 2) 计算芯片区域与图像大小
    // ---------------------------
    const double chip_ll_x = db->chipRegion.ll.x;
    const double chip_ll_y = db->chipRegion.ll.y;
    const double chip_ur_x = db->chipRegion.ur.x;
    const double chip_ur_y = db->chipRegion.ur.y;

    double chipRegionWidth  = std::max(1e-9, (double)chip_ur_x - chip_ll_x);
    double chipRegionHeight = std::max(1e-9, (double)chip_ur_y - chip_ll_y);

    int imageLength = 1000; // 画布参考宽度，可按需调大让更清晰
    int xMargin = 20, yMargin = 20;

    // 保持长宽比：imageWidth 固定为 imageLength，imageHeight 按比例计算
    int imageWidth  = imageLength;
    int imageHeight = (int)std::ceil(chipRegionHeight * (imageLength / chipRegionWidth));
    imageHeight = std::max(imageHeight, 1); // 防 0

    // ---------------------------
    // 3) 新建画布（含边距）
    // ---------------------------
    CImg<unsigned char> img(
        imageWidth + 2 * xMargin,
        imageHeight + 2 * yMargin,
        1, 3, 255
    );

    // 坐标缩放：芯片坐标 -> 像素
    const double unitX = (double)imageWidth  / chipRegionWidth;
    const double unitY = (double)imageHeight / chipRegionHeight;

    // 定义颜色（RGB）
    const unsigned char Red[3]    = {255,   0,   0};   // 标准单元（红色）
    const unsigned char Green[3]  = {  0, 200,   0};   // 宏单元（绿色）
    const unsigned char Blue[3]   = {  0,   0, 255};   // 固定单元（蓝色）
    const unsigned char Orange[3] = {255, 128,   0};   // 填充单元（橙色）
    const unsigned char Gray[3]   = {180, 180, 180};   // 边框（灰色）
    const unsigned char Black[3]  = {  0,   0,   0};   // 黑色（用于绘制边框）

    float opacity = 0.7f;

    // 绘制芯片边框
    const int frame_l = xMargin;
    const int frame_t = yMargin;
    const int frame_r = xMargin + imageWidth;
    const int frame_b = yMargin + imageHeight;
    img.draw_line(frame_l, frame_t, frame_r, frame_t, Gray);  // 顶部
    img.draw_line(frame_r, frame_t, frame_r, frame_b, Gray);  // 右侧
    img.draw_line(frame_r, frame_b, frame_l, frame_b, Gray);  // 底部
    img.draw_line(frame_l, frame_b, frame_l, frame_t, Gray);  // 左侧

    // ---------------------------
    // 4) 遍历模块并绘制（含详尽调试）
    // ---------------------------

    for (size_t k = 0; k < db->Nodes.size(); ++k) {
        Module* curNode = db->Nodes[k];
        if (!curNode) continue;

        // 获取模块的左下角（LL）和右上角（UR）坐标
        const double llx = (double)curNode->center.x - (double)curNode->width  * 0.5;
        const double lly = (double)curNode->center.y - (double)curNode->height * 0.5;
        const double urx = (double)curNode->center.x + (double)curNode->width  * 0.5;
        const double ury = (double)curNode->center.y + (double)curNode->height * 0.5;

        // 相对 chip LL 的坐标
        double ll_rel_x = llx - chip_ll_x;
        double ll_rel_y = lly - chip_ll_y;
        double ur_rel_x = urx - chip_ll_x;
        double ur_rel_y = ury - chip_ll_y;

        // 映射到图像坐标系
        int x_left  = (int)std::llround(ll_rel_x * unitX) + xMargin;
        int x_right = (int)std::llround(ur_rel_x * unitX) + xMargin;

        // **Y 轴镜像关键**：UR -> top, LL -> bottom
        int y_top    = (int)std::llround((chipRegionHeight - ur_rel_y) * unitY) + yMargin;
        int y_bottom = (int)std::llround((chipRegionHeight - ll_rel_y) * unitY) + yMargin;

        // 兜底确保 left<right, top<bottom
        if (x_left > x_right)   std::swap(x_left, x_right);
        if (y_top  > y_bottom)  std::swap(y_top, y_bottom);

        int px_w = x_right - x_left;
        int px_h = y_bottom - y_top;

        // 根据模块类型选择颜色
        const unsigned char* color_fill =
            curNode->isMacro  ? Green :
            curNode->isFixed  ? Blue  :
            curNode->isFiller ? Orange:
                                Red;

        // 边界检查：要对比实际画布尺寸（含边距），即 img.width()/img.height()
        bool out = (x_left  < 0 || y_top < 0 ||
                    x_right > img.width() ||
                    y_bottom > img.height());

        // 实际绘制
        img.draw_rectangle(
            x_left, y_top, x_right, y_bottom,
            color_fill,
            opacity
        );
    }

    // ---------------------------
    // 5) 标题与输出
    // ---------------------------
    img.draw_text(30, 20, imageName.c_str(), Black, NULL, 1, 24);
    std::string outPath = imageName + ".bmp";
    img.save_bmp(outPath.c_str());
    cout << "bitmap file has been saved: " << outPath << endl;
}