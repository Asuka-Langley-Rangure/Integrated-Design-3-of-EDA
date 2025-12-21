#include "plot.h"
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace PLOTTING;

void PLOTTING::plotPlacement(std::string imageName, PlaceData* db)
{
    using std::cout;
    using std::endl;

    // 0) 安全检查
    if (!db) {
        std::cerr << "[ERR] db is null.\n";
        return;
    }
    if (db->Nodes.empty()) {
        std::cerr << "[WARN] no modules to plot.\n";
    }

    // 1) 第一次绘图确定 chipRegion，之后固定
    static bool firstPlotDone = false;
    static POS_2D fixedLL = {0.0f, 0.0f};
    static POS_2D fixedUR = {0.0f, 0.0f};

    if (!firstPlotDone) {
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

        double w = std::max(1.0, maxx - minx);
        double h = std::max(1.0, maxy - miny);
        double pad = 0.05;
        double px = w * pad;
        double py = h * pad;

        fixedLL.x = static_cast<float>(minx - px);
        fixedLL.y = static_cast<float>(miny - py);
        fixedUR.x = static_cast<float>(maxx + px);
        fixedUR.y = static_cast<float>(maxy + py);

        db->chipRegion.ll = fixedLL;
        db->chipRegion.ur = fixedUR;

        firstPlotDone = true;
    }
    else {
        // 后续绘图使用固定 region，不改动
        db->chipRegion.ll = fixedLL;
        db->chipRegion.ur = fixedUR;
    }

    // 2) 计算芯片区域与图像大小
    const double chip_ll_x = db->chipRegion.ll.x;
    const double chip_ll_y = db->chipRegion.ll.y;
    const double chip_ur_x = db->chipRegion.ur.x;
    const double chip_ur_y = db->chipRegion.ur.y;

    double chipRegionWidth  = std::max(1e-9, chip_ur_x - chip_ll_x);
    double chipRegionHeight = std::max(1e-9, chip_ur_y - chip_ll_y);

    int imageLength = 1000;
    int xMargin = 20, yMargin = 20;

    int imageWidth  = imageLength;
    int imageHeight = imageLength; // 固定正方画布，避免比例变化

    // 3) 新建画布（含边距）
    CImg<unsigned char> img(
        imageWidth + 2 * xMargin,
        imageHeight + 2 * yMargin,
        1, 3, 255
    );

    // 坐标缩放：芯片坐标 -> 像素
    const double unitX = (double)imageWidth  / chipRegionWidth;
    const double unitY = (double)imageHeight / chipRegionHeight;

    // 定义颜色（RGB）
    const unsigned char Red[3]    = {255,   0,   0};
    const unsigned char Green[3]  = {  0, 200,   0};
    const unsigned char Blue[3]   = {  0,   0, 255};
    const unsigned char Orange[3] = {255, 128,   0};
    const unsigned char Gray[3]   = {180, 180, 180};
    const unsigned char Black[3]  = {  0,   0,   0};

    float opacity = 0.7f;

    // 绘制芯片边框
    const int frame_l = xMargin;
    const int frame_t = yMargin;
    const int frame_r = xMargin + imageWidth;
    const int frame_b = yMargin + imageHeight;
    img.draw_line(frame_l, frame_t, frame_r, frame_t, Gray);
    img.draw_line(frame_r, frame_t, frame_r, frame_b, Gray);
    img.draw_line(frame_r, frame_b, frame_l, frame_b, Gray);
    img.draw_line(frame_l, frame_b, frame_l, frame_t, Gray);

    // 4) 遍历模块并绘制
    for (size_t k = 0; k < db->Nodes.size(); ++k) {
        Module* curNode = db->Nodes[k];
        if (!curNode) continue;

        double llx = (double)curNode->center.x - (double)curNode->width  * 0.5;
        double lly = (double)curNode->center.y - (double)curNode->height * 0.5;
        double urx = (double)curNode->center.x + (double)curNode->width  * 0.5;
        double ury = (double)curNode->center.y + (double)curNode->height * 0.5;

        double ll_rel_x = llx - chip_ll_x;
        double ll_rel_y = lly - chip_ll_y;
        double ur_rel_x = urx - chip_ll_x;
        double ur_rel_y = ury - chip_ll_y;

        int x_left  = static_cast<int>(std::llround(ll_rel_x * unitX)) + xMargin;
        int x_right = static_cast<int>(std::llround(ur_rel_x * unitX)) + xMargin;

        int y_top    = static_cast<int>(std::llround((chipRegionHeight - ur_rel_y) * unitY)) + yMargin;
        int y_bottom = static_cast<int>(std::llround((chipRegionHeight - ll_rel_y) * unitY)) + yMargin;

        if (x_left > x_right)   std::swap(x_left, x_right);
        if (y_top  > y_bottom)  std::swap(y_top, y_bottom);

        const unsigned char* color_fill =
            curNode->isMacro  ? Green :
            curNode->isFixed  ? Blue  :
            curNode->isFiller ? Orange:
                                Red;

        img.draw_rectangle(x_left, y_top, x_right, y_bottom, color_fill, opacity);
        // if (color_fill == Red){
        //     std::cout << "x_left: " << x_left << ", y_top: " << y_top << ", x_right: " << x_right << ", y_bottom: " << y_bottom << std::endl;
        // }
    }

    // 5) 标题与输出
    img.draw_text(30, 20, imageName.c_str(), Black, NULL, 1, 24);
    std::string outPath = imageName + ".bmp";
    img.save_bmp(outPath.c_str());
    cout << "bitmap file has been saved: " << outPath << endl;
}
