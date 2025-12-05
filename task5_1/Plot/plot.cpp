#include "plot.h"
#include <iostream>
#include <limits>
#include <algorithm>

using namespace cimg_library;

namespace PLOTTING {

void plotPlacement(const std::string& imageName, const PlacementDB* db) {
    using std::cerr;
    using std::cout;
    using std::endl;

    // --- 0) 安全检查 ---
    if (!db) {
        cerr << "[ERR] plotPlacement: db is null.\n";
        return;
    }
    if (db->cells.empty()) {
        cerr << "[WARN] plotPlacement: no cells to draw.\n";
        return;
    }

    // --- 1) 确定芯片边界 ---
    static bool regionFixed = false;
    static double llx, lly, urx, ury;

    if (!regionFixed) {
        llx = db->die_xl;
        lly = db->die_yl;
        urx = db->die_xh;
        ury = db->die_yh;

        // 若未指定 DIEAREA，自动根据单元计算
        if (urx - llx < 1e-9 || ury - lly < 1e-9) {
            llx = lly = std::numeric_limits<double>::infinity();
            urx = ury = -std::numeric_limits<double>::infinity();
            for (const auto& c : db->cells) {
                llx = std::min(llx, c.x - c.width / 2);
                lly = std::min(lly, c.y - c.height / 2);
                urx = std::max(urx, c.x + c.width / 2);
                ury = std::max(ury, c.y + c.height / 2);
            }
            double pad = 0.05;
            double w = urx - llx, h = ury - lly;
            llx -= pad * w; lly -= pad * h;
            urx += pad * w; ury += pad * h;
        }
        regionFixed = true;
    }

    const double dieW = urx - llx;
    const double dieH = ury - lly;

    // --- 2) 图像设置 ---
    const int canvasW = 1000;
    const int canvasH = 1000;
    const int margin = 20;

    const double unitX = canvasW / dieW;
    const double unitY = canvasH / dieH;

    CImg<unsigned char> img(canvasW + 2 * margin, canvasH + 2 * margin, 1, 3, 255);

    const unsigned char Gray[3]  = {180, 180, 180};
    const unsigned char Red[3]   = {255, 0, 0};
    const unsigned char Blue[3]  = {0, 0, 255};
    const unsigned char Green[3] = {0, 200, 0};
    const unsigned char Black[3] = {0, 0, 0};

    // 绘制芯片边框
    int l = margin, t = margin;
    int r = margin + canvasW, b = margin + canvasH;
    img.draw_rectangle(l, t, r, b, Gray, 1.0f);

    // --- 3) 绘制单元 ---
    for (const auto& c : db->cells) {
        double llcx = (c.x - c.width / 2 - llx) * unitX + margin;
        double llyc = (c.y - c.height / 2 - lly) * unitY + margin;
        double urcx = (c.x + c.width / 2 - llx) * unitX + margin;
        double uryc = (c.y + c.height / 2 - lly) * unitY + margin;

        // 坐标转换为像素坐标（Y 翻转）
        int x1 = static_cast<int>(llcx);
        int x2 = static_cast<int>(urcx);
        int y1 = static_cast<int>(canvasH - (uryc - margin)) + margin;
        int y2 = static_cast<int>(canvasH - (llyc - margin)) + margin;

        if (x1 > x2) std::swap(x1, x2);
        if (y1 > y2) std::swap(y1, y2);

        const unsigned char* color = c.fixed ? Blue : Red;
        img.draw_rectangle(x1, y1, x2, y2, color, 0.7f);
    }

    // --- 4) 输出 ---
    img.draw_text(30, 20, imageName.c_str(), Black, NULL, 1, 24);
    std::string outPath = imageName + ".bmp";
    img.save_bmp(outPath.c_str());
    cout << "[PLOT] placement saved: " << outPath << endl;
}

void plotPlacement(const std::string& imageName, const PlaceData* db)
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

        // db->chipRegion.ll = fixedLL;
        // db->chipRegion.ur = fixedUR;

        firstPlotDone = true;
    }
    // else {
    //     // 后续绘图使用固定 region，不改动
    //     db->chipRegion.ll = fixedLL;
    //     db->chipRegion.ur = fixedUR;
    // }

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

void plotPlacementSimple(const std::string& basename, const PlaceData& db) {
    const int IMG_W = 2000;
    const int IMG_H = 2000;

    // -------- 1) 统计所有模块（Nodes + Terminals）的包围盒 --------
    bool first = true;
    double minx = 0, maxx = 0, miny = 0, maxy = 0;

    auto updateBBox = [&](const Module* m) {
        if (!m) return;
        double x0 = m->center.x - 0.5 * m->width;
        double x1 = m->center.x + 0.5 * m->width;
        double y0 = m->center.y - 0.5 * m->height;
        double y1 = m->center.y + 0.5 * m->height;
        if (first) {
            minx = x0; maxx = x1;
            miny = y0; maxy = y1;
            first = false;
        } else {
            minx = std::min(minx, x0);
            maxx = std::max(maxx, x1);
            miny = std::min(miny, y0);
            maxy = std::max(maxy, y1);
        }
    };

    for (const Module* m : db.Nodes)     updateBBox(m);
    for (const Module* m : db.Terminals) updateBBox(m);

    if (first) {
        std::cerr << "[plotPlacementSimple] No modules to draw.\n";
        return;
    }

    double w = maxx - minx;
    double h = maxy - miny;
    if (w <= 0) w = 1;
    if (h <= 0) h = 1;

    // -------- 2) 计算缩放比例和边距 --------
    double scaleX = (IMG_W * 0.9) / w;
    double scaleY = (IMG_H * 0.9) / h;
    double scale  = std::min(scaleX, scaleY);

    double marginX = 0.5 * (IMG_W - scale * w);
    double marginY = 0.5 * (IMG_H - scale * h);

    auto mapX = [&](double x) {
        return marginX + (x - minx) * scale;
    };
    auto mapY = [&](double y) {
        // Y 轴翻转，SVG 左上角是 (0,0)
        return IMG_H - (marginY + (y - miny) * scale);
    };

    // -------- 3) 输出 SVG --------
    std::ofstream ofs(basename + ".svg");
    if (!ofs) {
        std::cerr << "[plotPlacementSimple] Cannot open output file.\n";
        return;
    }

    ofs << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << IMG_W << "\" height=\"" << IMG_H << "\">\n";

    // 背景灰色
    ofs << "<rect x=\"0\" y=\"0\" width=\"" << IMG_W
        << "\" height=\"" << IMG_H
        << "\" fill=\"#eeeeee\" stroke=\"none\"/>\n";

    auto drawModule = [&](const Module* m, const char* fillColor) {
        if (!m) return;
        double x0 = mapX(m->center.x - 0.5 * m->width);
        double x1 = mapX(m->center.x + 0.5 * m->width);
        double y0 = mapY(m->center.y + 0.5 * m->height);
        double y1 = mapY(m->center.y - 0.5 * m->height);

        double rw = std::max(1.0, x1 - x0);
        double rh = std::max(1.0, y1 - y0);

        ofs << "<rect x=\"" << x0 << "\" y=\"" << y0
            << "\" width=\"" << rw << "\" height=\"" << rh
            << "\" fill=\"" << fillColor
            << "\" stroke=\"#000000\" stroke-width=\"0.5\"/>\n";
    };

    // 可动单元画成绿色，终端画成红色，这样一眼能区分
    for (const Module* m : db.Nodes)     drawModule(m, "#66cc66");
    for (const Module* m : db.Terminals) drawModule(m, "#cc6666");

    ofs << "</svg>\n";
    ofs.close();

    std::cerr << "[plotPlacementSimple] Output written to "
              << basename << ".svg\n";
}

} // namespace PLOTTING

