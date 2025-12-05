#include "../Library/CImg/CImg.h"
#include "../algorithm/db.h"
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

} // namespace PLOTTING

