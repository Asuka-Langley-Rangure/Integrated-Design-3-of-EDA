// #include "../Library/CImg/CImg.h"
// #include "../algorithm/db.h"
// #include <iostream>
// #include <limits>
// #include <algorithm>

// using namespace cimg_library;

// namespace PLOTTING {

// void plotPlacement(const std::string& imageName, const PlacementDB* db) {
//     using std::cerr;
//     using std::cout;
//     using std::endl;

//     // --- 0) 安全检查 ---
//     if (!db) {
//         cerr << "[ERR] plotPlacement: db is null.\n";
//         return;
//     }
//     if (db->cells.empty()) {
//         cerr << "[WARN] plotPlacement: no cells to draw.\n";
//         return;
//     }

//     // --- 1) 确定芯片边界 ---
//     static bool regionFixed = false;
//     static double llx, lly, urx, ury;

//     if (!regionFixed) {
//         llx = db->die_xl;
//         lly = db->die_yl;
//         urx = db->die_xh;
//         ury = db->die_yh;

//         // 若未指定 DIEAREA，自动根据单元计算
//         if (urx - llx < 1e-9 || ury - lly < 1e-9) {
//             llx = lly = std::numeric_limits<double>::infinity();
//             urx = ury = -std::numeric_limits<double>::infinity();
//             for (const auto& c : db->cells) {
//                 llx = std::min(llx, c.x - c.width / 2);
//                 lly = std::min(lly, c.y - c.height / 2);
//                 urx = std::max(urx, c.x + c.width / 2);
//                 ury = std::max(ury, c.y + c.height / 2);
//             }
//             double pad = 0.05;
//             double w = urx - llx, h = ury - lly;
//             llx -= pad * w; lly -= pad * h;
//             urx += pad * w; ury += pad * h;
//         }
//         regionFixed = true;
//     }

//     const double dieW = urx - llx;
//     const double dieH = ury - lly;

//     // --- 2) 图像设置 ---
//     const int canvasW = 1000;
//     const int canvasH = 1000;
//     const int margin = 20;

//     const double unitX = canvasW / dieW;
//     const double unitY = canvasH / dieH;

//     CImg<unsigned char> img(canvasW + 2 * margin, canvasH + 2 * margin, 1, 3, 255);

//     const unsigned char Gray[3]  = {180, 180, 180};
//     const unsigned char Red[3]   = {255, 0, 0};
//     const unsigned char Blue[3]  = {0, 0, 255};
//     const unsigned char Green[3] = {0, 200, 0};
//     const unsigned char Black[3] = {0, 0, 0};

//     // 绘制芯片边框
//     int l = margin, t = margin;
//     int r = margin + canvasW, b = margin + canvasH;
//     img.draw_rectangle(l, t, r, b, Gray, 1.0f);

//     // --- 3) 绘制单元 ---
//     for (const auto& c : db->cells) {
//         double llcx = (c.x - c.width / 2 - llx) * unitX + margin;
//         double llyc = (c.y - c.height / 2 - lly) * unitY + margin;
//         double urcx = (c.x + c.width / 2 - llx) * unitX + margin;
//         double uryc = (c.y + c.height / 2 - lly) * unitY + margin;

//         // 坐标转换为像素坐标（Y 翻转）
//         int x1 = static_cast<int>(llcx);
//         int x2 = static_cast<int>(urcx);
//         int y1 = static_cast<int>(canvasH - (uryc - margin)) + margin;
//         int y2 = static_cast<int>(canvasH - (llyc - margin)) + margin;

//         if (x1 > x2) std::swap(x1, x2);
//         if (y1 > y2) std::swap(y1, y2);

//         const unsigned char* color = c.fixed ? Blue : Red;
//         img.draw_rectangle(x1, y1, x2, y2, color, 0.7f);
//     }

//     // --- 4) 输出 ---
//     img.draw_text(30, 20, imageName.c_str(), Black, NULL, 1, 24);
//     std::string outPath = imageName + ".bmp";
//     img.save_bmp(outPath.c_str());
//     cout << "[PLOT] placement saved: " << outPath << endl;
// }

// } // namespace PLOTTING

#include "../algorithm/db.h"
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>

namespace PLOTTING {

void plotPlacementSVG(const std::string& filename, const PlacementDB* db) {
    using std::cerr;
    using std::cout;
    using std::endl;

    if (!db) {
        cerr << "[ERR] plotPlacementSVG: db is null.\n";
        return;
    }
    if (db->cells.empty()) {
        cerr << "[WARN] plotPlacementSVG: no cells to draw.\n";
        return;
    }

    // =====================================================
    // 1) 计算布局边界（真实 min/max）
    // =====================================================
    double llx = 1e100, lly = 1e100;
    double urx = -1e100, ury = -1e100;

    for (const auto& c : db->cells) {
        llx = std::min(llx, c.x - c.width / 2);
        lly = std::min(lly, c.y - c.height / 2);
        urx = std::max(urx, c.x + c.width / 2);
        ury = std::max(ury, c.y + c.height / 2);
    }

    double dieW = urx - llx;
    double dieH = ury - lly;

    // 给 5% margin
    double padX = dieW * 0.05;
    double padY = dieH * 0.05;
    llx -= padX; lly -= padY;
    urx += padX; ury += padY;
    dieW = urx - llx;
    dieH = ury - lly;

    // =====================================================
    // 2) SVG 画布
    // =====================================================
    const int canvasW = 1200;
    const int canvasH = (int)(canvasW * (dieH / dieW));
    const int margin = 20;

    std::ofstream out(filename);
    if (!out) {
        cerr << "[ERR] cannot open " << filename << " for writing.\n";
        return;
    }

    out << "<svg xmlns='http://www.w3.org/2000/svg' "
        << "width='" << canvasW + margin*2 << "' "
        << "height='" << canvasH + margin*2 << "' "
        << "style='background:white'>\n";

    // =====================================================
    // 3) 缩放参数
    // =====================================================
    double scaleX = canvasW / dieW;
    double scaleY = canvasH / dieH;

    auto mapX = [&](double x){ return (x - llx) * scaleX + margin; };
    auto mapY = [&](double y){
        double py = (y - lly) * scaleY;
        return canvasH + margin - py;
    };

    // =====================================================
    // 4) 先绘制真实 net 连接线（pin-to-pin）
    // =====================================================
    out << "<g stroke='#0096FF' stroke-width='0.5' stroke-opacity='0.6'>\n";

    for (const auto& net : db->nets) {
        if (net.pins.size() < 2) continue;

        for (size_t k = 0; k + 1 < net.pins.size(); ++k) {
            int i1 = net.pins[k];
            int i2 = net.pins[k+1];

            const auto& c1 = db->cells[i1];
            const auto& c2 = db->cells[i2];

            double x1 = mapX(c1.x);
            double y1 = mapY(c1.y);
            double x2 = mapX(c2.x);
            double y2 = mapY(c2.y);

            out << "<line x1='" << x1 << "' y1='" << y1
                << "' x2='" << x2 << "' y2='" << y2
                << "' />\n";
        }
    }
    out << "</g>\n";

    // =====================================================
    // 5) 绘制单元（带颜色）
    // =====================================================
    out << "<g stroke='black' stroke-width='0.2'>\n";

    for (const auto& c : db->cells) {
        double x1 = mapX(c.x - c.width/2);
        double x2 = mapX(c.x + c.width/2);
        double y1 = mapY(c.y + c.height/2);
        double y2 = mapY(c.y - c.height/2);

        double w = x2 - x1;
        double h = y2 - y1;

        std::string color = c.fixed ? "#1E90FF" : "#FF4C4C"; // 蓝=固定 红=可动

        out << "<rect x='" << x1 << "' y='" << y1
            << "' width='" << w << "' height='" << h
            << "' fill='" << color
            << "' fill-opacity='0.75'/>\n";
    }

    out << "</g>\n";

    // =====================================================
    // 6) 文本标题
    // =====================================================
    out << "<text x='" << margin
        << "' y='" << margin + 20
        << "' font-size='20' fill='black'>Placement (pin-to-pin nets)</text>\n";

    out << "</svg>";
    out.close();

    cout << "[PLOT] SVG saved: " << filename << endl;
}

} // namespace PLOTTING
