#include "plot.h"
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace PLOTTING;

// 是否绘制 FILLER（默认不画，避免一屏 1px 细线干扰观察）
static constexpr bool kDrawFillers = false;

// 极薄/极窄单元最小像素保护：至少 1×1 px
static inline void ensure_min_pixel_rect(int &x_left, int &y_top, int &x_right, int &y_bottom) {
    if (x_left > x_right)   std::swap(x_left, x_right);
    if (y_top  > y_bottom)  std::swap(y_top, y_bottom);
    if (x_right  <= x_left)   x_right  = x_left  + 1;
    if (y_bottom <= y_top)    y_bottom = y_top   + 1;
}

void PLOTTING::plotPlacement(std::string imageName, PlaceData* db)
{
    using std::cout; using std::endl;

    // 0) 安全检查
    if (!db) { std::cerr << "[ERR] plotPlacement: db is null.\n"; return; }
    if (db->Nodes.empty()) { std::cerr << "[WARN] plotPlacement: no modules to plot.\n"; }

    // 1) 若 chipRegion 无效（宽高≈0），用模块范围自动扩展并加 5% 边距
    {
        const double w0 = (double)db->chipRegion.ur.x - (double)db->chipRegion.ll.x;
        const double h0 = (double)db->chipRegion.ur.y - (double)db->chipRegion.ll.y;
        if (w0 < 1e-6 || h0 < 1e-6) {
            double minx =  std::numeric_limits<double>::infinity();
            double miny =  std::numeric_limits<double>::infinity();
            double maxx = -std::numeric_limits<double>::infinity();
            double maxy = -std::numeric_limits<double>::infinity();

            for (Module* m : db->Nodes) {
                if (!m) continue;
                // ★ 用当前 center/width/height 动态计算盒子（不再依赖 getLL_2D/getUR_2D）
                const double llx = (double)m->center.x - (double)m->width  * 0.5;
                const double lly = (double)m->center.y - (double)m->height * 0.5;
                const double urx = (double)m->center.x + (double)m->width  * 0.5;
                const double ury = (double)m->center.y + (double)m->height * 0.5;
                minx = std::min(minx, llx);
                miny = std::min(miny, lly);
                maxx = std::max(maxx, urx);
                maxy = std::max(maxy, ury);
            }
            if (std::isfinite(minx) && std::isfinite(miny) &&
                std::isfinite(maxx) && std::isfinite(maxy) &&
                maxx > minx && maxy > miny) {
                const double w = std::max(1.0, maxx - minx);
                const double h = std::max(1.0, maxy - miny);
                const double px = 0.05 * w, py = 0.05 * h; // 5% padding
                db->chipRegion.ll.x = (float)(minx - px);
                db->chipRegion.ll.y = (float)(miny - py);
                db->chipRegion.ur.x = (float)(maxx + px);
                db->chipRegion.ur.y = (float)(maxy + py);
            } else {
                db->chipRegion.ll = {0, 0};
                db->chipRegion.ur = {1000, 1000};
            }
        }
    }

    // 2) 计算画布参数与缩放
    const double chip_ll_x = (double)db->chipRegion.ll.x;
    const double chip_ll_y = (double)db->chipRegion.ll.y;
    const double chip_ur_x = (double)db->chipRegion.ur.x;
    const double chip_ur_y = (double)db->chipRegion.ur.y;

    double chipRegionWidth  = std::max(1e-9, chip_ur_x - chip_ll_x);
    double chipRegionHeight = std::max(1e-9, chip_ur_y - chip_ll_y);

    const int imageLength = 1200; // 参考宽度
    const int xMargin = 20, yMargin = 20;
    const int imageWidth  = imageLength;
    int imageHeight = (int)std::ceil(chipRegionHeight * (imageLength / chipRegionWidth));
    imageHeight = std::max(imageHeight, 1);

    CImg<unsigned char> img(
        imageWidth + 2 * xMargin,
        imageHeight + 2 * yMargin,
        1, 3, 255
    );

    const double unitX = (double)imageWidth  / chipRegionWidth;
    const double unitY = (double)imageHeight / chipRegionHeight;

    // 颜色
    const unsigned char Red[3]    = {255,   0,   0};   // STD
    const unsigned char Green[3]  = {  0, 200,   0};   // MACRO
    const unsigned char Blue[3]   = {  0,   0, 255};   // FIXED
    const unsigned char Orange[3] = {255, 128,   0};   // FILLER
    const unsigned char Gray[3]   = {180, 180, 180};   // 边框
    const unsigned char Black[3]  = {  0,   0,   0};

    const float opacity_fill = 0.7f;

    std::cout << "Plot canvas: " << img.width() << "x" << img.height()
              << "  unitX=" << unitX << "  unitY=" << unitY
              << "  chipBox=(" << chip_ll_x << "," << chip_ll_y
              << ")~(" << chip_ur_x << "," << chip_ur_y << ")\n";

    // 3) 芯片边框
    const int frame_l = xMargin;
    const int frame_t = yMargin;
    const int frame_r = xMargin + imageWidth;
    const int frame_b = yMargin + imageHeight;
    img.draw_line(frame_l, frame_t, frame_r, frame_t, Gray);
    img.draw_line(frame_r, frame_t, frame_r, frame_b, Gray);
    img.draw_line(frame_r, frame_b, frame_l, frame_b, Gray);
    img.draw_line(frame_l, frame_b, frame_l, frame_t, Gray);

    // 4) 绘制模块（使用 center/width/height；最小像素保护；可选跳过 FILLER）
    for (size_t k = 0; k < db->Nodes.size(); ++k) {
        Module* curNode = db->Nodes[k];
        if (!curNode) continue;
        if (curNode->isFiller && !kDrawFillers) continue; // 可切换

        // 用当前 center/width/height 计算芯片坐标盒子
        const double llx = (double)curNode->center.x - (double)curNode->width  * 0.5;
        const double lly = (double)curNode->center.y - (double)curNode->height * 0.5;
        const double urx = (double)curNode->center.x + (double)curNode->width  * 0.5;
        const double ury = (double)curNode->center.y + (double)curNode->height * 0.5;

        // 相对 chip LL
        const double ll_rel_x = llx - chip_ll_x;
        const double ll_rel_y = lly - chip_ll_y;
        const double ur_rel_x = urx - chip_ll_x;
        const double ur_rel_y = ury - chip_ll_y;

        // 映射到像素（Y 轴镜像）
        int x_left   = (int)std::llround(ll_rel_x * unitX) + xMargin;
        int x_right  = (int)std::llround(ur_rel_x * unitX) + xMargin;
        int y_top    = (int)std::llround((chipRegionHeight - ur_rel_y) * unitY) + yMargin;
        int y_bottom = (int)std::llround((chipRegionHeight - ll_rel_y) * unitY) + yMargin;

        ensure_min_pixel_rect(x_left, y_top, x_right, y_bottom);

        const unsigned char* color_fill =
            curNode->isMacro  ? Green :
            curNode->isFixed  ? Blue  :
            curNode->isFiller ? Orange:
                                Red;

        img.draw_rectangle(x_left, y_top, x_right, y_bottom, color_fill, opacity_fill);
        img.draw_rectangle(x_left, y_top, x_right, y_bottom, Black, 0.0f); // 细黑边

        const int px_w = x_right - x_left;
        const int px_h = y_bottom - y_top;

        std::cout
            << "---- Module #" << k << " : " << curNode->name << " ----\n"
            << "Type    : " << (curNode->isMacro ? "MACRO" : "STD")
            << (curNode->isFixed ? " | FIXED" : "")
            << (curNode->isTerminal ? " | TERMINAL" : "")
            << (curNode->isFiller ? " | FILLER" : "") << "\n"
            << "ChipBox : LL(" << llx << "," << lly << ")"
            << "  UR(" << urx << "," << ury << ")"
            << "  W/H(" << curNode->width << "," << curNode->height << ")\n"
            << "Center  : (" << curNode->center.x << "," << curNode->center.y << ")\n"
            << "Image   : left=" << x_left << ", top=" << y_top
            << "  right=" << x_right << ", bottom=" << y_bottom
            << "  px(W,H)=(" << px_w << "," << px_h << ")\n";
    }

    // 5) 标题与保存
    img.draw_text(30, 20, imageName.c_str(), Black, NULL, 1, 24);
    const std::string outPath = imageName + ".bmp";
    img.save_bmp(outPath.c_str());
    cout << "bitmap file has been saved: " << outPath << endl;
}



// void PLOTTING::plotPlacement(string imageName, PlaceData *db)
// {
//     // 输出目录（生成的位图将保存在 ./output/ 下）
//     string plotPath = "./output/";

//     // === 1️⃣ 计算芯片区域大小 ===
//     float chipRegionWidth  = db->chipRegion.ur.x - db->chipRegion.ll.x;
//     float chipRegionHeight = db->chipRegion.ur.y - db->chipRegion.ll.y;

//     // === 2️⃣ 定义图片尺寸参数 ===
//     int ImageLength = 500;   // 图像的参考宽度（单位像素）
//     int imageHeight;
//     int imageWidth;

//     float opacity = 0.7;     // 绘制矩形时的不透明度（0~1）
//     int xMargin = 15, yMargin = 15;   // 图像边界留白

//     // === 3️⃣ 根据芯片长宽比计算图像高宽 ===
//     imageHeight = 1.0 * chipRegionHeight / (chipRegionWidth / ImageLength);
//     imageWidth  = ImageLength;

//     // === 4️⃣ 创建一张空白图像 ===
//     // 参数：(宽, 高, 深度=1, 通道数=3, 初始颜色=255白色)
//     CImg<unsigned char> img(
//         imageWidth + 2 * xMargin,
//         imageHeight + 2 * yMargin,
//         1, 3, 255
//     );

//     // === 5️⃣ 坐标缩放比例 ===
//     // 实际芯片坐标到图像像素的比例换算
//     float unitX = imageWidth  / chipRegionWidth;
//     float unitY = imageHeight / chipRegionHeight;

//     // === 6️⃣ 遍历每一个模块（Module）并绘制矩形 ===
//     for (Module *curNode : db->Nodes)
//     {
//         // 将模块左下角 / 右上角坐标从芯片坐标系映射到图像坐标系
//         int x1 = (curNode->getLL_2D().x - db->chipRegion.ll.x) * unitX + xMargin;
//         int x2 = (curNode->getUR_2D().x - db->chipRegion.ll.x) * unitX + xMargin;

//         // Y轴方向需要“镜像”处理，因为图像坐标原点在左上角，而芯片坐标在左下角
//         int y1 = (chipRegionHeight - (curNode->getLL_2D().y - db->chipRegion.ll.y)) * unitY + yMargin;
//         int y2 = (chipRegionHeight - (curNode->getUR_2D().y - db->chipRegion.ll.y)) * unitY + yMargin;

//         // === 判断模块类型并上色 ===
//         // 宏块 (macro) → 绿色
//         // 其他单元（标准单元） → 红色
//         if (curNode->isMacro)
//         {
//             img.draw_rectangle(x1, y1, x2, y2, Green, opacity);
//         }
//         else
//         {
//             img.draw_rectangle(x1, y1, x2, y2, Red, opacity);
//             std::cout << "Plotting module: " << curNode->name
//             << " at (" << x1 << "," << y1 << ") to (" << x2 << "," << y2 << ")\n";
//         }
//     }

//     // === 7️⃣ 绘制图像标题文字 ===
//     img.draw_text(50, 50, imageName.c_str(), Black, NULL, 1, 30);

//     // === 8️⃣ 保存为 BMP 文件 ===
//     img.save_bmp(string(imageName + ".bmp").c_str());
//     cout << "bitmap file has been saved: " << imageName + ".bmp" << endl;
// }





// void PLOTTING::plotPlacement(std::string imageName, PlaceData* db)
// {
//     using std::cout; using std::endl;

//     // ---------------------------
//     // 0) 安全检查
//     // ---------------------------
//     if (!db) { std::cerr << "[ERR] db is null.\n"; return; }
//     if (db->Nodes.empty()) { std::cerr << "[WARN] no modules to plot.\n"; }

//     // ---------------------------
//     // 1) 以模块几何为基准，修正/扩展 chipRegion（5% 留白）
//     // ---------------------------
//     auto expandChipRegionFromModules = [&](PlaceData* db) {
//         double minx =  std::numeric_limits<double>::infinity();
//         double miny =  std::numeric_limits<double>::infinity();
//         double maxx = -std::numeric_limits<double>::infinity();
//         double maxy = -std::numeric_limits<double>::infinity();

//         for (Module* m : db->Nodes) {
//             if (!m) continue;
//             POS_2D ll = m->getLL_2D();
//             POS_2D ur = m->getUR_2D();
//             minx = std::min(minx, (double)ll.x);
//             miny = std::min(miny, (double)ll.y);
//             maxx = std::max(maxx, (double)ur.x);
//             maxy = std::max(maxy, (double)ur.y);
//         }
//         if (!std::isfinite(minx) || !std::isfinite(miny) ||
//             !std::isfinite(maxx) || !std::isfinite(maxy)) {
//             // 兜底：若没有有效模块，保持原 region
//             return;
//         }
//         // 若原 chipRegion 太小或未覆盖，按模块盒子扩展
//         double w0 = db->chipRegion.ur.x - db->chipRegion.ll.x;
//         double h0 = db->chipRegion.ur.y - db->chipRegion.ll.y;

//         double minx_all = std::min((double)db->chipRegion.ll.x, minx);
//         double miny_all = std::min((double)db->chipRegion.ll.y, miny);
//         double maxx_all = std::max((double)db->chipRegion.ur.x, maxx);
//         double maxy_all = std::max((double)db->chipRegion.ur.y, maxy);

//         double w = std::max(1.0, maxx_all - minx_all);
//         double h = std::max(1.0, maxy_all - miny_all);
//         double pad = 0.05; // 5% padding
//         double px = w * pad, py = h * pad;

//         db->chipRegion.ll.x = minx_all - px;
//         db->chipRegion.ll.y = miny_all - py;
//         db->chipRegion.ur.x = maxx_all + px;
//         db->chipRegion.ur.y = maxy_all + py;
//     };

//     expandChipRegionFromModules(db);

//     // ---------------------------
//     // 2) 计算芯片区域与图像大小
//     // ---------------------------
//     const double chip_ll_x = db->chipRegion.ll.x;
//     const double chip_ll_y = db->chipRegion.ll.y;
//     const double chip_ur_x = db->chipRegion.ur.x;
//     const double chip_ur_y = db->chipRegion.ur.y;

//     double chipRegionWidth  = std::max(1e-9, (double)chip_ur_x - chip_ll_x);
//     double chipRegionHeight = std::max(1e-9, (double)chip_ur_y - chip_ll_y);

//     int imageLength = 1000; // 画布参考宽度，可按需调大让更清晰
//     int xMargin = 20, yMargin = 20;

//     // 保持长宽比：imageWidth 固定为 imageLength，imageHeight 按比例计算
//     int imageWidth  = imageLength;
//     int imageHeight = (int)std::ceil(chipRegionHeight * (imageLength / chipRegionWidth));
//     imageHeight = std::max(imageHeight, 1); // 防 0

//     // ---------------------------
//     // 3) 新建画布（含边距）
//     // ---------------------------
//     CImg<unsigned char> img(
//         imageWidth + 2 * xMargin,
//         imageHeight + 2 * yMargin,
//         1, 3, 255
//     );

//     // 坐标缩放：芯片坐标 -> 像素
//     const double unitX = (double)imageWidth  / chipRegionWidth;
//     const double unitY = (double)imageHeight / chipRegionHeight;

//     // 颜色（CImg 使用 0..255 三通道）
//     const unsigned char Red[3]   = {255,   0,   0};
//     const unsigned char Green[3] = {  0, 200,   0};
//     const unsigned char Black[3] = {  0,   0,   0};

//     float opacity = 0.7f;

//     // ---------------------------
//     // 4) 遍历模块并绘制（含详尽调试）
//     // ---------------------------
//     cout << "Plot canvas: " << img.width() << "x" << img.height()
//          << "  unitX=" << unitX << "  unitY=" << unitY
//          << "  chipBox=(" << chip_ll_x << "," << chip_ll_y
//          << ")~(" << chip_ur_x << "," << chip_ur_y << ")\n";

//     for (size_t k = 0; k < db->Nodes.size(); ++k) {
//         Module* curNode = db->Nodes[k];
//         if (!curNode) continue;

//         POS_2D ll_chip = curNode->getLL_2D();  // 芯片坐标 LL
//         POS_2D ur_chip = curNode->getUR_2D();  // 芯片坐标 UR
//         POS_2D cc_chip = curNode->center;      // 中心（如有维护）

//         // 相对 chip LL 的坐标
//         double ll_rel_x = ll_chip.x - chip_ll_x;
//         double ll_rel_y = ll_chip.y - chip_ll_y;
//         double ur_rel_x = ur_chip.x - chip_ll_x;
//         double ur_rel_y = ur_chip.y - chip_ll_y;

//         // 映射到图像坐标系
//         int x_left  = (int)std::llround(ll_rel_x * unitX) + xMargin;
//         int x_right = (int)std::llround(ur_rel_x * unitX) + xMargin;

//         // **Y 轴镜像关键**：UR -> top, LL -> bottom
//         int y_top    = (int)std::llround((chipRegionHeight - ur_rel_y) * unitY) + yMargin;
//         int y_bottom = (int)std::llround((chipRegionHeight - ll_rel_y) * unitY) + yMargin;

//         // 兜底确保 left<right, top<bottom
//         if (x_left > x_right)   std::swap(x_left, x_right);
//         if (y_top  > y_bottom)  std::swap(y_top, y_bottom);

//         int px_w = x_right - x_left;
//         int px_h = y_bottom - y_top;

//         bool isMacro  = curNode->isMacro;
//         bool isFixed  = curNode->isFixed;
//         bool isTerm   = curNode->isTerminal;
//         bool isFiller = curNode->isFiller;

//         // 边界检查：要对比实际画布尺寸（含边距），即 img.width()/img.height()
//         bool out = (x_left  < 0 || y_top < 0 ||
//                     x_right > img.width() ||
//                     y_bottom > img.height());

//         std::cout
//             << "---- Module #" << k << " : " << curNode->name << " ----\n"
//             << "Type    : " << (isMacro ? "MACRO" : "STD")
//             << (isFixed ? " | FIXED" : "")
//             << (isTerm  ? " | TERMINAL" : "")
//             << (isFiller? " | FILLER" : "") << "\n"
//             << "Orient  : " << curNode->orientation << "\n"
//             << "ChipBox : LL(" << ll_chip.x << "," << ll_chip.y << ")"
//             << "  UR(" << ur_chip.x << "," << ur_chip.y << ")"
//             << "  W/H(" << curNode->width << "," << curNode->height << ")\n"
//             << "Center  : (" << cc_chip.x << "," << cc_chip.y << ")\n"
//             << "Rel-LL  : (" << ll_rel_x << "," << ll_rel_y << ")  (to chip LL)\n"
//             << "Image   : left=" << x_left << ", top=" << y_top
//             << "  right=" << x_right << ", bottom=" << y_bottom
//             << "  px(W,H)=(" << px_w << "," << px_h << ")  "
//             << (out ? "[WARN OUT-OF-BOUNDS]" : "[OK]") << "\n";

//         // 实际绘制
//         img.draw_rectangle(
//             x_left, y_top, x_right, y_bottom,
//             (isMacro ? Green : Red),
//             opacity
//         );
//     }

//     // ---------------------------
//     // 5) 标题与输出
//     // ---------------------------
//     img.draw_text(30, 20, imageName.c_str(), Black, NULL, 1, 24);
//     std::string outPath = imageName + ".bmp";
//     img.save_bmp(outPath.c_str());
//     cout << "bitmap file has been saved: " << outPath << endl;
// }