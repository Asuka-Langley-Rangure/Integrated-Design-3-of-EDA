#include "plot.h"

using namespace PLOTTING;

void PLOTTING::plotPlacement(string imageName, PlaceData *db)
{
    // 输出目录（生成的位图将保存在 ./output/ 下）
    string plotPath = "./output/";

    // === 1️⃣ 计算芯片区域大小 ===
    float chipRegionWidth  = db->chipRegion.ur.x - db->chipRegion.ll.x;
    float chipRegionHeight = db->chipRegion.ur.y - db->chipRegion.ll.y;

    // === 2️⃣ 定义图片尺寸参数 ===
    int ImageLength = 500;   // 图像的参考宽度（单位像素）
    int imageHeight;
    int imageWidth;

    float opacity = 0.7;     // 绘制矩形时的不透明度（0~1）
    int xMargin = 15, yMargin = 15;   // 图像边界留白

    // === 3️⃣ 根据芯片长宽比计算图像高宽 ===
    imageHeight = 1.0 * chipRegionHeight / (chipRegionWidth / ImageLength);
    imageWidth  = ImageLength;

    // === 4️⃣ 创建一张空白图像 ===
    // 参数：(宽, 高, 深度=1, 通道数=3, 初始颜色=255白色)
    CImg<unsigned char> img(
        imageWidth + 2 * xMargin,
        imageHeight + 2 * yMargin,
        1, 3, 255
    );

    // === 5️⃣ 坐标缩放比例 ===
    // 实际芯片坐标到图像像素的比例换算
    float unitX = imageWidth  / chipRegionWidth;
    float unitY = imageHeight / chipRegionHeight;

    // === 6️⃣ 遍历每一个模块（Module）并绘制矩形 ===
    for (Module *curNode : db->Nodes)
    {
        // 将模块左下角 / 右上角坐标从芯片坐标系映射到图像坐标系
        int x1 = (curNode->getLL_2D().x - db->chipRegion.ll.x) * unitX + xMargin;
        int x2 = (curNode->getUR_2D().x - db->chipRegion.ll.x) * unitX + xMargin;

        // Y轴方向需要“镜像”处理，因为图像坐标原点在左上角，而芯片坐标在左下角
        int y1 = (chipRegionHeight - (curNode->getLL_2D().y - db->chipRegion.ll.y)) * unitY + yMargin;
        int y2 = (chipRegionHeight - (curNode->getUR_2D().y - db->chipRegion.ll.y)) * unitY + yMargin;

        // === 判断模块类型并上色 ===
        // 宏块 (macro) → 绿色
        // 其他单元（标准单元） → 红色
        if (curNode->isMacro)
        {
            img.draw_rectangle(x1, y1, x2, y2, Green, opacity);
            std::cout << "Plotting module: " << curNode->name
                      << " at (" << x1 << "," << y1 << ") to (" << x2 << "," << y2 << ")\n";
        }
        else
        {
            img.draw_rectangle(x1, y1, x2, y2, Red, opacity);
        }
    }

    // === 7️⃣ 绘制图像标题文字 ===
    img.draw_text(50, 50, imageName.c_str(), Black, NULL, 1, 30);

    // === 8️⃣ 保存为 BMP 文件 ===
    img.save_bmp(string(imageName + ".bmp").c_str());
    cout << "bitmap file has been saved: " << imageName + ".bmp" << endl;
}

