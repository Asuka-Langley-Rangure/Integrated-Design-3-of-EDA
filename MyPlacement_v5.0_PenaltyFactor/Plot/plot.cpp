#include "plot.h"

using namespace PLOTTING;

void PLOTTING::plotPlacement(string imageName, PlaceData *db)
{
    string plotPath="./output/";

    float chipRegionWidth = db->chipRegion.ur.x - db->chipRegion.ll.x;
    float chipRegionHeight = db->chipRegion.ur.y - db->chipRegion.ll.y;

    int ImgaeLength = 500;
    int imageHeight;
    int imageWidth;

    float opacity = 0.7;
    int xMargin = 15, yMargin = 15;

    imageHeight = 1.0 * chipRegionHeight / (chipRegionWidth / ImgaeLength);
    imageWidth = ImgaeLength;

    CImg<unsigned char> img(imageWidth + 2 * xMargin, imageHeight + 2 * yMargin, 1, 3, 255);//deep=1,color,white

    float unitX = imageWidth / chipRegionWidth,
          unitY = imageHeight / chipRegionHeight;

    for (Module *curNode : db->Nodes)
    {
        int x1 = (curNode->getLL_2D().x -db->chipRegion.ll.x) *unitX + xMargin;
        int x2 = (curNode->getUR_2D().x -db->chipRegion.ll.x) *unitX + xMargin;

        int y1 = (chipRegionHeight-(curNode->getLL_2D().y -db->chipRegion.ll.y) ) * unitY + yMargin; //mirror Y
        int y2 = (chipRegionHeight-(curNode->getUR_2D().y -db->chipRegion.ll.y) ) * unitY + yMargin; //mirror Y
        if (curNode->isMacro)
        {
            img.draw_rectangle(x1, y1, x2, y2, Green, opacity);
        }
        else
        {
            img.draw_rectangle(x1, y1, x2, y2, Red, opacity);
        }
    }

    img.draw_text(50, 50, imageName.c_str(), Black, NULL, 1, 30);
    img.save_bmp(string(plotPath + imageName + string(".bmp")).c_str());
    cout << "bitmap file has been saved: " << imageName + string(".bmp") << endl;
}

