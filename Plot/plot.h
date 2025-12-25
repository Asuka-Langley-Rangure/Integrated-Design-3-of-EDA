#ifndef PLOT_H
#define PLOT_H
#include "common.h"

#define cimg_display 0
#include "CImg.h"

#include "placedata.h"

// 前向声明，避免循环依赖
class MyPlacer;

using namespace cimg_library;
namespace PLOTTING
{
    const unsigned char Black[] = {0, 0, 0},
                        Green[] = {0, 150, 0},
                        Red[] = {255, 0, 0};
    void plotPlacement(string, PlaceData *, MyPlacer * = nullptr);
}

#endif