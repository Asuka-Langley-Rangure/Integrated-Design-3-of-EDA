#ifndef PLOT_H
#define PLOT_H
#include "common.h"

#define cimg_display 0
#include "../Library/CImg/CImg.h"
#include "../algorithm/db.h"

using namespace cimg_library;
namespace PLOTTING
{
    const unsigned char Black[] = {0, 0, 0},
                        Green[] = {0, 150, 0},
                        Red[] = {255, 0, 0};
    void plotPlacement(const std::string& imageName, const PlacementDB* db);

}

#endif