#ifndef PLACEDATA_H
#define PLACEDATA_H
#include "objects.h"

class PlaceData
{
public:
    PlaceData()
    {
        chipRegion = CRect();
        chipRegion.ll = POS_2D(100,100);
        chipRegion.ur = POS_2D(1300,1900);
    }

    CRect chipRegion;

    int moduleCount;
    int terminalCount;
    int macroCount;
    int netCount;
    int pinCount;

    vector<Module *> Nodes; 
    vector<Module *> Terminals;
    vector<Pin *> Pins;
    vector<Net *> Nets;
    vector<SiteRow> SiteRows;

    std::vector<std::vector<Bin>> bins;
    int binRows, binCols;  // 实际 = M x M
    float siteHeight;

    map<string, Module *> moduleMap; 
};

#endif