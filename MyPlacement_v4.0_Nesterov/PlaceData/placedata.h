#ifndef PLACEDATA_H
#define PLACEDATA_H
#include "objects.h"

class PlaceData
{
public:
    PlaceData()
    {
        coreRegion = CRect();
        coreRegion.ll = POS_2D(459,459);
        coreRegion.ur = POS_2D(11151,11139);
        chipRegion = CRect();
        chipRegion.ll = POS_2D(22,22);
        chipRegion.ur = POS_2D(11589,11589);
        totalRowArea=114190560;
    }
    int moduleCount; 
    int MacroCount;
    int netCount;
    int pinCount;

    double commonRowHeight;
    CRect coreRegion;
    CRect chipRegion;
    float totalRowArea;

    vector<Module *> Nodes; 
    vector<Module *> Terminals;
    vector<Pin *> Pins;
    vector<Net *> Nets;
    vector<SiteRow> SiteRows;
    
    map<string, Module *> moduleMap; 
    void setModuleCenter_2D(Module *, float, float);
    void setModuleCenter_2D(Module *, VECTOR_3D);
    void setModuleLocation_2D_random(Module *module);
    void setModuleLocation_2D(Module *, float, float);
};

#endif