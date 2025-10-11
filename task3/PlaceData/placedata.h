#ifndef PLACEDATA_H
#define PLACEDATA_H
#include "../Common/objects.h"

class PlaceData
{
public:
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

    map<string, Module *> moduleMap; 
};
#endif