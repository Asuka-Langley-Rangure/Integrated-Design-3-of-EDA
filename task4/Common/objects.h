#ifndef OBJECTS_H
#define OBJECTS_H
#include "common.h"

class Module;
class SiteRow;
class Pin;
class Net;
class PlaceData;

class Module
{
public:
    Module()
    {
        Init();
    }
    int idx;
    string name;
    float width;
    float height;
    float area;
    float orientation;
    bool isMacro;
    bool isFixed;
    bool isFiller;
    vector<Pin *> modulePins;
    vector<Net *> nets;
    void Init()
    {
        idx = -1;
        center.SetZero();
        width = 0;
        height = 0;
        area = 0;
        orientation = 0;
        isMacro = false;
        isFiller = false;
        isFixed = false;
     }
    POS_2D center; 
 
};
class SiteRow 
{
public:
    SiteRow()
    {
        bottom = 0;
        height = 0;
        step = 0;
        start.SetZero();
        end.SetZero();
    }

    double bottom;             
    double height;             
    double step;                
    POS_2D start;               
    POS_2D end;                 
};

class Pin
{
public:
    Pin()
    {
        init();
    }

    void init()
    {
        idx = -1;
        module = NULL;
        net = NULL;
        offset.SetZero();

    }
    int idx;
    Module *module;
    Net *net;
    POS_2D offset;
};

class Net
{
public:
    Net()
    {
        init();
    }
    int idx;
    vector<Pin *> netPins;

    void init()
    {
        idx = 0;
        netPins.clear();
    }
};

class Bin {
public:
    POS_2D ll;          // left-bottom
    POS_2D ur;          // right-top
    POS_2D center;
    double width;
    double height;
    double terminalDensity;  // 固定宏贡献的密度
    double darkDensity;      // 超出 SiteRow 的“暗区”密度

    Bin() {
        terminalDensity = 0.0;
        darkDensity = 0.0;
    }
};

#endif