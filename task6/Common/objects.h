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
    Module(int _index, string _name, float _width = 0, float _height = 0)
    {
        Init();
        name = _name;
        width = _width;
        height = _height;
        area = width*height; 
        idx = _index;
    }
    int idx;
    string name;
    float width;
    float height;
    float area;
    string orientation;

    bool isMacro;
    bool isFixed;
    bool isFiller;
    
    bool isTerminal;

    vector<Pin *> modulePins;
    vector<Net *> nets;

    void Init()
    {
        idx = -1;
        center.SetZero();
        coor.x = 500;
        coor.y = 500;
        width = 0;
        height = 0;
        area = 0;
        orientation = "None";
        isMacro = false;
        isFiller = false;
        isFixed = false;
     };

    POS_2D getLL_2D()
    {
        return coor;
    };

    POS_2D getUR_2D()
    {
        POS_2D ur_2D;

        ur_2D.x = coor.x;
        ur_2D.y = coor.y;

        ur_2D.x += width;
        ur_2D.y += height;
        return ur_2D;
    };

    POS_2D center; 
    POS_2D coor; 
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
        direction = "None";
        offset.SetZero();

    }
    int idx;
    string direction;
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
    string name;
    vector<Pin *> netPins;

    void init()
    {
        idx = -1;
        name = "none";
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