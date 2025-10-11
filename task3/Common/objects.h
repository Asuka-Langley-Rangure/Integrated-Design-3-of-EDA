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
    string orientation;
    bool isMacro;
    bool isFixed;
    bool isFiller;
    vector<Pin *> modulePins;
    vector<Net *> nets;

    bool isTerminal;
    void Init()
    {
        idx = -1;
        center.SetZero();
        width = 0;
        height = 0;
        area = 0;
        orientation = "none";
        isMacro = false;
        isFiller = false;
        isFixed = false;
        isTerminal = false;
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
        direction = "none";
        offset.SetZero();

    }
    int idx;
    Module *module;
    Net *net;
    string direction;
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

#endif