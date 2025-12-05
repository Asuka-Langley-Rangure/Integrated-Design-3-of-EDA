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
    Module(int _index, string _name, float _width = 0, float _height = 0, bool _isFixed = false, bool _isNI = false)
    {
        Init();
        name = _name;
        width = _width;
        height = _height;
        area = width*height; //! area calculated here!
        isFixed = _isFixed;
        idx = _index;
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
    POS_3D center; 
    POS_3D coor; 
    void Init()
    {
        idx = -1;
        center.SetZero();
        coor.x = 500;
        coor.y = 500;
        width = 0;
        height = 0;
        area = 0;
        orientation = 0;
        isMacro = false;
        isFiller = false;
        isFixed = false;
     };
    POS_3D getCenter() { return center; }
    float getWidth() { return width; }
    float getHeight() { return height; }
    float getArea() { return area; }
    POS_2D getLL_2D();
    POS_2D getUR_2D();
    void setLocation_2D(float, float);
    void setCenter_2D(float, float);
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


    POS_2D getLL_2D()
    {
        return start;
    }

    POS_2D getUR_2D()
    {
        POS_2D ur_2D = end;
        ur_2D.y += height;
        return ur_2D;
    }

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
    POS_2D absolutePos;
    void calculateAbsolutePos();
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

#endif