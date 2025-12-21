#include "placedata.h"

void Module::setLocation_2D(float _x, float _y)
{
    coor.x = _x;
    coor.y = _y;

    // update center
    center.x = coor.x + (float)0.5 * width; //! be careful of float problems
    center.y = coor.y + (float)0.5 * height;

}

void PlaceData::setModuleCenter_2D(Module *module, float x, float y)
{
    //? use high precision comparison functions in global.h??
    //  check if x,y are legal(inside the chip)
    if (!module->isFixed)
    {
        if (x - 0.5 * module->width < coreRegion.ll.x)
        {
            // x = coreRegion.ll.x + 0.5 * module->width;
            x = coreRegion.ll.x + 0.5 * module->width + EPS;
        }
        if (x + 0.5 * module->width > coreRegion.ur.x)
        {
            // x = coreRegion.ur.x - 0.5 * module->width;
            x = coreRegion.ur.x - 0.5 * module->width - EPS;
        }
        if (y - 0.5 * module->height < coreRegion.ll.y)
        {
            // y = coreRegion.ll.y + 0.5 * module->height;
            y = coreRegion.ll.y + 0.5 * module->height + EPS;
        }
        if (y + 0.5 * module->height > coreRegion.ur.y)
        {
            // y = coreRegion.ur.y - 0.5 * module->height;
            y = coreRegion.ur.y - 0.5 * module->height - EPS;
        }
    }

    module->setCenter_2D(x, y);
    for (Pin *curPin : module->modulePins)
    {
        curPin->calculateAbsolutePos();
    }
}

void PlaceData::setModuleLocation_2D(Module *module, float x, float y)
{
    //? use high precision comparison functions in global.h??
    //  check if x,y are legal(inside the chip)
    if (!module->isFixed)
    {
        if (x < coreRegion.ll.x)
        {
            x = coreRegion.ll.x;
        }
        if (x + module->width > coreRegion.ur.x)
        {
            x = coreRegion.ur.x - module->width - EPS;
        }
        if (y < coreRegion.ll.y)
        {
            y = coreRegion.ll.y;
        }
        if (y + module->height > coreRegion.ur.y)
        {
            y = coreRegion.ur.y - module->height - EPS;
        }
    }

    module->setLocation_2D(x, y);
    for (Pin *curPin : module->modulePins)
    {
        curPin->calculateAbsolutePos();
    }
}

void PlaceData::setModuleLocation_2D_random(Module *module)
{
    assert(module);
    float x = rand();
    float y = rand();

    assert(coreRegion.ur.x > coreRegion.ll.x);
    assert(coreRegion.ur.y > coreRegion.ll.y);

    float potentialRegionWidth = coreRegion.ur.x - coreRegion.ll.x; // potential region for randomly place module
    float potentialRegionHeight = coreRegion.ur.y - coreRegion.ll.y;

    potentialRegionHeight -= module->getHeight();
    potentialRegionWidth -= module->getWidth();

    float RAND_MAX_INVERSE = (float)1.0 / RAND_MAX;
    x = x * RAND_MAX_INVERSE * potentialRegionWidth;
    y = y * RAND_MAX_INVERSE * potentialRegionHeight;

    setModuleLocation_2D(module, x, y);
}


void PlaceData::setModuleCenter_2D(Module *module, VECTOR_3D pos)
{
    setModuleCenter_2D(module, pos.x, pos.y);
}

double PlaceData::calcHPWL() 
{
    double HPWL = 0;
    for (Net *curNet : Nets)
    {
        HPWL += curNet->calcNetHPWL();
    }
    return HPWL;
}
