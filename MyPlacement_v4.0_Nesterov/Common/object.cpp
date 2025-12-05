#include "objects.h"

POS_2D Module::getLL_2D()
{
        POS_2D ll_2D;
        ll_2D.x = coor.x;
        ll_2D.y = coor.y;
        return ll_2D;
}

POS_2D Module::getUR_2D()
{
        POS_2D ur_2D;

        ur_2D.x = coor.x;
        ur_2D.y = coor.y;

        ur_2D.x += width;
        ur_2D.y += height;

        return ur_2D;
}
    
void Module::setCenter_2D(float _x, float _y)
{
    center.x = _x;
    center.y = _y;

    coor.x = center.x - (float)0.5 * width; 
    coor.y = center.y - (float)0.5 * height;

}

void Pin::calculateAbsolutePos()
{
    absolutePos.x = module->getCenter().x + offset.x;
    absolutePos.y = module->getCenter().y + offset.y;
    
}