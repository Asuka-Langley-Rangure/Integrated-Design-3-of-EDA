#include "objects.h"

POS_2D Module::getLL_2D() const
{
        POS_2D ll_2D;
        ll_2D.x = coor.x;
        ll_2D.y = coor.y;
        return ll_2D;
}

POS_2D Module::getUR_2D() const
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

double Net::calcNetHPWL()
{
    double maxX = -DOUBLE_MAX;
    double minX = DOUBLE_MAX;
    double maxY = -DOUBLE_MAX;
    double minY = DOUBLE_MAX;
    double maxZ = -DOUBLE_MAX; // potential bug: double_min >0 so boundPinZmax might be null when all z == 0
    double minZ = DOUBLE_MAX;

    double curX;
    double curY;
    double curZ;
    POS_3D curPos;
    double HPWL;
    for (Pin *curPin : netPins)
    {
        curPos.x = curPin->absolutePos.x;
        curPos.y = curPin->absolutePos.y;
        curPos.z = 0.0; // no z info for pins; assume flat plane
        curX = curPos.x;
        curY = curPos.y;
        curZ = curPos.z;
        minX = min(minX, curX);
        maxX = max(maxX, curX);
        minY = min(minY, curY);
        maxY = max(maxY, curY);
        minZ = min(minZ, curZ);
        maxZ = max(maxZ, curZ);
    }

    HPWL = ((maxX - minX) + (maxY - minY) + (maxZ - minZ));
    return HPWL;
}
