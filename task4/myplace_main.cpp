#include "myplace.h"
#include "placedata.h"
//#include "plot.h"
#include <iostream>
using namespace std;

int main(int argc, char *argv[])
{
    PlaceData *placedata = new PlaceData();
    MyPlacer *myplacer = new MyPlacer(placedata);

    myplacer->createfillerCells();
    myplacer->initializeBins(0.8);
    myplacer->initialPlacement();
}
