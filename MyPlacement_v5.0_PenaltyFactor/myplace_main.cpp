#include "myplace.h"
#include "placedata.h"
#include "plot.h"
#include "myplace.h"
#include "NesterovOpt.h"

#include <iostream>
using namespace std;

int ReadNodesFile(string file, PlaceData &db)
{
    int Nodenum=1;    
    int index=0; 
    db.Nodes.resize(Nodenum);
    Module *node = new Module(index, "o1", 8, 12);
    db.Nodes[index] = node; 
    return 0;
}

int main(int argc, char *argv[])
{
    PlaceData *placedata = new PlaceData();
    ReadNodesFile("adaptec1.nodes",*placedata);
    MyPlacer *myplacer = new MyPlacer(placedata);
       
    myplacer->setTargetDensity(0.9);
    myplacer->Init();
    NesterovOpt* nesterov = new NesterovOpt(myplacer);
    nesterov->NAG_Process();
    PLOTTING::plotPlacement("gp_result", placedata);
    return 0;
}

