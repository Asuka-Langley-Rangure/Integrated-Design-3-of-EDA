#include "MyPlace/myplace.h"
#include "PlaceData/placedata.h"
#include "Plot/plot.h"
#include "Optimization/NesterovOpt.h"

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
    ReadNodesFile("./data/adaptec1/adaptec1.nodes",*placedata);
    MyPlacer *myplacer = new MyPlacer(placedata);
       
    myplacer->setTargetDensity(0.9);
    myplacer->Init();
    NesterovOpt* nesterov = new NesterovOpt(myplacer);
    nesterov->NAG_Process();
    PLOTTING::plotPlacement("gp_result", placedata);
    return 0;
}

