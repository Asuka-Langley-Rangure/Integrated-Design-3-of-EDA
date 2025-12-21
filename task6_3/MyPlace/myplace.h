#ifndef MYPLACE_H
#define MYPLACE_H

#include "placedata.h"
#include "common.h"
#include "plot.h"
#include "fft.h"

#include "../eigen3/Eigen/Sparse"
#include "../eigen3/Eigen/IterativeLinearSolvers"  // BiCGSTAB/CG 等迭代法
#include <random>

#include <filesystem>
#include <iostream>
#include <fstream> 

class Bin_2D;
class MyPlacer;
class Bin_2D
{
public:
    POS_2D center;
    POS_2D ll;
    POS_2D ur;
    float width;
    float height;
    float area;

    float nodeDensity;    
    float fillerDensity;   
    float terminalDensity; 
    float darkDensity;  
    VECTOR_2D E;
    float phi;   
};

class MyPlacer
{
public:
     MyPlacer(PlaceData *_db)
    {
        db = _db;
        lambda = 1.0f;            // default density weight
        targetDensity = 0.8f;     // typical GP target
        globalDensityOverflow = 0.0f;
    }
    vector<vector<Bin_2D *>> bins;

    VECTOR_2D_INT binDimension; // How many bins in X/Y direction
    VECTOR_2D binStep;          // length of a bin in X/Y direction

    float width;
    float height;

    float StdCellArea; 
    float MacroArea;
        
    PlaceData *db;

    void initialPlacement();

    vector<Module *> Fillers;  
    vector<Module *> NodesAndFillers;
    vector<Module *> CellsAndFillers; 
    vector<VECTOR_3D> wirelengthGradient; 
    vector<VECTOR_3D> densityGradient;   
    vector<VECTOR_3D> totalGradient;      
    vector<VECTOR_3D> fillerGradient;
    vector<VECTOR_3D> cGPGradient; 

    void Init();
    void FillerInit();
    void BinInit();
    void gradientVectorInitialization();
    float lambda; 
    float targetOverflow = 0.1;
    float targetDensity;         
    float globalDensityOverflow; 
    void GetWirelengthGradient();
    void GetBinDensity();
    void GetDensityGradient();
    void GetTotalGradient();
    void setTargetDensity(float target);
    vector<VECTOR_3D> getPosition();
    void setPosition(vector<VECTOR_3D>);
    vector<VECTOR_3D> getModulePositions(vector<Module *>);
};
#endif