#pragma once
#ifndef MYPLACE_H
#define MYPLACE_H

#include "placedata.h"
#include "common.h"
#include "../eigen3/Eigen/Sparse"
#include "../eigen3/Eigen/IterativeLinearSolvers"  // BiCGSTAB/CG 等迭代法
#include <random>

#include <filesystem>
#include <iostream>
#include <fstream>  // 用于文件操作
namespace fs = std::filesystem;

// 迭代求解器参数（可选）
struct SolverParams {
    int    max_iters = 2000;    // BiCGSTAB 最大迭代步
    double tol       = 1e-8;    // 收敛容差（残差范数相对量）
    bool   verbose   = true;    // 输出每个维度的迭代结果
};


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
    float DarkDensity;  
    float phi;   
};

class MyPlacer
{
public:
     MyPlacer(PlaceData *_db)
    {
        db = _db;
    }
    vector<vector<Bin_2D *>> bins;

    float StdCellArea; 
    float MacroArea;
    
        
    PlaceData *db;

    void initialPlacement();

};

#endif