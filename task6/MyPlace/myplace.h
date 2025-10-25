#pragma once
#ifndef MYPLACE_H
#define MYPLACE_H

#include "placedata.h"
#include "common.h"
#include "../eigen3/Eigen/Sparse"
#include <random>

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

    vector<VECTOR_3D> wirelengthGradient; 
    vector<VECTOR_3D> densityGradient;   
    vector<VECTOR_3D> totalGradient;      
    vector<VECTOR_3D> fillerGradient;
    void createfillerCells();
    void initialPlacement();
    std::pair<POS_2D, POS_2D> getSiteRowBoundingBox();
    void initializeBins(double targetDensity);
};

///////////////////////////////////////////////////////////////
// 进度条
//
///////////////////////////////////////////////////////////////

#include <chrono>
#include <iomanip>
#include <iostream>

// 轻量实时计数器（支持时间/步数节流；想要“每个 pin 刷新一次”，把 step_gate=1, ms_gate=0）
struct LiveCounter {
    size_t total = 1, done = 0, last_printed = 0;
    size_t step_gate = 1;                 // 步数门限：累计多少步后刷新一次（1 = 每步刷新）
    int ms_gate = 0;                      // 时间门限：两次刷新至少间隔多少毫秒（0 = 不限）
    std::chrono::steady_clock::time_point last = std::chrono::steady_clock::now();

    void init(size_t total_, size_t step_gate_=1, int ms_gate_=0) {
        total = total_ ? total_ : 1;
        step_gate = step_gate_;
        ms_gate = ms_gate_;
        done = last_printed = 0;
        last = std::chrono::steady_clock::now();
        print(true);
    }
    void tick(size_t inc = 1) {
        done += inc;
        auto now = std::chrono::steady_clock::now();
        bool by_step = (done - last_printed) >= step_gate;
        bool by_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count() >= ms_gate;
        if (done >= total || by_step || by_time) {
            last = now;
            print(false);
            last_printed = done;
        }
    }
    void finish() { done = total; print(true); }

    void print(bool force_newline) const {
        double pct = 100.0 * double(done) / double(total);
        std::cout << "\rAssembling pins: " << done << " / " << total
                  << " (" << std::fixed << std::setprecision(1) << pct << "%)";
        std::cout.flush();
        if (force_newline) std::cout << std::endl;
    }
};

#endif