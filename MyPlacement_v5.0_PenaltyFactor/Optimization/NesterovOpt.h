#include <vector>
#include "common.h"
#include "myplace.h"

#define BKTRK_EPS 0.95
#define MAX_ITERATION 300

class NSIter
{
public:
    void resize(size_t length);
    std::vector<VECTOR_3D> main_solution;
    std::vector<VECTOR_3D> reference_solution;
    std::vector<VECTOR_3D> gradient;
};

class NesterovOpt
{
public:
    NesterovOpt(MyPlacer *placer) : placer(placer) {};
    void init();
    void NAG_Step();
    void NAG_Process();
    MyPlacer *placer;
    NSIter cur_iter, last_iter;
    size_t iter_count;
    float NS_opt_param; 
};