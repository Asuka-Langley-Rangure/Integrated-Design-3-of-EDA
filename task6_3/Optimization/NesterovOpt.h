#ifndef NESTEROV_OPT_H
#define NESTEROV_OPT_H

#include <vector>
#include <cstddef>
#include <cmath>
#include "common.h"
#include "myplace.h"

#define BKTRK_EPS      0.95f   // 回溯里 ε，略小于 1
#define MAX_ITERATION  300     // 最多迭代次数

class NSIter
{
public:
    void resize(std::size_t length);

    // u^k：主解
    std::vector<VECTOR_3D> main_solution;
    // v^k：参考解
    std::vector<VECTOR_3D> reference_solution;
    // ∇f_pre(v^k)：预条件梯度（注意：这里我们约定它已经是“下降方向”）
    std::vector<VECTOR_3D> gradient;
};

class NesterovOpt
{
public:
    explicit NesterovOpt(MyPlacer *placer)
        : placer(placer),
          iter_count(0),
          NS_opt_param(1.0f),
          step_alpha(1e-3f),
          lastHPWL(0.0)
    {}

    // 初始化（拉位置、算梯度、算初始 HPWL）
    void init();

    // 单步 NAG
    void NAG_Step();

    // 完整 NAG 求解流程
    void NAG_Process();

    // 和原框架保持同名的 public 成员
    MyPlacer *placer;
    NSIter    cur_iter;
    NSIter    last_iter;
    std::size_t iter_count;
    float    NS_opt_param;   // 对应 pdf 里的 β_k（实际上是 FISTA 里的 t_k）

private:
    float  step_alpha;       // 当前步长 α_k
    double lastHPWL;         // 上一次 HPWL(v_{k-1})

    // 把位置写回 db / MyPlacer
    void pushPositionToPlacer(const std::vector<VECTOR_3D> &pos);
    // 从 MyPlacer 取出总梯度（已经预条件、带 λ，且为下降方向）
    void pullGradientFromPlacer(std::vector<VECTOR_3D> &grad);

    // pdf (3-3-4) 的步长估计：α̂ = ||v_new - v_old|| / ||g_new - g_old||
    double computeStepLength(const std::vector<VECTOR_3D> &v_new,
                             const std::vector<VECTOR_3D> &v_old,
                             const std::vector<VECTOR_3D> &g_new,
                             const std::vector<VECTOR_3D> &g_old) const;

    // 按 pdf 3.3 的 HPWL（38）从 db 里算当前 HPWL
    double computeHPWLFromDB() const;

    // pdf (37) 的 λ 更新
    void updateLambda(double curHPWL);
};

#endif // NESTEROV_OPT_H
