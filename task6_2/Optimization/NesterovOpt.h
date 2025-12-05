#ifndef NESTEROV_OPT_H
#define NESTEROV_OPT_H

#include <vector>
#include <cstddef>
#include "common.h"
#include "myplace.h"

// 回溯步长的松弛因子（对应 pdf 中 ε）
// 取值略小于 1，使得步长不会剧烈震荡
#define BKTRK_EPS 0.95f

// 最大迭代次数（pdf 中“迭代次数大于规定最大迭代次数”）
#define MAX_ITERATION 300

class NSIter
{
public:
    void resize(std::size_t length)
    {
        main_solution.resize(length);
        reference_solution.resize(length);
        gradient.resize(length);
    }

    // u^k：主解（真实解）
    std::vector<VECTOR_3D> main_solution;
    // v^k：参考解（Nesterov 中间变量）
    std::vector<VECTOR_3D> reference_solution;
    // ∇f_pre(v^k)：预条件梯度（下降方向）
    std::vector<VECTOR_3D> gradient;
};

class NesterovOpt
{
public:
    explicit NesterovOpt(MyPlacer *placer) 
        : placer(placer),
          iter_count(0),
          NS_opt_param(1.0f),
          step_alpha(1e-3f) // 初始步长给一个较小值，后面会自适应更新
    {}

    // 初始化 Nesterov 迭代（读取初始位置，计算初始梯度）
    void init();

    // 单步 NAG 更新（对应 pdf 3.2 节“单步迭代”）
    void NAG_Step();

    // 整个 NAG 流程（对应 pdf 3.3 节“目标求解”）
    void NAG_Process();

    MyPlacer *placer;
    NSIter    cur_iter;   // 当前迭代的 (u^k, v^k, g^k)
    NSIter    last_iter;  // 前一迭代的 (u^{k-1}, v^{k-1}, g^{k-1})
    std::size_t iter_count;

    // Nesterov 参数（类似 pdf 中的 β_k / t_k）
    float NS_opt_param;

    // 当前步长 α_k
    float step_alpha;

private:
    // 把位置写回 db（非常关键：保证布局结果打入 PlaceData）
    void pushPositionToPlacer(const std::vector<VECTOR_3D> &pos);

    // 从 MyPlacer 中取出当前总梯度（已经是预条件、带 λ 的下降方向）
    void pullGradientFromPlacer(std::vector<VECTOR_3D> &grad);

    // 根据 pdf 3-3-4 计算新的步长 α^{k+1}
    double computeStepLength(const std::vector<VECTOR_3D> &v_new,
                             const std::vector<VECTOR_3D> &v_old,
                             const std::vector<VECTOR_3D> &g_new,
                             const std::vector<VECTOR_3D> &g_old) const;
};

#endif // NESTEROV_OPT_H
