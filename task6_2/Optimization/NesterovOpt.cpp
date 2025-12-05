#include "NesterovOpt.h"
#include <cmath>
#include <cstdio>

void NesterovOpt::pushPositionToPlacer(const std::vector<VECTOR_3D> &pos)
{
    // 将位置写回 MyPlacer / PlaceData
    // 保证 db 中的 NodesAndFillers 中心坐标与 pos 一致
    placer->setPosition(pos);
}

void NesterovOpt::pullGradientFromPlacer(std::vector<VECTOR_3D> &grad)
{
    // 这里使用的是 MyPlacer::totalGradient，
    // 在 GetTotalGradient() 中已经：
    // 1) 计算线长梯度
    // 2) 计算密度梯度
    // 3) 按 f = WL + λD 做线性叠加
    // 4) 做了预条件（preconditioner）
    // 5) **取了负号，使其成为“下降方向”**
    const std::vector<VECTOR_3D> &g_src = placer->totalGradient;

    grad.resize(g_src.size());
    for (std::size_t i = 0; i < g_src.size(); ++i)
    {
        grad[i] = g_src[i];
    }
}

double NesterovOpt::computeStepLength(const std::vector<VECTOR_3D> &v_new,
                                      const std::vector<VECTOR_3D> &v_old,
                                      const std::vector<VECTOR_3D> &g_new,
                                      const std::vector<VECTOR_3D> &g_old) const
{
    // 对应 pdf 式 (33) 之后的步长估计：
    // α̂_{k+1} = ||v^{k+1} - v^k|| / ||∇f_pre(v^{k+1}) - ∇f_pre(v^k)||
    double num = 0.0;
    double den = 0.0;

    const std::size_t n = v_new.size();
    for (std::size_t i = 0; i < n; ++i)
    {
        double dx = static_cast<double>(v_new[i].x) -
                    static_cast<double>(v_old[i].x);
        double dy = static_cast<double>(v_new[i].y) -
                    static_cast<double>(v_old[i].y);
        num += dx * dx + dy * dy;

        double dgx = static_cast<double>(g_new[i].x) -
                     static_cast<double>(g_old[i].x);
        double dgy = static_cast<double>(g_new[i].y) -
                     static_cast<double>(g_old[i].y);
        den += dgx * dgx + dgy * dgy;
    }

    if (den <= 0.0)
        return 0.0;

    return std::sqrt(num / den);
}

void NesterovOpt::init()
{
    iter_count   = 0;
    NS_opt_param = 1.0f;     // Nesterov 参数 t_0
    step_alpha   = 1e-3f;    // 初始步长，会在后续迭代中自适应更新

    // 读取当前模块位置（包括 Cells + Fillers），作为 u^0 = v^0
    std::vector<VECTOR_3D> initPos = placer->getPosition();

    cur_iter.resize(initPos.size());
    last_iter.resize(initPos.size());

    cur_iter.main_solution      = initPos;   // u^0
    cur_iter.reference_solution = initPos;   // v^0

    // 把参考位置写回 db，保证 MyPlacer 中的位置一致
    pushPositionToPlacer(cur_iter.reference_solution);

    // 计算初始梯度 ∇f_pre(v^0)
    placer->GetTotalGradient();
    pullGradientFromPlacer(cur_iter.gradient);

    // 初始时 last_iter 与 cur_iter 保持一致
    last_iter = cur_iter;

    std::printf("[Nesterov] init: numVars = %d\n",
                static_cast<int>(initPos.size()));
}

void NesterovOpt::NAG_Step()
{
    const std::size_t n = cur_iter.main_solution.size();
    if (n == 0)
        return;

    NSIter new_iter;
    new_iter.resize(n);

    // ---- 3-3-1-1：计算 u^{k+1}
    //
    // pdf 中公式： û^{k+1} = v^k - α^k * ∇f_pre(v^k)
    // 由于 MyPlacer::totalGradient 已经是“下降方向”（即 -∇f），
    // 这里用：
    //     u^{k+1} = v^k + α^k * gradient
    for (std::size_t i = 0; i < n; ++i)
    {
        const VECTOR_3D &vk = cur_iter.reference_solution[i];
        const VECTOR_3D &gk = cur_iter.gradient[i];

        new_iter.main_solution[i].x = vk.x + step_alpha * gk.x;
        new_iter.main_solution[i].y = vk.y + step_alpha * gk.y;
        new_iter.main_solution[i].z = 0.0f; // 2D 布局，z 置 0
    }

    // ---- 3-3-1-2：计算 v^{k+1}
    //
    // pdf 中公式 (34)： v̂^{k+1} = û^{k+1} + β * (û^{k+1} - u^k)
    // 这里采用常用的 Nesterov / FISTA 形式更新 β_k：
    //
    //   t_{k+1} = (1 + sqrt(1 + 4 t_k^2)) / 2
    //   β_k     = (t_k - 1) / t_{k+1}
    //
    float t_prev = NS_opt_param;
    float t_new  = 0.5f * (1.0f + std::sqrt(1.0f + 4.0f * t_prev * t_prev));
    float beta   = (t_prev - 1.0f) / t_new;

    for (std::size_t i = 0; i < n; ++i)
    {
        const VECTOR_3D &uk_old = cur_iter.main_solution[i];
        const VECTOR_3D &uk_new = new_iter.main_solution[i];

        new_iter.reference_solution[i].x =
            uk_new.x + beta * (uk_new.x - uk_old.x);
        new_iter.reference_solution[i].y =
            uk_new.y + beta * (uk_new.y - uk_old.y);
        new_iter.reference_solution[i].z = 0.0f;
    }

    // ---- 用 v^{k+1} 写回 db，并重新计算梯度与密度（包括溢出）
    pushPositionToPlacer(new_iter.reference_solution);
    placer->GetTotalGradient();
    pullGradientFromPlacer(new_iter.gradient);

    // ---- 3-3-4：基于 Lipschitz 估计更新步长 α_{k+1}
    double alpha_new = computeStepLength(
        new_iter.reference_solution,
        cur_iter.reference_solution,
        new_iter.gradient,
        cur_iter.gradient);

    if (alpha_new > 0.0)
    {
        // 3-3-5：回溯条件（对应 pdf 中 α^k > ε·α̂^{k+1}）
        //
        // 如果新估计的步长太小，则用 BKTRK_EPS 做一个缓和，
        // 防止步长在迭代初期骤降过快。
        if (step_alpha > BKTRK_EPS * alpha_new)
        {
            step_alpha = BKTRK_EPS * static_cast<float>(alpha_new);
        }
        else
        {
            step_alpha = static_cast<float>(alpha_new);
        }
    }

    // ---- 更新迭代状态
    last_iter    = cur_iter;
    cur_iter     = new_iter;
    NS_opt_param = t_new;
    ++iter_count;

    std::printf("[Nesterov] iter = %zu, step = %.3e, overflow = %.6f\n",
                iter_count,
                static_cast<double>(step_alpha),
                static_cast<double>(placer->globalDensityOverflow));
}

void NesterovOpt::NAG_Process()
{
    if (!placer)
        return;

    float targetOverflow = placer->targetOverflow;

    init();

    std::printf("[Nesterov] start: targetOverflow = %.6f\n",
                static_cast<double>(targetOverflow));

    // pdf 3.1：停止条件
    // 1) 全局密度溢出 τ < τ_min(targetOverflow)
    // 2) 迭代次数 > 最大迭代次数
    //
    // 因此循环条件应为：
    // while (溢出仍然偏大 && 还没到最大迭代次数)
    while (placer->globalDensityOverflow > targetOverflow &&
           iter_count < MAX_ITERATION)
    {
        NAG_Step();
    }

    // 最后再将当前参考解写回 db（防止最后一次没有同步）
    pushPositionToPlacer(cur_iter.reference_solution);

    std::printf("[Nesterov] finish: iter = %zu, finalOverflow = %.6f\n",
                iter_count,
                static_cast<double>(placer->globalDensityOverflow));
}
