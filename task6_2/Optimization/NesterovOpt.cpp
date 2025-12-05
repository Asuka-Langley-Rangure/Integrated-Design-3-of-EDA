#include "NesterovOpt.h"

// ================ NSIter ================

void NSIter::resize(std::size_t length)
{
    main_solution.resize(length);
    reference_solution.resize(length);
    gradient.resize(length);
}

// ================ NesterovOpt 私有工具函数 ================

void NesterovOpt::pushPositionToPlacer(const std::vector<VECTOR_3D> &pos)
{
    // 写回 MyPlacer/PlaceData
    placer->setPosition(pos);
}

void NesterovOpt::pullGradientFromPlacer(std::vector<VECTOR_3D> &grad)
{
    const std::vector<VECTOR_3D> &src = placer->totalGradient;
    grad.resize(src.size());
    for (std::size_t i = 0; i < src.size(); ++i)
        grad[i] = src[i];
}

double NesterovOpt::computeStepLength(const std::vector<VECTOR_3D> &v_new,
                                      const std::vector<VECTOR_3D> &v_old,
                                      const std::vector<VECTOR_3D> &g_new,
                                      const std::vector<VECTOR_3D> &g_old) const
{
    double num = 0.0;
    double den = 0.0;
    std::size_t n = v_new.size();

    for (std::size_t i = 0; i < n; ++i)
    {
        double dx = static_cast<double>(v_new[i].x) - static_cast<double>(v_old[i].x);
        double dy = static_cast<double>(v_new[i].y) - static_cast<double>(v_old[i].y);
        num += dx * dx + dy * dy;

        double dgx = static_cast<double>(g_new[i].x) - static_cast<double>(g_old[i].x);
        double dgy = static_cast<double>(g_new[i].y) - static_cast<double>(g_old[i].y);
        den += dgx * dgx + dgy * dgy;
    }

    if (den <= 0.0 || num <= 0.0)
        return 0.0;

    return std::sqrt(num / den);
}

// 直接在 NesterovOpt 里实现 HPWL 计算（用 db->Nets）
double NesterovOpt::computeHPWLFromDB() const
{
    if (!placer || !placer->db) return 0.0;

    PlaceData *db = placer->db;
    double total = 0.0;

    for (Net *net : db->Nets)
    {
        if (!net) continue;
        const std::vector<Pin*> &pins = net->netPins;
        int P = static_cast<int>(pins.size());
        if (P < 2) continue;

        double minX = 0.0, maxX = 0.0;
        double minY = 0.0, maxY = 0.0;
        bool first = true;

        for (Pin *p : pins)
        {
            if (!p || !p->module) continue;
            Module *m = p->module;

            double cx = m->center.x;
            double cy = m->center.y;
            double px = cx + p->offset.x;
            double py = cy + p->offset.y;

            if (first)
            {
                minX = maxX = px;
                minY = maxY = py;
                first = false;
            }
            else
            {
                if (px < minX) minX = px;
                if (px > maxX) maxX = px;
                if (py < minY) minY = py;
                if (py > maxY) maxY = py;
            }
        }
        if (!first)
            total += (maxX - minX) + (maxY - minY);
    }
    return total;
}

void NesterovOpt::updateLambda(double curHPWL)
{
    // pdf 式 (37)：λ_k = μ_k * λ_{k-1}
    // μ_k = 1.1 - (ΔHPWL_k / ΔHPWL_REF) + 1.0
    // ΔHPWL_REF = 3.5e5
    const double DELTA_HPWL_REF = 3.5e5;

    double deltaHPWL = curHPWL - lastHPWL;
    double mu_k = 1.1 - (deltaHPWL / DELTA_HPWL_REF) + 1.0;

    // 做一点裁剪，避免 λ 变成负数或者爆炸
    if (mu_k < 0.5) mu_k = 0.5;
    if (mu_k > 2.0) mu_k = 2.0;

    placer->lambda = static_cast<float>(placer->lambda * mu_k);
    lastHPWL = curHPWL;
}

// ================ NesterovOpt::init ================

void NesterovOpt::init()
{
    iter_count   = 0;
    NS_opt_param = 1.0f;    // β_0
    step_alpha   = 1e-3f;   // 初始步长猜一个较小值

    // 1) 先尝试直接拿位置
    std::vector<VECTOR_3D> pos = placer->getPosition();

    // 如果位置为空，很可能没有调用 MyPlacer::Init()，按 pdf 流程自动初始化一次
    if (pos.empty())
    {
        placer->Init();              // 里面会 FillerInit / BinInit / gradient init
        pos = placer->getPosition();
    }

    cur_iter.resize(pos.size());
    last_iter.resize(pos.size());

    if (pos.empty())
    {
        // 说明 NodesAndFillers 真的是空的（比如没有节点），直接退出
        lastHPWL = 0.0;
        std::printf("[Nesterov] init: numVars = 0 (no movable nodes)\n");
        return;
    }

    // u^0 = v^0 = 当前位置
    cur_iter.main_solution      = pos;
    cur_iter.reference_solution = pos;

    // 写回 db，保证 MyPlacer 中和 Nesterov 一致
    pushPositionToPlacer(cur_iter.reference_solution);

    // 计算初始梯度 ∇f_pre(v^0)
    placer->GetTotalGradient();
    pullGradientFromPlacer(cur_iter.gradient);

    // 初始时 last_iter = cur_iter
    last_iter = cur_iter;

    // 初始 HPWL，用于之后的 ΔHPWL_k 和 lambda 更新
    lastHPWL = computeHPWLFromDB();

    std::printf("[Nesterov] init: numVars = %d\n",
                (int)cur_iter.reference_solution.size());
}

// ================ NesterovOpt::NAG_Step ================

void NesterovOpt::NAG_Step()
{
    std::size_t n = cur_iter.reference_solution.size();
    if (n == 0) return;

    NSIter new_iter;
    new_iter.resize(n);

    // ---- 步骤 3-1：用 v_k, v_{k-1}, g_k, g_{k-1} 估计 Lipschitz L_k 并更新 α_k ----
    if (iter_count > 0)
    {
        double Lk = computeStepLength(cur_iter.reference_solution,
                                      last_iter.reference_solution,
                                      cur_iter.gradient,
                                      last_iter.gradient);
        if (Lk > 0.0)
            step_alpha = static_cast<float>(1.0 / Lk);
    }

    // ---- 步骤 3-2：更新“动量系数” β_{k+1}（FISTA 形式）----
    float beta_prev = NS_opt_param;
    float beta_new  = 0.5f * (1.0f + std::sqrt(1.0f + 4.0f * beta_prev * beta_prev));

    // ---- 步骤 3-3：带回溯的预测位置 ----
    float alpha_k = step_alpha;

    while (true)
    {
        // 3-3-1-1：û^{k+1} = v^k - α_k ∇f_pre(v^k)
        // 注意：totalGradient 已经是“下降方向”，所以这里写成 v_k + α_k * gradient
        for (std::size_t i = 0; i < n; ++i)
        {
            const VECTOR_3D &vk = cur_iter.reference_solution[i];
            const VECTOR_3D &gk = cur_iter.gradient[i];

            new_iter.main_solution[i].x = vk.x + alpha_k * gk.x;
            new_iter.main_solution[i].y = vk.y + alpha_k * gk.y;
            new_iter.main_solution[i].z = 0.0f;
        }

        // 3-3-1-2：v̂^{k+1} = û^{k+1} + (β_k - 1)/β_{k+1} * (û^{k+1} - u^k)
        float momentum = 0.0f;
        if (iter_count > 0)
            momentum = (beta_prev - 1.0f) / beta_new;
        else
            momentum = 0.0f;

        for (std::size_t i = 0; i < n; ++i)
        {
            const VECTOR_3D &u_old = cur_iter.main_solution[i];
            const VECTOR_3D &u_new = new_iter.main_solution[i];

            new_iter.reference_solution[i].x =
                u_new.x + momentum * (u_new.x - u_old.x);
            new_iter.reference_solution[i].y =
                u_new.y + momentum * (u_new.y - u_old.y);
            new_iter.reference_solution[i].z = 0.0f;
        }

        // 3-3-2：用 v̂^{k+1} 更新 db 中位置
        pushPositionToPlacer(new_iter.reference_solution);

        // 3-3-3：重新计算梯度 ∇f_pre(v̂^{k+1})
        placer->GetTotalGradient();
        pullGradientFromPlacer(new_iter.gradient);

        // 3-3-4：计算 α̂_{k+1}
        double alpha_hat_next = computeStepLength(
            new_iter.reference_solution,
            cur_iter.reference_solution,
            new_iter.gradient,
            cur_iter.gradient);

        if (alpha_hat_next <= 0.0)
        {
            // 无法估计 Lipschitz，就用当前 alpha 直接接受
            break;
        }

        // 3-3-5：判断 α_k > ε * α̂_{k+1} ？（pdf 式 35）
        if (alpha_k > BKTRK_EPS * alpha_hat_next)
        {
            // 需要缩小步长：α_k = α̂_{k+1}，重新回到 3-3
            alpha_k = static_cast<float>(alpha_hat_next);
            continue;
        }
        else
        {
            // 满足回溯条件，接受当前 α_k，更新 β_k = β_{k+1}
            step_alpha   = alpha_k;
            NS_opt_param = beta_new;
            break;
        }
    }

    // 此时 new_iter 即为 (u^{k+1}, v^{k+1}, g^{k+1})，db 里也已经是 v^{k+1} 的位置

    // ---- 步骤 4：更新 λ（需要 HPWL 变化）----
    double curHPWL = computeHPWLFromDB();
    updateLambda(curHPWL);

    // ---- 更新迭代状态 ----
    last_iter = cur_iter;
    cur_iter  = new_iter;
    ++iter_count;

    std::printf("[Nesterov] iter = %zu, step = %.3e, overflow = %.6f, HPWL = %.6f\n",
                iter_count,
                static_cast<double>(step_alpha),
                static_cast<double>(placer->globalDensityOverflow),
                curHPWL);
}

// ================ NesterovOpt::NAG_Process ================

void NesterovOpt::NAG_Process()
{
    if (!placer) return;

    float targetOverflow = placer->targetOverflow;

    init();

    std::size_t n = cur_iter.reference_solution.size();
    std::printf("[Nesterov] start: targetOverflow = %.6f\n",
                static_cast<double>(targetOverflow));

    if (n == 0)
    {
        std::printf("[Nesterov] no variables to optimize, skip.\n");
        return;
    }

    // pdf 3.1：溢出 > 目标 且 未超过最大迭代次数 时继续迭代
    while (placer->globalDensityOverflow > targetOverflow &&
           iter_count < MAX_ITERATION)
    {
        NAG_Step();
    }

    // 最后再把参考解写回 db，防止遗漏
    pushPositionToPlacer(cur_iter.reference_solution);

    std::printf("[Nesterov] finish: iter = %zu, finalOverflow = %.6f\n",
                iter_count,
                static_cast<double>(placer->globalDensityOverflow));
}
