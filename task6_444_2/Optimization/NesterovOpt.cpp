#include "NesterovOpt.h"

void NSIter::resize(size_t length)
{
    main_solution.resize(length);
    reference_solution.resize(length);
    gradient.resize(length);
}

void NesterovOpt::init()
{
    iter_count = 0;
    NS_opt_param = 1;

    // 确保 placer 的 bin / filler / 梯度等已初始化
    if (placer && placer->bins.empty())
    {
        placer->Init();
    }

    if (placer->NodesAndFillers.empty())
    {
        PlaceData* db = placer->db;
        if (!db)
        {
            std::cerr << "[NesterovOpt::init] placedata is null\n";
            return;
        }
        placer->NodesAndFillers.reserve(db->Nodes.size());

        for (Module* m : db->Nodes)
        {
            if (!m) continue;
            if (!m->isFixed)
            {
                placer->NodesAndFillers.push_back(m);
            }
        }
        std::cout << "[NesterovOpt::init] init NodesAndFillers from db, size = "
                  << placer->NodesAndFillers.size() << std::endl;
    }

    cur_iter.main_solution = placer->getPosition();
    
    printf("number of pos  %d\n", (int) (cur_iter.main_solution.size()));
}


void NesterovOpt::NAG_Step()
{
    cur_iter.reference_solution = placer->getPosition();
    placer->GetTotalGradient(); 
    cur_iter.gradient = placer->totalGradient;
    float step_size;
    NSIter new_iter;
    size_t length = cur_iter.main_solution.size();
    new_iter.resize(length);

    // initial step size
    if (iter_count == 0)
    {
        step_size = 0.01f; // 更保守的初始步长
    }
    else
    {
        float lipschitz_constant = calc_lipschitz_constant(
            cur_iter.reference_solution, last_iter.reference_solution,
            cur_iter.gradient, last_iter.gradient);
        if (!std::isfinite(lipschitz_constant) || lipschitz_constant <= 1e-12f)
            lipschitz_constant = 1e-12f;
        step_size = 1.0f / lipschitz_constant;
    }

    if (!std::isfinite(step_size) || step_size <= 0.0f)
        step_size = 0.01f;

    printf("stepSize : %f\n", step_size);

    float new_NS_opt_param = (1 + sqrt(4 * NS_opt_param*NS_opt_param + 1)) / 2; // ak+1

    int bk_iter = 0;
    const int BKTRK_LIMIT = 2000; // 防止回溯死循环
    while (true)
    {
        // first move cells
        for (size_t idx = 0; idx < length; idx++)
        {
            VECTOR_3D &new_position = new_iter.main_solution[idx];
            VECTOR_3D &new_reference_position = new_iter.reference_solution[idx];

            VECTOR_3D gradient = cur_iter.gradient[idx];
            VECTOR_3D cur_position = cur_iter.main_solution[idx];   // uk
            VECTOR_3D cur_reference_position = cur_iter.reference_solution[idx];    // vk
            new_position = cur_reference_position + gradient * step_size;   // uk+1（这个地方是加号，公式中是减号） 梯度并不是文章中的梯度，而是已经取了相反反向
            new_reference_position = new_position + (new_position - cur_position) * ((NS_opt_param - 1) / new_NS_opt_param);    // Vk+1
        }
        placer->setPosition(new_iter.reference_solution);   // 更新单元位置
        placer->GetTotalGradient();      // 根据更新后的单元位置更新梯度
        new_iter.gradient = placer->totalGradient;
        float new_lipschitz_constant = calc_lipschitz_constant(
            new_iter.reference_solution, cur_iter.reference_solution,
            new_iter.gradient, cur_iter.gradient);
        if (!std::isfinite(new_lipschitz_constant) || new_lipschitz_constant <= 1e-12f) {
            new_lipschitz_constant = 1e-12f; // 避免除零或 NaN
        }
        float new_step_size = 1.0f / new_lipschitz_constant;   // 根据更新后单元位置、梯度信息计算新步长
        if (!std::isfinite(new_step_size) || new_step_size <= 0.0f)
        {
            new_step_size = step_size * 0.5f; // fallback
        }
        // stop backtracking if new step is not smaller than BKTRK_EPS * current
        if (BKTRK_EPS * step_size <= new_step_size)
        {
            step_size = new_step_size;
            break;
        }
        printf("bktrk!\n");
        step_size = new_step_size;  // 若当前步长*BKTRK_EPS大于新步长，则缩小当前步长至新步长再次更新单元位置
        if (++bk_iter > BKTRK_LIMIT) {
            printf("bktrk hit limit, break to avoid infinite loop.\n");
            break;
        }
    }

    last_iter = cur_iter;
    cur_iter = new_iter;
    NS_opt_param = new_NS_opt_param;
    placer->updatePenaltyFactor();
}

void NesterovOpt::NAG_Process()
{
    float targetOverflow = placer->targetOverflow;
    init();

    placer->GetTotalGradient();

    while ((placer->globalDensityOverflow >= targetOverflow) && (iter_count < MAX_ITERATION))
    {
        NAG_Step();
        ++iter_count;

        // debug：每轮打印一下，确保你看得到迭代过程
        printf("[Iter %d] overflow = %f (target %f)\n",
               iter_count, placer->globalDensityOverflow, targetOverflow);
        if(iter_count % 10 == 0){        
            PLOTTING::plotPlacement(
            "iteration/nag_iter_" + std::to_string(iter_count),
            placer->db);
        }
    }

    printf("[NAG_Process] done. iter=%d, overflow=%f, target=%f\n",
           iter_count, placer->globalDensityOverflow, targetOverflow);
}
