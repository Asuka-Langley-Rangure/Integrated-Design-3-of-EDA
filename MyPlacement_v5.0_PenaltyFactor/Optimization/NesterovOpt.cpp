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
        // step_size = 0.01;
        step_size = 1;
    }
    else
    {
        float lipschitz_constant = calc_lipschitz_constant(cur_iter.reference_solution, last_iter.reference_solution, cur_iter.gradient, last_iter.gradient);
        step_size = 1 / lipschitz_constant;
    }

    printf("stepSize : %f\n", step_size);

    float new_NS_opt_param = (1 + sqrt(4 * NS_opt_param*NS_opt_param + 1)) / 2; // ak+1

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
        float new_lipschitz_constant = calc_lipschitz_constant(new_iter.reference_solution, cur_iter.reference_solution, new_iter.gradient, cur_iter.gradient);
        float new_step_size = 1 / new_lipschitz_constant;   // 根据更新后单元位置、梯度信息计算新步长
        if (BKTRK_EPS * step_size <= new_step_size) // 若当前步长*BKTRK_EPS小于新步长，则停止回溯
        {
            step_size = new_step_size;  
            break;
        }
        printf("bktrk!\n");
        step_size = new_step_size;  // 若当前步长*BKTRK_EPS大于新步长，则缩小当前步长至新步长再次更新单元位置
    }
    // update iter variable
    last_iter = cur_iter;
    cur_iter = new_iter;
    NS_opt_param = new_NS_opt_param;
    placer->updatePenaltyFactor();
}

void NesterovOpt::NAG_Process()
{
    float targetOverflow = placer->targetOverflow;
    init();

    while(!(placer->globalDensityOverflow < targetOverflow) || (iter_count > MAX_ITERATION))
    {
           NAG_Step();
    }
}