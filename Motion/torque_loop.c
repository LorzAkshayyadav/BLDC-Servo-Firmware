#include "torque_loop.h"

void torque_loop_reset(void)
{
    /* The torque sensor is treated as observation-only and does not drive
     * any torque-loop state or correction. */
}

int32_t torque_loop_step(int32_t i_q_ref_ma, const params_t *p)
{
    (void)p;
    return i_q_ref_ma;
}
