#include "manual_tune.h"
#include <stdint.h>


// Which gain is the switch pointing at?
static pid_sel_t pid_selected(uint16_t button2)
{
    if (button2 < 719)  return PID_SEL_P;
    if (button2 < 1265) return PID_SEL_I;
    return PID_SEL_D;
}

// Your tune_pid, now returns the selection so you can feed it to the display
pid_sel_t tune_pid(uint16_t button1, uint16_t button2, uint16_t slider, Control *c)
{
    pid_sel_t sel = pid_selected(button2);

    if (button1 > SWITCH_HIGH)
    {
        control_init(c);
        return sel;
    }

    float value = 0.0f;
    if (slider > CRSF_MIN)
    {
        value = (float)(slider - CRSF_MIN) / (float)(CRSF_MAX - CRSF_MIN);
    }
    if (value > 1.0f) value = 1.0f;

    switch (sel)
    {
    case PID_SEL_P:
    {
        float kp = 0.01f + value * (0.15f - 0.01f);
        c->roll_rate.kp = kp;
        c->pitch_rate.kp = kp;
        break;
    }
    case PID_SEL_I:
    {
        float ki = value * 0.10f;
        c->roll_rate.ki = ki;
        c->pitch_rate.ki = ki;
        break;
    }
    default:
    {
        float kd = value * 0.002f;
        c->roll_rate.kd = kd;
        c->pitch_rate.kd = kd;
        break;
    }
    }

    return sel;
}

