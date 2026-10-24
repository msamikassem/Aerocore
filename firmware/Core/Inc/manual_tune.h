#ifndef MANUAL_TUNE_H
#define MANUAL_TUNE_H

#include <stdint.h>
#include "control.h"  // Must define Control

/**
 * @brief PID gain selection
 *
 * P selects the proportional gain, I selects the integral gain,
 * and D selects the derivative gain.
 */
typedef enum
{
    PID_SEL_P = 0,
    PID_SEL_I = 1,
    PID_SEL_D = 2
} pid_sel_t;

/**
 * @brief Adjust PID gains using the controller inputs
 *
 * @param button1 First button value
 * @param button2 Second button value
 * @param slider Slider value
 * @param c Pointer to the PID control structure
 * @return Selected PID gain (P, I, or D)
 */
pid_sel_t tune_pid(uint16_t button1, uint16_t button2, uint16_t slider, Control *c);

/**
 * @brief Format the selected PID gain as text
 *
 * @param out Output buffer for the text
 * @param size Size of the output buffer
 * @param sel Selected PID gain (P, I, or D)
 * @param c Pointer to the PID control structure
 */
void CRSF_pid_text(char *out, unsigned int size, pid_sel_t sel, const Control *c);

/**
 * @brief Build a CRSF flight mode telemetry frame
 *
 * @param packet Output buffer for the packet
 * @param text Flight mode text to send
 * @return Number of bytes in the packet
 */
uint8_t CRSF_build_fm_frame(uint8_t *packet, const char *text);

#endif
