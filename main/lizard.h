#ifndef LIZARD_H
#define LIZARD_H

#include <stdbool.h>

// Call once from app_main() before the gamepad/CRSF tasks start.
// Initializes LEDC PWM channels for all 7 servos and creates the
// lizard gait FreeRTOS task.
void lizard_init(void);

// Call from tx_task() (or anywhere polling the gamepad) every loop with
// the CURRENT held state of D-pad "forward/up" (true while pressed,
// false once released). Non-blocking. While held, the gait runs
// continuously; as soon as held goes false, servos snap back to the
// center/zero pose (within one ~50ms step).
void lizard_set_forward_held(bool held);

// Same pattern as lizard_set_forward_held(), but for D-pad left/right.
// While held, the same walking gait runs but with a constant turn bias
// added to the body/tail joints (ported from the old Dynamixel
// "turn_amp" logic) so the robot arcs left or right while walking.
// Legs are unaffected, matching the original MATLAB behavior.
void lizard_set_left_held(bool held);
void lizard_set_right_held(bool held);

// Same pattern as lizard_set_forward_held(), but for D-pad down. Uses
// the reversed leg phase timing (m3/m6 <-> m4/m5 swapped vs. forward)
// to walk backward. No turn bias applied.
void lizard_set_backward_held(bool held);

#endif // LIZARD_H
