#ifndef _pid_h_
#define _pid_h_
typedef struct
{
    double A0;          /**< The derived gain, A0 = Kp + Ki + Kd . */
    double A1;          /**< The derived gain, A1 = -Kp - 2Kd. */
    double A2;          /**< The derived gain, A2 = Kd . */
    double state[3];    /**< The state array of length 3. */
    double Kp;               /**< The proportional gain. */
    double Ki;               /**< The integral gain. */
    double Kd;               /**< The derivative gain. */
} arm_pid_instance_f32;

double arm_pid_speed(arm_pid_instance_f32 * S,double in);
double arm_pid_state(arm_pid_instance_f32 * S,double in);
void arm_pid_init_f32(arm_pid_instance_f32 * S,int resetStateFlag);

#endif
