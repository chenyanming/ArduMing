static float g_kp, g_ki, g_kd;

void pid_initialize(float kp, float ki, float kd)
{
    /* Initialize controller parameters */
    g_kp = kp;
    g_ki = ki;
    g_kd = kd;
}

float pid_update(float error)
{
    static float prev_error = 0;
    static float prev2_error = 0;
    float tp, ti, td;

    tp = g_kp * (error - prev_error);
    ti = g_ki * error;
    td = g_kd * (error - 2 * prev_error + prev2_error);

    prev2_error = prev_error;
    prev_error = error;

    return (tp + ti + td);
}
