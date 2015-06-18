#include "pid.h"
// #include <iostream>
// using namespace std;

Pid::Pid()
{
    // cout << "Hello World" << endl;
}

Pid::~Pid()
{
}


int Pid::Init(float kp, float ki, float kd)
{
    /* Initialize controller parameters */
    g_kp = kp;
    g_ki = ki;
    g_kd = kd;
    return 0;
}

float Pid::Update(float error)
{
    static float prev_error = 0;
    static float prev2_error = 0;
    // static int hello=0;
    float tp, ti, td;

    // cout << "g_kp = " << g_kp << endl;
    // cout << "g_ki = " << g_ki << endl;
    // cout << "g_kd = " << g_kd << endl;

    tp = g_kp * (error - prev_error);
    ti = g_ki * error;
    td = g_kd * (error - 2 * prev_error + prev2_error);

    // hello++;
    prev2_error = prev_error;
    prev_error = error;

    // cout << "hello = " << hello << endl;
    // cout << endl;

    return (tp + ti + td);
}
