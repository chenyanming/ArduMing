#ifndef PID_H
#define PID_H

class Pid
{
public:
    Pid();
    ~Pid();
    int Init(float, float, float);
    float Update(float);
private:
    float g_kp, g_ki, g_kd;
};

#endif // PID_H