#include <stdio.h>
#include <float.h>


class MotorControl
{
public:
    bool m_bConnected = false;
    int m_r = 0;
    int m_l = 0;
    
    MotorControl(void);

    void drive(int rval, int lval);
};