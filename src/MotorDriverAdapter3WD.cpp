#include "MotorDriverAdapter3WD.h"

MotorDriverAdapter3WD::MotorDriverAdapter3WD(uint8_t Fcw, uint8_t Fccw, uint8_t BRcw, uint8_t BRccw, uint8_t BLcw, uint8_t BLccw)
{
    FCWPin = Fcw;
    FCCWPin = Fccw;
    BRCWPin = BRcw;
    BRCCWPin = BRccw;
    BLCWPin = BLcw;
    BLCCWPin = BLccw;
    pinMode(FCWPin,OUTPUT);
    pinMode(FCCWPin,OUTPUT); 
    pinMode(BRCWPin,OUTPUT); 
    pinMode(BRCCWPin,OUTPUT);
    pinMode(BLCWPin,OUTPUT); 
    pinMode(BLCCWPin,OUTPUT);
    for (int i = 0; i < 3; i++)
    {
        prevPWM[i] = 0;
    }
}

void MotorDriverAdapter3WD::apply(int pwm[3])
{
    for (int i = 0; i < 2; i++)
    {
        pwm[i] = RCconstant * prevPWM[i] + (1 - RCconstant) * pwm[i];
        prevPWM[i] = pwm[i];
    }

    duty[0] = pwm[0];
    duty[1] = pwm[1];
    duty[2] = pwm[2];

    if (0 < duty[0])
    {
        analogWrite(FCWPin,abs(duty[0]));
        analogWrite(FCCWPin,0);
    }
    else if (duty[0] < 0)
    {
        analogWrite(FCWPin,0);
        analogWrite(FCCWPin,abs(duty[0]));
    }
    else
    {
        analogWrite(FCWPin,0);
        analogWrite(FCCWPin,0);
    }

    if (0 < duty[1])
    {
        analogWrite(BRCWPin,abs(duty[1]));
        analogWrite(BRCCWPin,0);
    }
    else if (duty[1] < 0)
    {
        analogWrite(BRCWPin,0);
        analogWrite(BRCCWPin,abs(duty[1]));
    }
    else
    {
        analogWrite(BRCWPin,0);
        analogWrite(BRCCWPin,0);
    }

    if (0 < duty[2])
    {
        analogWrite(BLCWPin,abs(duty[2]));
        analogWrite(BLCCWPin,0);
    }
    else if (duty[2] < 0)
    {
        analogWrite(BLCWPin,0);
        analogWrite(BLCCWPin,abs(duty[2]));
    }
    else
    {
        analogWrite(BLCWPin,0);
        analogWrite(BLCCWPin,0);
    }
}