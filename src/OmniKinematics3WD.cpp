#include "OmniKinematics3WD.h"

void OmniKinematics3WD::getOutput(int x, int y, int yaw, float yawAngle, int pwm[3])
{
    XVector = x;
    YVector = y;
    YawVector = yaw;     //ユーザーからの回転量
    yawAngle = yawAngle; //ロボットの傾き

    //逆運動学を使って各軸の移動量からモータの回転方向・量を計算する
    if (YawVector != 0)
    { //ユーザーからの回転命令があるとき
        userBias = yawAngle;
        pwm[0] = -XVector * cos(yawAngle * DEG_TO_RAD) - YVector * sin(yawAngle * DEG_TO_RAD) + YawVector;
        pwm[1] = -XVector * cos((yawAngle + 2.094) * DEG_TO_RAD) + YVector * sin((yawAngle + 2.094) * DEG_TO_RAD) + YawVector;
        pwm[2] = -XVector * cos((yawAngle + 1.047) * DEG_TO_RAD) + YVector * sin((yawAngle + 1.047) * DEG_TO_RAD) + YawVector;
    }
    else
    { //IMUがYAW軸を自動補正するとき
        int outputYawPWM = (yawAngle - userBias) * 5;
        if (-5 < outputYawPWM && outputYawPWM < 5)
        {
            outputYawPWM = 0;
        }
        pwm[0] = -XVector * cos(yawAngle * DEG_TO_RAD) - YVector * sin(yawAngle * DEG_TO_RAD) + outputYawPWM;
        pwm[1] = -XVector * cos((yawAngle + 2.094) * DEG_TO_RAD) + YVector * sin((yawAngle + 2.094) * DEG_TO_RAD) + outputYawPWM;
        pwm[2] = -XVector * cos((yawAngle + 1.047) * DEG_TO_RAD) + YVector * sin((yawAngle + 1.047) * DEG_TO_RAD) + outputYawPWM;
    }

    /*pwm[0] = +(XVector)-YawVector;
    pwm[1] = +(XVector / 2) - (YVector * 1.732 / 2) + YawVector;
    pwm[2] = +(XVector / 2) + (YVector * 1.732 / 2) + YawVector;*/

    //計算上の最大出力を求める
    int max = 0;
    for (int i = 0; i < 3; i++)
    {
        if (max < abs(pwm[i]))
            max = abs(pwm[i]);
    }

    //最大出力が許容最大出力を超えていたら再計算する
    float rate = 0;
    if (maxAllocateOutput < max)
    {
        rate = max / maxAllocateOutput;
        for (int i = 0; i < 3; i++)
        {
            pwm[i] = pwm[i] / rate;
        }
    }
}