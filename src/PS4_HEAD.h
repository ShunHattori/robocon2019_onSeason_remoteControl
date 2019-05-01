//AnalogButton
#define APUSH_L2        PS4.getAnalogButton(L2)
#define APUSH_R2        PS4.getAnalogButton(R2)
#define PUSH_L2         PS4.getButtonPress(L2)
#define PUSH_R2         PS4.getButtonPress(R2)
#define PUSH_L1         PS4.getButtonPress(L1)
#define PUSH_R1         PS4.getButtonPress(R1)
#define PUSH_CIRCLE     PS4.getButtonPress(CIRCLE)
#define PUSH_TRIANGLE   PS4.getButtonPress(TRIANGLE)
#define PUSH_SQUARE     PS4.getButtonPress(SQUARE)
#define PUSH_CROSS      PS4.getButtonPress(CROSS)
#define PUSH_UP         PS4.getButtonPress(UP)
#define PUSH_DOWN       PS4.getButtonPress(DOWN)
#define PUSH_LEFT       PS4.getButtonPress(LEFT)
#define PUSH_RIGHT      PS4.getButtonPress(RIGHT)

//ButtonClick
#define CLICK_SHARE     PS4.getButtonClick(SHARE)
#define CLICK_TOUCHPAD  PS4.getButtonClick(TOUCHPAD)
#define CLICK_OPTIONS   PS4.getButtonClick(OPTIONS)
#define CLICK_L1        PS4.getButtonClick(L1)
#define CLICK_R1        PS4.getButtonClick(R1)
#define CLICK_CIRCLE    PS4.getButtonClick(CIRCLE)
#define CLICK_CROSS     PS4.getButtonClick(CROSS)
#define CLICK_SQUARE    PS4.getButtonClick(SQUARE)
#define CLICK_TRIANGLE  PS4.getButtonClick(TRIANGLE)
#define CLICK_UP        PS4.getButtonClick(UP)
#define CLICK_DOWN      PS4.getButtonClick(DOWN)
#define CLICK_LEFT      PS4.getButtonClick(LEFT)
#define CLICK_RIGHT     PS4.getButtonClick(RIGHT)

//Stick
double HatY_S  = 0;
double HatX_S  = 0;
double LeftHatY_S  = 0;
double LeftHatX_S  = 0;
double RightHatY_S = 0;
double RightHatX_S = 0;
double R2Hat_S = 0;
#define LH_Y_G    60
#define LH_Y_B   195
#define LH_Y_L    60
#define LH_Y_R   195
#define RH_Y_G    60
#define RH_Y_B   190
#define RH_Y_L    60
#define RH_Y_R   190
#define STICK_L_GO      LeftHatY_S  < LH_Y_G
#define STICK_L_BACK    LeftHatY_S  > LH_Y_B
#define STICK_L_L       LeftHatX_S  < LH_Y_L
#define STICK_L_R       LeftHatX_S  > LH_Y_R
#define STICK_L_RF      STICK_L_GO && STICK_L_R
#define STICK_L_LF      STICK_L_GO && STICK_L_L
#define STICK_L_RB      STICK_L_BACK && STICK_L_R
#define STICK_L_LB      STICK_L_BACK && STICK_L_L
#define STICK_R_GO      RightHatY_S < LH_Y_G
#define STICK_R_BACK    RightHatY_S > LH_Y_B
#define STICK_R_L       RightHatX_S < LH_Y_L
#define STICK_R_R       RightHatX_S > LH_Y_R
