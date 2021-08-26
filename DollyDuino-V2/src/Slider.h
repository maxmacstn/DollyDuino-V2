#ifndef SLIDER_H
#define SLIDER_H

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <CanonBLERemote.h>

#define PIN_SLIDE_STEP 26
#define PIN_SLIDE_DIR 27
#define PIN_PAN_STEP 14
#define PIN_PAN_DIR 12
#define PIN_SLIDER_LIMIT_LEFT 35
#define PIN_SLIDER_LIMIT_RIGHT 34
#define PIN_MS1 32
#define PIN_MS2 33
#define PIN_MS3 25
#define PIN_MOTOR_EN 13
#define PIN_LCD_SCL 22
#define PIN_LCD_SDA 21
#define PIN_AUX_IO_1 16
#define PIN_AUX_IO_2 4

#define HALF_STEP 2
#define QUARTER_STEP 4
#define EIGHTH_STEP 8
#define SIXTEENTH_STEP 16

#define PAN_GEAR_RATIO 2.75
#define SLIDER_PULLEY_TEETH 20

#define INVERT_SLIDE 1 //1 = invert ; 0 = non-invert
#define INVERT_PAN 0   //1 = invert ; 0 = non-invert

#define ANALOG_DEADZONE 3
// #define KEYFRAME_ARRAY_LENGTH 32
#define KEYFRAME_ARRAY_LENGTH 2

#define MAX_SLIDE_SPEED 90  // 60mm/s
#define MAX_PAN_SPEED 60    // 40deg/s

#define TRIGGER_MS 100

#define TOTAL_MODE 6
#define LCD_FPS 10




class Slider
{

    byte bluetoothChar[8] = {B00100,B00110,B10101,  B01110,  B01110, B10101,B00110,B00100};

    struct KeyframeElement
    {
        long panStepCount = 0;
        float panSpeed = 0;
        long tiltStepCount = 0;
        float tiltSpeed = 0;
        long sliderStepCount = 0;
        float sliderSpeed = 0;
        int msDelay = 0;
    };

    struct FloatCoordinate
    {
        float x;
        float y;
        float z;
    };

    struct LinePoints
    {
        float x0;
        float y0;
        float x1;
        float y1;
    };

    struct AnalogStick
    {
        int LposX;
        int LposY;
        int RposX;
        int RposY;
    };

public:
    // Slider();
    void init();
    void onPreviousModeButtonPressed();
    void onNextModeButtonPressed();
    void onAnalogStickChanged(byte LposX, byte LposY, byte RposX, byte RposY);
    void onActionButtonPressed();
    void onAddKeyFrameButtonPressed();
    void onLeftButtonPressed();
    void onRightButtonPressed();
    void onUpButtonPressed();
    void onDnButtonPressed();
    void onRBButtonPressed();
    void onLBButtonPressed();
    void onRTButtonPressed();
    void onLTButtonPressed();
    void onStartButtonPressed();
    void onBackButtonPressed();
    void onSetHomeButtonPressed();
    void disableAllMotors(bool);
    void run();
    void updateScreen(bool forceUpdate = false);
    void displayBootMessage(String);
    uint32_t panDegreeToStep(float);

private:
    LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
    AccelStepper stepper_pan;
    AccelStepper stepper_slide;
    MultiStepper multiStepper;
    bool isMoving = false;
    enum Mode
    {
        INIT,
        SLIDE,
        ORBIT,
        TIMELAPSE,
        SETTINGS
    };
  
    String modeString[TOTAL_MODE] = {"INIT", "SLIDE", "ORBIT", "TIMELAPSE","SETTINGS"};
    bool useEOSRemote = true;
    unsigned int timelapseDuration[8] = {1,5,10,20,30,60,90,120};
    uint8_t timelapseInterval = 0;
    uint8_t timelapseShutterTime = 1;
    byte timelapseSelectedtime = 4;
    byte eosPairingProcess = 0; //0 = idle, 1 = paring, 2 = pair success, 3 = pair failed
    Mode currentMode = SLIDE;
    float pan_steps_per_degree = (200.0 * SIXTEENTH_STEP * PAN_GEAR_RATIO) / 360.0;           //Stepper motor has 200 steps per 360 degrees
    float slider_steps_per_millimetre = (200.0 * SIXTEENTH_STEP) / (SLIDER_PULLEY_TEETH * 2); //Stepper motor has 200 steps per 360 degrees, the timing pully has 20 teeth and the belt has a pitch of 2mm
    float panSpeed = MAX_PAN_SPEED;                                                                   //degree per second
    float slideSpeed = MAX_SLIDE_SPEED;                                                                 //mm per second
    long target_position[3];                                                                  //Array to store stepper motor step counts
    float currentSpeedScale = 1.0;
    AnalogStick currentAnalogStick;
    float panDegreesToSteps(float angle);
    long sliderMillimetresToSteps(float mm);
    float sliderStepsToMillimetres(long steps);
    FloatCoordinate intercept;
    KeyframeElement keyframe_array[KEYFRAME_ARRAY_LENGTH];
    int keyframe_elements = 0;
    int current_keyframe_index = -1;
    unsigned long screenUpdateMills = 1000 / LCD_FPS;
    unsigned long lastScreenUpdate = 0;
    unsigned long lastShotTime = 0;
    bool autoMoveRunning = false;
    CanonBLERemote canonBLERemote = CanonBLERemote("DollyDuino 2.0");

    void initStepper();
    int addPosition();
    void clearKeyframes();
    void printKeyframeElements();
    void interpolateTargetPoint(FloatCoordinate, int);
    bool calculateTargetCoordinate();
    void goToTargetPosition(float panDeg, float sliderMillimetre, bool isBlocking);
    // float panStepsToDegrees(float);
    float panStepsToDegrees(long);
    int setTargetPositions(float panDeg, float sliderMillimetre);
    void doSlide();
    void doTimelapse();
    void triggerCamera();
    void pairEOSCamera();
};

#endif