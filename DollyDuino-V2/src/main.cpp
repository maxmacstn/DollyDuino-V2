#include <Arduino.h>
#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include "hidjoystickrptparser.h"
#include "Slider.h"
#include "AccelStepper.h"

USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb); //Default pins use P5, P17 for ESP32 board for SPI_SS and INTR
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);
TaskHandle_t usbTaskHandle;

byte gamePadRpos = 127;

Slider slider = Slider();

#define GAMEPAD_BUTTON_A 2
#define GAMEPAD_BUTTON_B 3
#define GAMEPAD_BUTTON_X 1
#define GAMEPAD_BUTTON_Y 4
#define GAMEPAD_BUTTON_UP 00
#define GAMEPAD_BUTTON_DN 04
#define GAMEPAD_BUTTON_L 06
#define GAMEPAD_BUTTON_R 02
#define GAMEPAD_BUTTON_LB 5
#define GAMEPAD_BUTTON_RB 6
#define GAMEPAD_BUTTON_LT 7
#define GAMEPAD_BUTTON_RT 8
#define GAMEPAD_BUTTON_BACK 9
#define GAMEPAD_BUTTON_START 10

void usbTask( void * pvParameters ){
  for(;;){
        Usb.Task();
        vTaskDelay(1);
  }
}



void setup()
{
        // Serial.begin(115200);
        Serial.println("Start");
        slider.init();
        if (Usb.Init() == -1){
                Serial.println("OSC did not start.");
                slider.displayBootMessage("USB Error");
                while(1){};
        }
        delay(200);
        if (!Hid.SetReportParser(0, &Joy)){
                ErrorMessage<uint8_t>(PSTR("SetReportParser"), 1);
                slider.displayBootMessage("USB Parser error");
                while(1){};
        }
        
        slider.displayBootMessage("USB OK!");
        for(int i = 0; i <100; i++){
                Usb.Task();
                delay(10);
        }

        slider.onAnalogStickChanged(0x80, 0x80,  0x80, 0x80);
        
        
}


void loop()
{
        // put your main code here, to run repeatedly:
        Usb.Task();
        slider.run();
        slider.updateScreen();

        // stepper_pan.run();
        // float currentPanSpeed = map(abs(gamePadRpos-128),0,128,0,panDegreesToSteps(180));
        // if (gamePadRpos > 129 ){
        //         stepper_pan.moveTo(stepper_pan.currentPosition() + 1);
        //         stepper_pan.setSpeed(currentPanSpeed);
        // }else if( gamePadRpos < 125 ){
        //         stepper_pan.moveTo(stepper_pan.currentPosition() - 1);
        //         stepper_pan.setSpeed(-currentPanSpeed);

        // }



}

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt)
{
        Serial.print("X1: ");
        PrintHex<uint8_t>(evt->X, 0x80);
        Serial.print("\tY1: ");
        PrintHex<uint8_t>(evt->Y, 0x80);
        Serial.print("\tX2: ");
        PrintHex<uint8_t>(evt->Z1, 0x80);
        Serial.print("\tY2: ");
        PrintHex<uint8_t>(evt->Z2, 0x80);
        Serial.print("\tRz: ");
        PrintHex<uint8_t>(evt->Rz, 0x80);
        Serial.println("");

        // analogLposX = evt->Y;
        // analogLposY = evt->Z1;
        // analogRposX = evt->Z2;
        // analogRposY = evt->Rz;


        slider.onAnalogStickChanged(evt->Y, evt->Z1,  evt->Z2, evt->Rz);
}

void JoystickEvents::OnHatSwitch(uint8_t hat)
{
        Serial.print("Hat Switch: ");
        PrintHex<uint8_t>(hat, 0x80);
        Serial.println("");

        switch (hat)
        {
        case GAMEPAD_BUTTON_L:
                slider.onPreviousModeButtonPressed();
                break;
        case GAMEPAD_BUTTON_R:
                slider.onNextModeButtonPressed();
                break;
        case GAMEPAD_BUTTON_UP:
                slider.onUpButtonPressed();
                break;
        case GAMEPAD_BUTTON_DN:
                slider.onDnButtonPressed();
                break;
        default:
                break;
        }
}

void JoystickEvents::OnButtonUp(uint8_t but_id)
{
        Serial.print("Up: ");
        Serial.println(but_id, DEC);
        switch (but_id)
        {
        case GAMEPAD_BUTTON_Y:
                slider.disableAllMotors(false);
        

        default:
                break;
        }
}

void JoystickEvents::OnButtonDn(uint8_t but_id)
{
        Serial.print("Dn: ");
        Serial.println(but_id, DEC);

        switch (but_id)
        {
        case GAMEPAD_BUTTON_A:
                slider.onActionButtonPressed();
                // Serial.println("Trigger");
                // canon_ble.triggerPhoto();
                break;
        case GAMEPAD_BUTTON_START:
                slider.onStartButtonPressed();
                break;
        case GAMEPAD_BUTTON_B:
                slider.onBackButtonPressed();
                break;
        case GAMEPAD_BUTTON_LB:
                slider.onLBButtonPressed();
                break;
        case GAMEPAD_BUTTON_RB:
                slider.onRBButtonPressed();
                break;
        case GAMEPAD_BUTTON_Y:
                slider.onAddKeyFrameButtonPressed();
                break;
        case GAMEPAD_BUTTON_LT:
                slider.onLTButtonPressed();
                break;
        case GAMEPAD_BUTTON_RT:
                slider.onRTButtonPressed();
                break;

        case GAMEPAD_BUTTON_X:
                slider.disableAllMotors(true);
                break;
        case GAMEPAD_BUTTON_BACK:
                slider.onSetHomeButtonPressed();
                break;
        
        default:
                break;
        }
}