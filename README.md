
# DollyDuino 2.0
ESP32 Based camera motion controller, implement based on Arduino and PlatformIO. This project is the successor from my [orginal design](https://github.com/maxmacstn/Arduino-Motorized-Dolly).

**Video:**  https://youtu.be/K7UvhgVBU1E

⚠️ This project is currently in the development process, it might contain some unfinished features or bugs. However, it was functioning quite well so far.

![Bluetooth Remote for Modern Canon Cameras](https://raw.githubusercontent.com/maxmacstn/DollyDuino-V2/main/images/feature.jpg)



## Features

* 2 Axis control: Slide & Pan.
* 16x2 LCD Display.
* Controlled by USB HID interface gamepad.
* Built-in Canon EOS Camera remote using Bluetooth LE.
* USB-C PD Power input.
* USB-A power out for powering the camera or charging.

  

## Modes

1. Manual/Slide - Operate the camera movement manually using a gamepad, or set the automatic linear movement with smooth acceleration/deceleration.
2. Orbit Mode - Lock the camera facing towards the object while sliding.
3. Time lapse - Precise micro-movement with automatic camera triggering system.
  

## Major components
Please see full list of components and parts in [Bill of Materials](https://github.com/maxmacstn/DollyDuino-V2/blob/main/BOM.xlsx).
- Andeor camera slider.
- DollyDuino 2.0 controller.
	- Node32 Lite development board.
	- TMC2226 silent stepper driver.
	- LCD 16x2 Display.
	- ZY12PDN USB-C PD Module.
	- Mini USB Host shield.
- NEMA 17 Stepper Driver
- GT2 Timing belt and pulley wheels.
- A bunch of 3D-Printed parts.

## Gamepad buttons

| Button | Function                                        |
|--------|-------------------------------------------------|
| A      | Trigger camera shutter                          |
| B      | Clear in/out point, Stop auto slide             |
| X      | Unlock motors                                   |
| Y      | Set in/out point                                |
| Left   | Previous mode                                   |
| Right  | Next Mode                                       |
| Up     | Increment Interval                              |
| Down   | Decrement Interval                              |
| LT     | Decrement shutter speed                         |
| RT     | Increment shutter speed                         |
| LB     | Decrease sliding speed                          |
| RB     | Increase sliding speed                          |
| Start  | Start auto sliding movement                     |
| Back   | Override all axis position as home position (0) |
  

## References 
-  [3D Printed Motorised DSLR Camera Slider by issac879](https://www.youtube.com/watch?v=v1b7Wvu87-U)
- [Mini USB Host shield with esp8266](https://create.arduino.cc/projecthub/139994/plug-any-usb-device-on-an-esp8266-e0ca8a)

## To-do
- [ ] Fix bug in timelapse mode, the slider should not move when the camera shutter is opening.
- [ ] Re-design pan axis to be more rigid.
- [ ] Improve usability.
  

Feel free to contribute!
