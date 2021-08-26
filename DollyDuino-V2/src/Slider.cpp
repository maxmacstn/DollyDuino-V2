#include "Slider.h"
#include "utils.h"

void Slider::init()
{
    lcd.init(); // initialize the lcd
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print(" DollyDuino 2.0 ");
    delay(100);
    Serial.println("Start init slider");
    lcd.createChar(0, bluetoothChar);
    pinMode({PIN_SLIDE_STEP}, OUTPUT);
    pinMode(PIN_SLIDE_DIR, OUTPUT);
    pinMode(PIN_PAN_STEP, OUTPUT);
    pinMode(PIN_PAN_DIR, OUTPUT);
    pinMode(PIN_SLIDER_LIMIT_LEFT, INPUT_PULLUP);
    pinMode(PIN_SLIDER_LIMIT_RIGHT, INPUT_PULLUP);

    //Setup stepper
    initStepper();
    multiStepper.addStepper(stepper_slide);
    multiStepper.addStepper(stepper_pan);

    //setup AUX pin
    pinMode(PIN_AUX_IO_1, OUTPUT);
    digitalWrite(PIN_AUX_IO_1, LOW);

    //init canon BLE remote lib
    canonBLERemote.init();
    

    Serial.println("Init Complete");
    // lcd.clear();
}

void Slider::displayBootMessage(String msg){
    lcd.setCursor(0,1);
    lcd.print("                ");
    if (msg.length() < 16){
        lcd.setCursor(7 - (msg.length()/2), 1 );
        lcd.print(msg);
    }
}

void Slider::initStepper()
{

    long currentSlidePos = 0;
    long currentPanPos = -panDegreesToSteps(90);

    if (stepper_slide.currentPosition())
    {
        currentSlidePos = stepper_slide.currentPosition();
    }
    if (stepper_pan.currentPosition())
    {
        currentPanPos = stepper_pan.currentPosition();
    }

    pinMode(PIN_MS1, OUTPUT);
    pinMode(PIN_MS2, OUTPUT);
    pinMode(PIN_MS3, OUTPUT);
    pinMode(PIN_MOTOR_EN, OUTPUT);
    digitalWrite(PIN_MS1, HIGH);
    digitalWrite(PIN_MS2, HIGH);
    digitalWrite(PIN_MS3, HIGH);
    digitalWrite(PIN_MOTOR_EN, LOW);
    stepper_pan = AccelStepper(1, PIN_PAN_STEP, PIN_PAN_DIR);
    stepper_slide = AccelStepper(1, PIN_SLIDE_STEP, PIN_SLIDE_DIR);
    stepper_pan.setPinsInverted(INVERT_PAN, false, false);
    stepper_slide.setPinsInverted(INVERT_SLIDE, false, false);
    stepper_pan.setMaxSpeed(panDegreesToSteps(panSpeed));
    stepper_slide.setMaxSpeed(sliderMillimetresToSteps(slideSpeed));
    stepper_slide.setAcceleration(sliderMillimetresToSteps(100));
    stepper_pan.setAcceleration(panDegreesToSteps(360) * 2);

    stepper_slide.setCurrentPosition(currentSlidePos);
    stepper_pan.setCurrentPosition(currentPanPos);
}



void Slider::pairEOSCamera(){

    if (eosPairingProcess == 0){
        Serial.println("Pair EOS Camera");
        eosPairingProcess = 1;
        updateScreen(1);
            bool res = false;
            for (int i = 0; i < 5; i++){
                res =  canonBLERemote.pair(5);
                if (res)
                    break;
            }
            if (res){
                Serial.println("Pairing success");
                eosPairingProcess = 2;
            }else{
                Serial.println("Pairing failed");
                eosPairingProcess = 3;
            }
            updateScreen(1);
            delay(1000);
            eosPairingProcess = 0;
            updateScreen(1);

    }
}

void Slider::onAddKeyFrameButtonPressed(){
    Serial.println("onAddKeyFrameButtonPressed");
    switch (currentMode)
    {
   
    case ORBIT:
        addPosition();
        printKeyframeElements();
        break;
    case SLIDE:
        addPosition();
        printKeyframeElements();
        break;
    case TIMELAPSE:
        addPosition();
        printKeyframeElements();
        break;
    default:
        break;
    }
}

void Slider::onActionButtonPressed()
{
    Serial.println("OnActionButtonPressed");
    switch (currentMode)
    {
   
    case ORBIT:
        triggerCamera();
        break;
    case SLIDE:
        triggerCamera();
        break;
    case TIMELAPSE:
        triggerCamera();
        break;
    case SETTINGS:
        pairEOSCamera();
    default:
        break;
    }
}

void Slider::onLBButtonPressed()
{

    if (currentMode == TIMELAPSE)
    {
        if (timelapseSelectedtime > 0)
        {
            timelapseSelectedtime = (timelapseSelectedtime - 1);
        }
    }
    else
    {

        if (currentSpeedScale > 0.1)
        {
            currentSpeedScale -= 0.1;
            slideSpeed = MAX_SLIDE_SPEED * currentSpeedScale;
            panSpeed = MAX_PAN_SPEED * currentSpeedScale;
            stepper_pan.setMaxSpeed(panDegreesToSteps(panSpeed));
            stepper_slide.setMaxSpeed(sliderMillimetresToSteps(slideSpeed));
        }
    }
}

void Slider::onRBButtonPressed()
{

    if (currentMode == TIMELAPSE)
    {
        if (timelapseSelectedtime < 7)
        {
            timelapseSelectedtime = timelapseSelectedtime + 1;
        }
    }
    else
    {
        if (currentSpeedScale < 1)
        {
            currentSpeedScale += 0.1;
            slideSpeed = MAX_SLIDE_SPEED * currentSpeedScale;
            panSpeed = MAX_PAN_SPEED * currentSpeedScale;
            stepper_pan.setMaxSpeed(panDegreesToSteps(panSpeed));
            stepper_slide.setMaxSpeed(sliderMillimetresToSteps(slideSpeed));
        }
    }
}

void Slider::onLTButtonPressed()
{
    if (currentMode == TIMELAPSE)
    {
        if (timelapseShutterTime > 0)
        {
            timelapseShutterTime -- ;
        }
    }

}

void Slider::onRTButtonPressed(){
        if (currentMode == TIMELAPSE)
    {
        if (timelapseShutterTime <30 )
        {
            timelapseShutterTime ++;
        }
    }
}
void Slider::onUpButtonPressed(){
    if(currentMode == TIMELAPSE && timelapseInterval < 9 )
        timelapseInterval ++;
}

void Slider::onDnButtonPressed(){
    if(currentMode == TIMELAPSE && timelapseInterval > 0)
        timelapseInterval --;
}

uint32_t Slider::panDegreeToStep(float angle)
{
    return pan_steps_per_degree * angle;
}

void Slider::onAnalogStickChanged(byte LposX, byte LposY, byte RposX, byte RposY)
{

    // Remove deadzones
    if (abs(LposX - 128) <= ANALOG_DEADZONE)
    {
        LposX = 128;
    }
    if (abs(RposX - 128) <= ANALOG_DEADZONE)
    {
        RposX = 128;
    }

    currentAnalogStick.LposX = LposX;
    currentAnalogStick.LposY = LposY;
    currentAnalogStick.RposX = RposX;
    currentAnalogStick.RposY = RposY;
    // switch (currentMode)
    // {
    // case (MANUAL):

    //     currentAnalogStick.LposX = LposX;
    //     currentAnalogStick.LposY = LposY;
    //     currentAnalogStick.RposX = RposX;
    //     currentAnalogStick.RposY = RposY;

    //     break;
    // case (ORBIT):

    //     currentAnalogStick.LposX = LposX;
    //     currentAnalogStick.LposY = LposY;
    //     currentAnalogStick.RposX = RposX;
    //     currentAnalogStick.RposY = RposY;

    //     break;
    // }
}

float Slider::panDegreesToSteps(float angle)
{
    return pan_steps_per_degree * angle;
}

long Slider::sliderMillimetresToSteps(float mm)
{
    return round(mm * slider_steps_per_millimetre);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float Slider::sliderStepsToMillimetres(long steps)
{
    return (float)steps / slider_steps_per_millimetre;
}

void Slider::onNextModeButtonPressed()
{
    if (currentMode == SETTINGS)
        return;
    currentMode = (Mode)(currentMode + 1);
    Serial.printf("CURRENT MODE = %d\n", currentMode);
}

void Slider::onPreviousModeButtonPressed()
{
    if (currentMode == SLIDE)
        return;
    currentMode = (Mode)(currentMode - 1);
    Serial.printf("CURRENT MODE = %d\n", currentMode);
}

void Slider::interpolateTargetPoint(FloatCoordinate targetPoint, int repeat)
{ //The first two keyframes are interpolated between while keeping the camera pointing at previously calculated intercept point.
    if (keyframe_elements < 2)
    {
        printi(F("Not enough keyframes recorded\n"));
        return; //check there are posions to move to
    }

    float sliderStartPos = sliderStepsToMillimetres(keyframe_array[0].sliderStepCount); //slider start position
    float sliderEndPos = sliderStepsToMillimetres(keyframe_array[1].sliderStepCount);
    float panAngle = 0;
    float x = targetPoint.x - sliderStepsToMillimetres(keyframe_array[0].sliderStepCount);
    float ySqared = pow(targetPoint.y, 2);
    float sliderTravel = sliderStepsToMillimetres(keyframe_array[1].sliderStepCount) - sliderStepsToMillimetres(keyframe_array[0].sliderStepCount);
    int numberOfIncrements = abs(sliderTravel);
    float increment = sliderTravel / numberOfIncrements; //size of interpolation increments in mm

    autoMoveRunning = true;
    updateScreen(true);

    //Go to 1st position
    setTargetPositions(panStepsToDegrees(keyframe_array[0].panStepCount), sliderStartPos);
    multiStepper.runSpeedToPosition(); //blocking move to the next position
       if (canonBLERemote.isConnected()){
                triggerCamera();
            }
    delay(1000);

    Serial.println(numberOfIncrements);
    for (int j = 0; (j < repeat || (repeat == 0 && j == 0)); j++)
    {

        for (int i = 0; i <= numberOfIncrements; i++)
        {

            x = targetPoint.x - (sliderStartPos + increment * i);
            panAngle = radsToDeg(atan2(targetPoint.y, x));
            setTargetPositions(panAngle, sliderStartPos + increment * i);
            multiStepper.runSpeedToPosition(); //blocking move to the next position
        }
        delay(1000);
        x = targetPoint.x - sliderEndPos;
        panAngle = radsToDeg(atan2(targetPoint.y, x));
        setTargetPositions(panAngle, sliderEndPos);
        multiStepper.runSpeedToPosition(); //blocking move to the next position

        delay(1000);

        for (int i = numberOfIncrements; (i >= 0 && repeat > 0); i--)
        {
            x = targetPoint.x - (sliderStartPos + increment * i);
            panAngle = radsToDeg(atan2(targetPoint.y, x));
            setTargetPositions(panAngle, sliderStartPos + increment * i);
            multiStepper.runSpeedToPosition(); //blocking move to the next position
        }
        if (repeat > 0)
        {
            delay(1000);
            setTargetPositions(panAngle, sliderStartPos);
            multiStepper.runSpeedToPosition(); //blocking move to the next position
        }
    }

       if (canonBLERemote.isConnected()){
                triggerCamera();
            }

    autoMoveRunning = false;
}

int Slider::setTargetPositions(float panDeg, float sliderMillimetre)
{
    target_position[0] = sliderMillimetresToSteps(sliderMillimetre);
    target_position[1] = panDegreesToSteps(panDeg);
    multiStepper.moveTo(target_position);
}

int Slider::addPosition(void)
{
    if (keyframe_elements >= 0 && keyframe_elements < KEYFRAME_ARRAY_LENGTH)
    {
        keyframe_array[keyframe_elements].panStepCount = stepper_pan.currentPosition();
        keyframe_array[keyframe_elements].sliderStepCount = stepper_slide.currentPosition();
        keyframe_array[keyframe_elements].panSpeed = panSpeed;
        keyframe_array[keyframe_elements].sliderSpeed = slideSpeed;
        keyframe_array[keyframe_elements].msDelay = 0;
        current_keyframe_index = keyframe_elements;
        keyframe_elements++; //increment the index
        Serial.print("Added at index: " + String(current_keyframe_index));
        return 0;
    }
    else
    {
        Serial.println("Max number of keyframes reached");
    }
    return -1;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void Slider::clearKeyframes(void)
{
    keyframe_elements = 0;
    current_keyframe_index = -1;
    Serial.println("Keyframes cleared");
}

void Slider::printKeyframeElements(void)
{
    printi(F("Keyframe index: "), current_keyframe_index, F("\n"));
    for (int row = 0; row < keyframe_elements; row++)
    {
        printi(F(""), row, F("\t|"));
        printi(F(" Pan: "), panStepsToDegrees(keyframe_array[row].panStepCount), 3, F("º\t"));
        // printi(F("Tilt: "), tiltStepsToDegrees(keyframe_array[row].tiltStepCount), 3, F("º\t"));
        printi(F("Slider: "), sliderStepsToMillimetres(keyframe_array[row].sliderStepCount), 3, F("mm\t"));
        printi(F("Pan Speed: "), keyframe_array[row].panSpeed, 3, F(" º/s\t"));
        printi(F("Slider Speed: "), keyframe_array[row].sliderSpeed, 3, F(" mm/s\t"));
        printi(F("Delay: "), keyframe_array[row].msDelay, F("ms |\n"));
    }
    printi(F("\n"));
}

void Slider::doSlide()
{
    if (keyframe_elements < 2)
    {
        printi(F("Not enough keyframes recorded\n"));
        return; //check there are posions to move to
    }

    float sliderStartPos = sliderStepsToMillimetres(keyframe_array[0].sliderStepCount); //slider start position
    float sliderStartAngle = panStepsToDegrees(keyframe_array[0].panStepCount);
    float sliderEndPos = sliderStepsToMillimetres(keyframe_array[1].sliderStepCount);
    float sliderEndAngle = panStepsToDegrees(keyframe_array[1].panStepCount);
    float panAngle = 0;
    float sliderTravel = sliderStepsToMillimetres(keyframe_array[1].sliderStepCount) - sliderStepsToMillimetres(keyframe_array[0].sliderStepCount);

    //Go to start position
    Serial.println("Going to first frame");
    setTargetPositions(sliderStartAngle, sliderStartPos);
    multiStepper.runSpeedToPosition(); //blocking move to the next position
    initStepper();                     // Somehow AccelStepper is not working after using MultiStepper, so re-init is required.
    stepper_slide.setAcceleration(sliderMillimetresToSteps(sliderTravel / 20));

    if (canonBLERemote.isConnected()){
        triggerCamera();
    }
    autoMoveRunning = true;
    updateScreen(true);
}

void Slider::doTimelapse()
{
    if (keyframe_elements < 2)
    {
        printi(F("Not enough keyframes recorded\n"));
        return; //check there are posions to move to
    }

    float sliderStartPos = sliderStepsToMillimetres(keyframe_array[0].sliderStepCount); //slider start position
    float sliderStartAngle = panStepsToDegrees(keyframe_array[0].panStepCount);
    float sliderEndPos = sliderStepsToMillimetres(keyframe_array[1].sliderStepCount);
    float sliderEndAngle = panStepsToDegrees(keyframe_array[1].panStepCount);
    float panAngle = 0;
    float sliderTravel = sliderStepsToMillimetres(keyframe_array[1].sliderStepCount) - sliderStepsToMillimetres(keyframe_array[0].sliderStepCount);

    //Go to start position
    Serial.println("Going to first frame");
    setTargetPositions(sliderStartAngle, sliderStartPos);
    multiStepper.runSpeedToPosition(); //blocking move to the next position

    float totalMoveTime = timelapseDuration[timelapseSelectedtime] * 60.0;  //Second
    float totalCapturingTime = 0;                                           //Second
    int totalShots = 0;

    if(timelapseInterval){
        totalShots = totalMoveTime / (((TRIGGER_MS+timelapseShutterTime)/1000.0) + timelapseInterval) ;
        totalCapturingTime = totalShots * ((TRIGGER_MS+timelapseShutterTime)/1000.0);
        totalMoveTime = totalMoveTime - totalCapturingTime;
    }

    Serial.println("Total move time: "+ String(totalMoveTime));
    Serial.println("Total capture time: "+ String(totalCapturingTime));
    Serial.println("Total totalShots: "+ String(totalShots));

    float moveSpeed = (keyframe_array[1].sliderStepCount - keyframe_array[0].sliderStepCount) / totalMoveTime;
    stepper_slide.setMaxSpeed(moveSpeed);
    setTargetPositions(sliderEndAngle, sliderEndPos);
    autoMoveRunning = true;
    updateScreen();
    delay(1000);
    //Start recording
}

void Slider::onStartButtonPressed()
{

    switch (currentMode)
    {
    case ORBIT:
        if (calculateTargetCoordinate())
        {
            interpolateTargetPoint(intercept, 1);
        }
        break;
    case SLIDE:
        doSlide();
        break;
    case TIMELAPSE:
        doTimelapse();
        break;
    default:
        break;
    }
}

bool Slider::calculateTargetCoordinate(void)
{
    float m1, c1, m2, c2;

    LinePoints line0;
    line0.x0 = sliderStepsToMillimetres(keyframe_array[0].sliderStepCount);
    line0.y0 = 0;
    line0.x1 = line0.x0 + cos(degToRads(panStepsToDegrees(keyframe_array[0].panStepCount)));
    line0.y1 = sin(degToRads(panStepsToDegrees(keyframe_array[0].panStepCount)));

    LinePoints line1;
    line1.x0 = sliderStepsToMillimetres(keyframe_array[1].sliderStepCount);
    line1.y0 = 0;
    line1.x1 = line1.x0 + cos(degToRads(panStepsToDegrees(keyframe_array[1].panStepCount)));
    line1.y1 = sin(degToRads(panStepsToDegrees(keyframe_array[1].panStepCount)));

    if ((line0.x1 - line0.x0) != 0)
    {
        m1 = (line0.y1 - line0.y0) / (line0.x1 - line0.x0);
        c1 = line0.y1 - m1 * line0.x1;
    }

    if ((line1.x1 - line1.x0) != 0)
    {
        m2 = (line1.y1 - line1.y0) / (line1.x1 - line1.x0);
        c2 = line1.y1 - m2 * line1.x1;
    }

    if ((line0.x1 - line0.x0) == 0)
    {
        intercept.x = line0.x0;
        intercept.y = m2 * intercept.x + c2;
    }
    else if ((line1.x1 - line1.x0) == 0)
    {
        intercept.x = line1.x0;
        intercept.y = m1 * intercept.x + c1;
    }
    else
    {
        if (m1 == m2)
        { //If the angle of the slope of both lines are the same they are parallel and cannot intercept.
            printi(F("Positions do not intersect."));
            return false;
        }
        intercept.x = (c2 - c1) / (m1 - m2);
        intercept.y = m1 * intercept.x + c1;
    }

    if (((panStepsToDegrees(keyframe_array[0].panStepCount) > 0 && panStepsToDegrees(keyframe_array[1].panStepCount) > 0) && intercept.y < 0) || ((panStepsToDegrees(keyframe_array[0].panStepCount) < 0 && panStepsToDegrees(keyframe_array[1].panStepCount) < 0) && intercept.y > 0) || intercept.y == 0)
    { //Checks that the intercept point is in the direction the camera was pointing and not on the opposite side behind the camera.
        printi(F("Invalid intercept.\n"));
        return false;
    }
    return true;
}

void Slider::goToTargetPosition(float panDeg, float sliderMillimetre, bool isBlocking = true)
{
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float Slider::panStepsToDegrees(long steps)
{
    return steps / pan_steps_per_degree;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void Slider::onBackButtonPressed()
{
    switch (currentMode)
    {
    case (ORBIT):
        clearKeyframes();
        break;
    case (SLIDE):
        if (autoMoveRunning)
        {
            Serial.print("Stop move");
            autoMoveRunning = false;
            stepper_slide.setSpeed(0);
            stepper_slide.runSpeed();
            if (canonBLERemote.isConnected()){
                triggerCamera();
            }
        }
        else
        {
            clearKeyframes();
        }

        break;
    case TIMELAPSE:
        if (autoMoveRunning)
        {
            Serial.print("Stop move");
            autoMoveRunning = false;
            stepper_slide.setSpeed(0);
            stepper_slide.runSpeed();
            stepper_slide.setMaxSpeed(sliderMillimetresToSteps(slideSpeed));
        }
        else
        {
            clearKeyframes();
        }
    default:
        break;
    }
}

void Slider::disableAllMotors(bool disable)
{
    if (disable)
    {
        digitalWrite(PIN_MOTOR_EN, HIGH);
    }
    else
    {
        digitalWrite(PIN_MOTOR_EN, LOW);
    }
}

void Slider::triggerCamera(){

    if (useEOSRemote){
        canonBLERemote.trigger();
    }
    else{
        digitalWrite(PIN_AUX_IO_1, HIGH);
        delay(TRIGGER_MS);
        digitalWrite(PIN_AUX_IO_1, LOW);

        if (currentMode == TIMELAPSE){
            delay(timelapseShutterTime);
        }
    }

}

void Slider::run()
{

    isMoving = false;

    if (autoMoveRunning == false)
    {
        float currentSlideSpeed = map(abs(currentAnalogStick.LposX - 128), 0, 128, 0, slideSpeed);
        float currentPanSpeed = map(abs(currentAnalogStick.RposX - 128), 0, 128, 0, panSpeed);

        if (currentAnalogStick.LposX > 128 + ANALOG_DEADZONE)
        {
            stepper_slide.moveTo(stepper_slide.currentPosition() + 1);
            stepper_slide.setSpeed(sliderMillimetresToSteps(currentSlideSpeed));
            isMoving = true;
        }
        else if (currentAnalogStick.LposX < 128 - ANALOG_DEADZONE)
        {
            stepper_slide.moveTo(stepper_slide.currentPosition() - 1);
            stepper_slide.setSpeed(-sliderMillimetresToSteps(currentSlideSpeed));
            isMoving = true;
        }

        if (currentAnalogStick.RposX > 128 + ANALOG_DEADZONE)
        {
            stepper_pan.moveTo(stepper_pan.currentPosition() + 1);
            stepper_pan.setSpeed(panDegreesToSteps(currentPanSpeed));
            isMoving = true;
        }
        else if (currentAnalogStick.RposX < 128 - ANALOG_DEADZONE)
        {
            stepper_pan.moveTo(stepper_pan.currentPosition() - 1);
            stepper_pan.setSpeed(-panDegreesToSteps(currentPanSpeed));
            isMoving = true;
        }

        multiStepper.run();
    }
    else if (autoMoveRunning == true)
    {
        if (currentMode == SLIDE)
        {
            if (stepper_slide.targetPosition() == keyframe_array[0].sliderStepCount && stepper_slide.distanceToGo() == 0)
            {
                stepper_slide.moveTo(keyframe_array[1].sliderStepCount);
            }
            if (stepper_slide.targetPosition() == keyframe_array[1].sliderStepCount && stepper_slide.distanceToGo() == 0)
            {
                stepper_slide.moveTo(keyframe_array[0].sliderStepCount);
            }
            stepper_slide.run();
            isMoving = true;
        }

        if (currentMode == TIMELAPSE)
        {
            if (stepper_slide.distanceToGo() != 0)
            {
                multiStepper.run();
                if (millis() - lastShotTime > timelapseInterval * 1000 && timelapseInterval){
                    triggerCamera();
                    lastShotTime = millis();
                }
            }
            else
            {
                autoMoveRunning = false;
                stepper_slide.setMaxSpeed(sliderMillimetresToSteps(slideSpeed));
            }
        }
    }
}

void Slider::updateScreen(bool forceUpdate)
{
    static int progress = 0;
    static Mode lastMode ;

    if (millis() - lastScreenUpdate > screenUpdateMills || forceUpdate)
    {
        // Screen updating logic when auto program is not running
        if (!isMoving && !autoMoveRunning)
        {
            
            if (lastMode == NULL){
                lcd.clear();
            }
            else if (lastMode != currentMode){
                lcd.clear();
            }
            lastMode = currentMode;

            lcd.setCursor(0, 0);
            lcd.printf("%-9s  ", modeString[currentMode]);

            lcd.setCursor(15, 0);
            if (canonBLERemote.isConnected()){
                lcd.write(0);
            }else{
                lcd.print(" ");
            }
    

            if (currentMode == ORBIT || currentMode == SLIDE)
            {
                lcd.setCursor(0, 1);
                lcd.printf("speed:%-2.f   ", 10.0 * currentSpeedScale);

                
                lcd.setCursor(11, 1);
                if (current_keyframe_index == -1)
                {
                    lcd.printf("   IN", keyframe_elements);
                }
                else if (current_keyframe_index == 0)
                {
                    lcd.printf("  OUT", keyframe_elements);
                }
                else if (current_keyframe_index == 1)
                {
                    if (currentMode == ORBIT && !calculateTargetCoordinate()){
                        lcd.printf("ERROR");
                    }
                    lcd.printf("READY");
                }
            }
            else if (currentMode == TIMELAPSE)
            {
                lcd.setCursor(11, 0);
                // const char* time = "123m";
                // sprintf("%dm",time,timelapseDuration[timelapseSelectedtime]);
                lcd.printf("I:%d", timelapseInterval);
                lcd.setCursor(0, 1);
                lcd.printf("T:%dm S:%-2d", timelapseDuration[timelapseSelectedtime],  timelapseShutterTime);

                lcd.setCursor(11, 1);
                if (current_keyframe_index == -1)
                {
                    lcd.printf("   IN");
                }
                else if (current_keyframe_index == 0)
                {
                    lcd.printf("  OUT");
                }
                else if (current_keyframe_index == 1)
                {
                    lcd.printf("READY");
                }
            }else if (currentMode == SETTINGS){
                lcd.setCursor(0,1);
                lcd.print("Pair EOS ");

                lcd.setCursor(9,1);
                switch (eosPairingProcess)
                {
                case 0:
                    lcd.print("   PAIR");
                    break;
                case 1:
                    lcd.print("PAIRING");
                    break;
                case 2:
                    lcd.print(" PAIRED");
                    break;
                case 3:
                    lcd.print(" FAILED");
                    break;
                default:
                    break;
                }
            }
        }

        if (autoMoveRunning)
        {
            if(currentMode == TIMELAPSE){



                if (stepper_slide.distanceToGo() != 0)
                {
                    int newProgress = 100 - (float(abs(stepper_slide.distanceToGo())) / abs(keyframe_array[1].sliderStepCount - keyframe_array[0].sliderStepCount)) * 100;
                    bool firstMove = abs(stepper_slide.distanceToGo()) == abs(keyframe_array[1].sliderStepCount - keyframe_array[0].sliderStepCount);
                    if (newProgress == progress && !firstMove)
                        return;

                    if (firstMove)
                    {
                        lcd.setCursor(11, 0);
                        lcd.printf("     ");
                        lcd.setCursor(0, 1);
                        lcd.printf("RUNNING..   ");
                    }
                    lcd.setCursor(12, 1);
                    lcd.printf("%3d", newProgress);
                    lcd.setCursor(15, 1);
                    lcd.print("%");
                    progress = newProgress;
                }
            }
            else if (forceUpdate){
                lcd.setCursor(0, 1);
                lcd.printf("RUNNING..       ");
            }

        }
        lastScreenUpdate = millis();
    }
}

void Slider::onSetHomeButtonPressed()
{
    Serial.println("Set Home");
    stepper_pan.setCurrentPosition(-panDegreesToSteps(90));
}
