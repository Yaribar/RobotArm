/*
  ESP32 MicroRobotArm
  Author: T-Kuhn
  Modified by: Yaribar
 */

#include <Arduino.h>
#include "Constants.h"
#include "SineStepper.h"
#include "SineStepperController.h"
#include "Queue.h"
#include "MoveBatch.h"
#include "RobotArmIK.h"
#include "Encoder.h"
#include "BluetoothSerial.h"
#include "Useful_Methods.h"
#include "ROBOT.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;


enum Mode
{
    adjustingJoint1,
    adjustingJoint2,
    adjustingJoint3,
    adjustingJoint4,
    adjustingJoint5,
    doingControlledMovements,
    error
};

float  angleBaseJoint;
float  angleJoint1;
float  angleJoint2;
float  angleJoint3;
float  angleJoint4;
float  angleJoint5;
float  inverseType;

float desired_position[4];
float angles[4];

float x,roll, y, pitch, z, yaw;

Mode currentMode = doingControlledMovements;

SineStepper sineStepper1(STEPPER1_STEP_PIN, STEPPER1_DIR_PIN, /*id:*/ 0);
SineStepper sineStepper2(STEPPER2_STEP_PIN, STEPPER2_DIR_PIN, /*id:*/ 1);
SineStepper sineStepper3(STEPPER3_STEP_PIN, STEPPER3_DIR_PIN, /*id:*/ 2);
SineStepper sineStepper4(STEPPER4_STEP_PIN, STEPPER4_DIR_PIN, /*id:*/ 3);
SineStepper sineStepper5(STEPPERBASE_STEP_PIN, STEPPERBASE_DIR_PIN, /*id:*/ 4);
SineStepperController sineStepperController(/*endlessRepeat:*/ true);


MoveBatch mb;

int buttonCoolDownCounter = 0;

volatile uint16_t pos=0;
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile SemaphoreHandle_t timerSemaphore;

String input;
float data[10];

ROBOT IK;

void handleModeChange(Mode newMode)
{
    if (buttonCoolDownCounter < BUTTON_COOLDOWN_CYCLES)
    {
        buttonCoolDownCounter++;
    }
    if (digitalRead(BUTTON_PIN) && buttonCoolDownCounter >= BUTTON_COOLDOWN_CYCLES)
    {
        buttonCoolDownCounter = 0;
        currentMode = newMode;
        portENTER_CRITICAL(&timerMux);
        Serial.printf("%d\n",currentMode);
        portEXIT_CRITICAL(&timerMux);
    }
}

void IRAM_ATTR onTimer()
{
    digitalWrite(EXECUTING_ISR_CODE, HIGH);
    pos = round(analogRead(POT) / 90);

    switch (currentMode)
    {
    case adjustingJoint1:
        digitalWrite(STEPPER1_DIR_PIN, digitalRead(BUTTONDIR));
        digitalWrite(STEPPER1_STEP_PIN, pos%2);
        handleModeChange(adjustingJoint2);
        break;
    case adjustingJoint2:
        digitalWrite(STEPPER2_DIR_PIN, digitalRead(BUTTONDIR));
        digitalWrite(STEPPER2_STEP_PIN, pos%2);
        handleModeChange(adjustingJoint3);
        break;
    case adjustingJoint3:
        digitalWrite(STEPPER3_DIR_PIN, digitalRead(BUTTONDIR));
        digitalWrite(STEPPER3_STEP_PIN, pos%2);
        handleModeChange(adjustingJoint4);
        break;
    case adjustingJoint4:
        digitalWrite(STEPPER4_DIR_PIN, digitalRead(BUTTONDIR));
        digitalWrite(STEPPER4_STEP_PIN, pos%2);
        handleModeChange(adjustingJoint5);
        break;
    case adjustingJoint5:
        digitalWrite(STEPPERBASE_DIR_PIN, digitalRead(BUTTONDIR));
        digitalWrite(STEPPERBASE_STEP_PIN, pos%2);
        handleModeChange(doingControlledMovements);
        break;
    case doingControlledMovements:
        portENTER_CRITICAL_ISR(&timerMux);
        sineStepperController.update();
        handleModeChange(adjustingJoint1);
        portEXIT_CRITICAL_ISR(&timerMux);
        break;
    default:
        break;
    }
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
    digitalWrite(EXECUTING_ISR_CODE, LOW);
}

void pruebaYarib(MoveBatch mb){
    mb.addMove(/*id:*/ 0, /*pos:*/(int32_t)(200* 1.5 / (2* M_PI)));
    mb.addMove(/*id:*/ 1, /*pos:*/(int32_t)(PULSES_PER_REVOLUTION * 1.5/ M_PI));
    mb.addMove(/*id:*/ 2, /*pos:*/(int32_t)(PULSES_PER_REVOLUTION * 1.5 / M_PI));
    mb.addMove(/*id:*/ 3, /*pos:*/(int32_t)(PULSES_PER_REVOLUTION * 1.5 / M_PI));
    mb.addMove(/*id:*/ 4, /*pos:*/(int32_t)(200 * 1.5 / M_PI));
    sineStepperController.addMoveBatch(mb);

    mb.addMove(/*id:*/ 0, /*pos:*/(int32_t)(200 * 0 / M_PI));
    mb.addMove(/*id:*/ 1, /*pos:*/(int32_t)(PULSES_PER_REVOLUTION * 0 / M_PI));
    mb.addMove(/*id:*/ 2, /*pos:*/(int32_t)(PULSES_PER_REVOLUTION * 0 / M_PI));
    mb.addMove(/*id:*/ 3, /*pos:*/(int32_t)(PULSES_PER_REVOLUTION * 0 / M_PI));
    mb.addMove(/*id:*/ 4, /*pos:*/(int32_t)(200 * 0 / M_PI));
    sineStepperController.addMoveBatch(mb);
}


void setup()
{
    Serial.begin(115200);
    SerialBT.begin("ESP32_RoboticArm_test"); //Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");

    pinMode(EXECUTING_ISR_CODE, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(BUTTONDIR, INPUT);
    float linkSize[] = {6,7,6};
    IK.setupLinks(linkSize);

    timerSemaphore = xSemaphoreCreateBinary();
    sineStepperController.attach(&sineStepper1);
    sineStepperController.attach(&sineStepper2);
    sineStepperController.attach(&sineStepper3);
    sineStepperController.attach(&sineStepper4);
    sineStepperController.attach(&sineStepper5);

    // initialize MoveBatches
    
    mb.moveDuration = 1.75;

    Serial.printf("Initialization Completed! \n");

    pruebaYarib(mb);

    // Set 80 divider for prescaler
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    // onTimer gets called every 100uS.
    timerAlarmWrite(timer, 100, true);
    timerAlarmEnable(timer);
}

void loop()
{
    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE)
    {
        int32_t pos = 0;
        if (SerialBT.available()){
            portENTER_CRITICAL(&timerMux);
            input = SerialBT.readStringUntil('\n');
            Serial.println(input);
            parseString(input, ",", data);

            inverseType =   data[0];

            
            //modalidad: 0 forward; angles base, joint1, joint2 , joint3, joint4, joint5
            //modalidad: 1 inverse; x,roll, y, pitch, z, yaw
            if(inverseType == 0){
                angleBaseJoint   =   data[1];
                angleJoint1 =   data[2];
                angleJoint2 =   data[3];
                angleJoint3 =   data[4];
                angleJoint4 =   data[5];
                angleJoint5 =   data[6];
                
                mb.addMove(/*id:*/ 0, /*pos:*/(int32_t)(200* angleJoint1 / M_PI));
                mb.addMove(/*id:*/ 1, /*pos:*/(int32_t)(PULSES_PER_REVOLUTION * angleJoint2 / M_PI));
                mb.addMove(/*id:*/ 2, /*pos:*/(int32_t)(PULSES_PER_REVOLUTION * angleJoint3 / M_PI));
                mb.addMove(/*id:*/ 3, /*pos:*/(int32_t)(PULSES_PER_REVOLUTION * angleJoint4 / M_PI));
                mb.addMove(/*id:*/ 4, /*pos:*/(int32_t)(200 * angleBaseJoint / M_PI));
                sineStepperController.addMoveBatch(mb);
            }
            if(inverseType == 1){
                //modalidad: 1 inverse; x,roll, y, pitch, z, yaw
                x       =   data[1];
                roll    =   data[2];
                y       =   data[3];
                pitch   =   data[4];
                z       =   data[5];
                yaw     =   data[6];

                desired_position[0] = x;
                desired_position[1] = y;
                desired_position[2] = z;
                desired_position[3] = roll;

                IK.inverseKinematics(desired_position,angles);
                
                mb.addMove(/*id:*/ 0, /*pos:*/(int32_t)(200* angles[0] / M_PI));
                mb.addMove(/*id:*/ 1, /*pos:*/(int32_t)(PULSES_PER_REVOLUTION * angles[1] / M_PI));
                mb.addMove(/*id:*/ 2, /*pos:*/(int32_t)(PULSES_PER_REVOLUTION * angles[2] / M_PI));
                mb.addMove(/*id:*/ 3, /*pos:*/(int32_t)(PULSES_PER_REVOLUTION * 0 / M_PI));
                mb.addMove(/*id:*/ 4, /*pos:*/(int32_t)(200 * angles[3] / M_PI));
                sineStepperController.addMoveBatch(mb);
            }
            portEXIT_CRITICAL(&timerMux);
        }
        
        delay(10);
    }
}
