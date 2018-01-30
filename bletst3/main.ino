/*********************************************************************
 * Based on BLUEUart for Feather nRF52 by Adafruit 
 * 
 * Main file
 * 
 * by:
 * Dawid Borowczak
 * Michał Kłos
 * Adam Neubauer
 * Wiktor Siwek
 * 
*********************************************************************/
#define ARM_MATH_CM4

#include <bluefruit.h>
#include <arm_math.h>


#define sensorLeft A0
#define SensorRight A1

/* 
 * Actuators pins declarations:
 */

// enable pins (PWMs)
#define enableLeft 14   // PWM signal left motor
#define enableRight 13  // PWM signal right motor

// left motor direction select pins
#define dirLeftA 15
#define dirLeftB 16

// right motor direction select pins
#define dirRightA 12
#define dirRightB 11

/******************************
 * Encoders pins declarations
 * ***************************/

// encoders output pins
#define encoderLeft A2
#define encoderRight A3

// encoder wheel reesolution
#define encoderResolution 32

/******************************
 * Encoders variables
 * ***************************/

// encoder ticks
int encoderLeftTicks = 0;
int encoderRightTicks = 0;

// number of wheel revs
int encoderLeftRevs = 0;
int encoderRightRevs = 0;

// previous encoders values
bool encoderLeftLast = 0;
bool encoderRightLast = 0;

bool encoderLeftCurr = 0;
bool encoderRightCurr = 0;

int encCurrLeftCount; 
int encCurrRightCount;

// ref voltage for ADC
//float mv_per_lsb = 3000.0f/256.0f; // 8-bit ADC z 3.3V input range

/******************************
 * Control variables and values
 * ***************************/

 // maximal control values
const uint8_t maxValue = bit(8)-1;
const uint8_t maxLeftValue = 190;

uint8_t leftWheelControl = maxLeftValue;
uint8_t rightWheelControl = maxValue;

bool leftWheelDirection = true;
bool rightWheelDirection = true;

// straight ride flag
bool straightF = 0;

bool manualControl = true; // control mode variable false - automode; true - manualmode

/******************************
 * Neural network variables
 * ***************************/

uint32_t InputVal[2];
float OutputVal[4];
float sum;

unsigned long t3;

uint8_t inputLayer;
uint8_t hiddenLayer;
uint8_t outputLayer;

float inputWeight[3][8];
float hiddenWeight[9][4];

float inputNeuron[2];
float hiddenNeuron[8];
float outputNeuron[4];

uint8_t omegaLeft[1];
uint8_t omegaRight = 0;

/******************************
 * Timers & flags
 * ***************************/

// phase flags
bool f1 = false;
bool f2 = false;

// timer snaps
unsigned long t1 = 0;
unsigned long t2 = 0;

unsigned long t0;
bool F0;

unsigned long encTimer;

float speedLeft;
float speedRight;

// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;

SoftwareTimer readEncTimer;

/******************************
 * Robot
 * ***************************/

float r = 0.01525; // wheel radius
float pi = 3.1415; // pi

float d = 0.0515; // wheel raster

float wl = 0.0;
float wp = 0.0;

float x = 0.0;
float y = 0.0;
float phi = 0.0;

float vel = 0.0;
float omg = 0.0;

float vx = 0.0;
float vy = 0.0;

float tp = 1.0;

bool Fr = true;

int ctr = 0;

/******************************
 * Bluetooth services
 * ***************************/

union {
    float val;
    unsigned char b[4];
} omgLeftToSend;

union {
    float val;
    unsigned char b[4];
} omgRightToSend;

char speLeft[6];
char speRight[6];

// BLE Service
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;


void setup()
{
    analogReadResolution(12);
    analogReference(AR_INTERNAL_3_0);

    omegaLeft[0] = 1;

    
    pinMode(dirLeftA, OUTPUT);
    pinMode(dirLeftB, OUTPUT);

    pinMode(dirRightA, OUTPUT);
    pinMode(dirRightB, OUTPUT);

    setupPWM();

    delay(1);


    F0 = true;
    t0 = millis() - 50;
    t3 = millis() -200;
    
    encoderLeftLast = 0;
    encoderRightLast = 0;
    encoderLeftCurr = 0;
    encoderRightCurr = 0;

    Serial.begin(115200);
    Serial.println("bug_board_v1.0 OS initialize...");
    Serial.println("-------------------------------\n");

    // to myślę że można wywalić ale pewny nie jestem
    // Initialize blinkTimer for 1000 ms and start it
    blinkTimer.begin(1000, blink_timer_callback);
    blinkTimer.start();
/*
    readEncTimer.begin(50, readEnc_Timer_callback);
    readEncTimer.start();
*/

    // Setup the BLE LED to be enabled on CONNECT
    Bluefruit.autoConnLed(true); // to zostawiamy

    Bluefruit.begin();
    Bluefruit.setTxPower(4); // maksymalna moc nadajnika
    Bluefruit.setName("guziec");
    
    //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
    Bluefruit.setConnectCallback(connect_callback);
    Bluefruit.setDisconnectCallback(disconnect_callback);

    // Configure and Start Device Information Service
    bledis.setManufacturer("BKNS");
    bledis.setModel("bug_board_v1.0");
    bledis.begin();

    // Configure and Start BLE Uart Service
    bleuart.begin();

    // Start BLE Battery Service
    blebas.begin();
    blebas.write(100);

    // Set up and start advertising
    startAdv();

    setupNeuralNet();

    //GoStraight(true, 192, 255);

    Serial.println("feel free to use BugApp to control BugBoard via BLE");
    Serial.println("You can send values to the app to test connectivity");
}

void setupPWM()
{
    // initialize PWM module
    HwPWMx[0]->addPin(enableLeft);
    HwPWMx[1]->addPin(enableRight);

    HwPWM0.begin();
    HwPWM0.setResolution(8); // max value 255
    HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_16);

    HwPWM1.begin();
    HwPWM1.setResolution(8); // max value 255
    HwPWM1.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_16);
}

void loop()
{

    // Encoders reading
    if(millis()- t0 > 10)
    {
        readEncoders();

        //receiveData();

        t0 = millis();
    }
    
  

    // wheels speeds computation
    unsigned long encMili = millis();
    int timeSpan = encMili - encTimer;

    if(timeSpan >= 1000)
    {
        // 360st/32prążki
        // 11,25 stopni to prążek
        float ttt = (float)timeSpan / 1000.0;

        //float encCurrLeft = (float)abs(encCurrLeftCount);
        float encCurrLeft = (float)encCurrLeftCount;
        float radLeft = (encCurrLeft * 11.25) * (3.1415 / 180);
        speedLeft = radLeft / ttt;
        Serial.print("left speed - ");
        Serial.println(speedLeft);
        Serial.println(encCurrLeft);
        //speedLeft = radLeft / ((timeSpan) / 1000);
        encCurrLeftCount = 0;

        
        //float encCurrRight = (float)abs(encCurrRightCount);
        float encCurrRight = (float)encCurrRightCount;
        float radRight = (encCurrRight * 11.25) * (3.1415 / 180);
        speedRight = radRight / ttt;
        Serial.print("right speed - ");
        Serial.println(speedRight);
        Serial.println(encCurrRight);
        //speedRight = radRight / ((timeSpan) / 1000);
        encCurrRightCount = 0;
        encTimer = millis();
        ctr++;

        // wheeel contro value Regulator
        //Regulator();

        if(ctr>= 5)
        {
            Serial.print("Left ticks : ");
            Serial.println(encoderLeftTicks);
            Serial.print("Right ticks : ");
            Serial.println(encoderRightTicks);
            ctr = 0;
        }

        wl = speedLeft;
        wp = speedRight;

        omgLeftToSend.val = speedLeft;
        omgRightToSend.val = speedRight;

        sprintf(speLeft, "%f", abs(speedLeft));
        sprintf(speRight, "%f", abs(speedRight));
        


        // zrzutować prędkości przy jeździe na wprost

        sendData();

        Velocity();
        Omega();
        Vxn();
        Vyn();
        Xn();
        Yn();
        Phin();

        Serial.print("X : ");
        Serial.print(x);
        Serial.print(" __ Y : ");
        Serial.println(y);
        Serial.print("phi : ");
        Serial.println(phi);


    }

    
    // Neural network control //
    if(!manualControl)
    {
        if (bleuart.available() )
        {
            CancelControl();
            receiveData();
            
        }

        //if()
        if(F0)
        {
            if(millis() - t3 > 200)
            {
                CalculateNet();
                t3 = millis();

            }
        
        }
        else
        {
            //Serial.println("************BACKWARD**************");
            if (millis() - t3 < 1500)
            {
                GoStraight(false, maxLeftValue,  maxValue);
            }
            else
            {
                F0 = true;
            }
        }
    }
    else
    {

        receiveData();

        // here manual control driver
    }
    
    /********************************************************************
     * TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST
     * *****************************************************************/

    /*
    if(Fr)
    {
        GoStraight(true, leftWheelControl , rightWheelControl);
        leftWheelDirection = true;
        rightWheelDirection = true;

        if(x >= 1.0)
        {
            Fr = false;
            
        }
    }
    else
    {
        GoStraight(false, leftWheelControl + 20, rightWheelControl);
        leftWheelDirection = false;
        rightWheelDirection = false;

        if(x <= 0.0)
        {
            Fr = true;
        }
    }
    
*/


    // Request CPU to enter low-power mode until an event/interrupt occurs
    waitForEvent();
}


void Velocity()
{
    //vel = (abs(wp + wl) * r)  / d;
    vel = ((wp + wl) * r)  / d;
}

void Omega()
{   
    //omg = (abs(wp - wl) * r) / d;   
    omg = ((wp - wl) * r) / d;   
}

void Vxn()
{
    //vx = ((abs(wp + wl) * r) / 2) * cos(phi);
    vx = (((wp + wl) * r) / 2) * cos(phi);
}

void Vyn()
{
    //vy = ((abs(wp + wl) * r) / 2) * sin(phi);
    vy = (((wp + wl) * r) / 2) * sin(phi);
}

void Xn()
{
    x = x + vx * tp;
}

void Yn()
{
    y = y + vy * tp;
}

void Phin()
{
    phi = phi + omg * tp;
}


void blink_timer_callback(TimerHandle_t xTimerID)
{
    
//bleuart.write(omegaLeft, sizeof(omegaLeft));
    //omegaLeft[0]++;
    (void) xTimerID;
    digitalToggle(LED_RED);
}

void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
}