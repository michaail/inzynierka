
/*********************************************************************
 * Based on BLUEUart for Feather nRF52 by Adafruit 
 * 
 * Sketch is designed to receive data as Peripheral
 * from BugApp for Android
 * 
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

/*
 * Encoders pins declarations
 */

// encoders output pins
#define encoderLeft A2
#define encoderRight A3

// encoder wheel reesolution
#define encoderResolution 32

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

// ref voltage for ADC
//float mv_per_lsb = 3000.0f/256.0f; // 8-bit ADC z 3.3V input range

const uint8_t maxValue = bit(8)-1;
const uint8_t maxLeftValue = 192;
int ctr = 0;

uint8_t leftWheelControl = maxValue;
uint8_t rightWheelControl = maxValue;

// straight ride flag
bool straightF = 0;

// flagi etapow
bool f1 = false;
bool f2 = false;

// chwile czasu
unsigned long t1 = 0;
unsigned long t2 = 0;

// BLE Service
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;

unsigned long t0;
bool F0;

// Siec
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

bool leftWheelDirection = true;
bool rightWheelDirection = true;

unsigned long encTimer;

float speedLeft;
float speedRight;

int encCurrLeftCount; 
int encCurrRightCount;

float R = 0.01525; // promien kola
float Pi = 3.1415; // pi

float d = 0.0515; // rozstaw kol

bool Fr = true;

// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;

SoftwareTimer readEncTimer;



void setup()
{
    analogReadResolution(12);
    analogReference(AR_INTERNAL_3_0);

    omegaLeft[0] = 1;
    setupPWM();

    delay(1);

    pinMode(dirLeftA, OUTPUT);
    pinMode(dirLeftB, OUTPUT);

    pinMode(dirRightA, OUTPUT);
    pinMode(dirRightB, OUTPUT);

    F0 = true;
    t0 = millis() - 50;
    t3 = millis() -200;
    
    encoderLeftLast = 0;
    encoderRightLast = 0;
    encoderLeftCurr = 0;
    encoderRightCurr = 0;

    Serial.begin(115200);
    Serial.println("BugBoard Manual Control example");
    Serial.println("---------------------------\n");

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
    Bluefruit.setName("Bluefruit52");
    
    //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
    Bluefruit.setConnectCallback(connect_callback);
    Bluefruit.setDisconnectCallback(disconnect_callback);

    // Configure and Start Device Information Service
    bledis.setManufacturer("Adafruit Industries");
    bledis.setModel("Bluefruit Feather52");
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

    Serial.println("Please use BugApp to control BugBoard via BLE");
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

    if(millis()- t0 > 10)
    {
        readEncoders();
        t0 = millis();
    }
    // Forward data from HW Serial to BLEUART
    // timer który bedzie wysyłał dane o prękości kół i tak dalej do apki
  
/*
    while (Serial.available())
    {
        // Delay to wait for enough input, since we have a limited transmission buffer
        delay(2);

        bleuart.write( buf, count );
    }

    // Forward from BLEUART to HW 
    
    while ( bleuart.available() ) // tu nie może być while
    {
        if(bleuart.available()<2)
        {
            delay(1);
        }
        else
        {
            uint8_t chs[2];
            for(int i = 0; i < sizeof(chs); i++)
            {
                chs[i] = (uint8_t)bleuart.read();
            }
            GetControl(chs);
        }
        
    }
    */


// odczyt enkoderów //////////////////////////////////////////////////////////
    //delay(20);

    //int anl = analogRead(encoderLeft);
    //int anp = analogRead(encoderRight);

    //Serial.println(anl);
    //Serial.println(anp);

    //bool encLeft = digitalizeAnalog( anl );
    //bool encRight = digitalizeAnalog( anp );


    /*
    Serial.print("left - ");
    Serial.println(encLeft);

    Serial.print("right - ");
    Serial.println(encRight);
    */
/*
    if (encLeft != encoderLeftLast)
    {
        if(leftWheelDirection)
        {
            encoderLeftTicks++;
            encCurrLeftCount++;
            Serial.print("l tick + : ");
            Serial.println(encoderLeftTicks);
        }
        else
        {
            encoderLeftTicks--;
            encCurrLeftCount--;
            Serial.print("l tick - : ");
            Serial.println(encoderLeftTicks);
        }
        //Serial.println("l");
        encoderLeftLast = !encoderLeftLast;
    }
    if (encRight != encoderRightLast)
    {
        if(rightWheelDirection)
        {
            encoderRightTicks++;
            encCurrRightCount++;
            Serial.print("p tick + : ");
            Serial.println(encoderRightTicks);
        }
        else
        {
            encoderRightTicks--;
            encCurrRightCount--;
            Serial.print("p tick - : ");
            Serial.println(encoderRightTicks);
        }
        //Serial.println("p");
        encoderRightLast = !encoderRightLast;
    }
    
*/
    



    unsigned long encMili = millis();
    int timeSpan = encMili - encTimer;

    if(timeSpan >= 1000)
    {
        // 360st/32prążki
        // 11,25 stopni to prążek
        
        float encCurrLeft = (float)abs(encCurrLeftCount);
        float radLeft = (encCurrLeft * 11.25) * (3.1415 / 180);
        Serial.print("left speed - ");
        Serial.println(radLeft);
        Serial.println(encCurrLeft);
        speedLeft = radLeft / ((encMili - encTimer) / 1000);
        encCurrLeftCount = 0;

        
        float encCurrRight = (float)abs(encCurrRightCount);
        float radRight = (encCurrRight * 11.25) * (3.1415 / 180);
        Serial.print("right speed - ");
        Serial.println(radRight);
        Serial.println(encCurrRight);
        speedRight = radRight / ((encMili - encTimer) / 1000);
        encCurrRightCount = 0;
        encTimer = millis();
        ctr++;

        //Regulator();

        if(ctr>= 5)
        {
            Serial.print("Left ticks : ");
            Serial.println(encoderLeftTicks);
            Serial.print("Right ticks : ");
            Serial.println(encoderRightTicks);
            ctr = 0;
        }

        

    }
    /*
    if(Fr)
    {
        GoStraight(true, leftWheelControl , rightWheelControl);
        leftWheelDirection = true;
        rightWheelDirection = true;

        if(encoderLeftTicks >= 320)
        {
            Fr = false;
            
        }
    }
    else
    {
        GoStraight(false, leftWheelControl, rightWheelControl);
        leftWheelDirection = false;
        rightWheelDirection = false;

        if(encoderLeftTicks <= 0)
        {
            Fr = true;
        }
    }
    */

// liczenie prękości kątowych kół///////////////////////////////////////////



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
        Serial.println("************BACKWARD**************");
        if (millis() - t3 < 1500)
        {
            GoStraight(false, maxValue,  maxValue);
        }
        else
        {
            F0 = true;
        }
    }
    

// Sieć














// wysyłanie danych:

















// tutaj ważny będzie czas ten z odbioru

    // Request CPU to enter low-power mode until an event/interrupt occurs
    waitForEvent();
}


void blink_timer_callback(TimerHandle_t xTimerID)
{
    
    bleuart.write(omegaLeft, sizeof(omegaLeft));
    omegaLeft[0]++;
    (void) xTimerID;
    digitalToggle(LED_RED);
}

void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
}