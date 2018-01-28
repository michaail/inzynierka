/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

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

void startAdv(void)
{
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

    // Include bleuart 128-bit uuid
    Bluefruit.Advertising.addService(bleuart);

    // Secondary Scan Response packet (optional)
    // Since there is no room for 'Name' in Advertising packet
    Bluefruit.ScanResponse.addName();
  
    /* Start Advertising
    * - Enable auto advertising if disconnected
    * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
    * - Timeout for fast mode is 30 seconds
    * - Start(timeout) with timeout = 0 will advertise forever (until connected)
    * 
    * For recommended advertising interval
    * https://developer.apple.com/library/content/qa/qa1931/_index.html   
    */
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}


void GoStraight(bool dir) // 1 - forward; 0 - backward
{
    digitalWrite(dirLeftA, !dir);
    digitalWrite(dirLeftB, dir);

    digitalWrite(dirRightA, !dir);
    digitalWrite(dirRightB, dir);

    HwPWMx[0]->writePin(enableLeft, maxValue, false);
    HwPWMx[1]->writePin(enableRight, maxValue, false);

    // tu jeszcze ten regulator jak będzie chyba że na pałę
    // tutaj obsługa tego żeby jechał prosto
}

void GoStraight(bool dir, uint8_t valueLeft, uint8_t valueRight)
{
    digitalWrite(dirLeftA, !dir);
    digitalWrite(dirLeftB, dir);

    digitalWrite(dirRightA, !dir);
    digitalWrite(dirRightB, dir);

    HwPWMx[0]->writePin(enableLeft, valueLeft, false);
    HwPWMx[1]->writePin(enableRight, valueRight, false);

    // tu jeszcze ten regulator jak będzie chyba że na pałę
    // tutaj obsługa tego żeby jechał prosto
}

void Turn(bool dir) // 1 - prawo; 0 - lewo
{
    digitalWrite(dirLeftA, dir);
    digitalWrite(dirLeftB, !dir);

    digitalWrite(dirRightA, !dir);
    digitalWrite(dirRightB, dir);

    HwPWMx[0]->writePin(enableLeft, maxValue, false);
    HwPWMx[1]->writePin(enableRight, maxValue, false);

}

void Turn(bool dir, uint8_t valueLeft, uint8_t valueRight)
{
    digitalWrite(dirLeftA, dir);
    digitalWrite(dirLeftB, !dir);

    digitalWrite(dirRightA, !dir);
    digitalWrite(dirRightB, dir);

    HwPWMx[0]->writePin(enableLeft, valueLeft, false);
    HwPWMx[1]->writePin(enableRight, valueRight, false);
}

void DriveNN(bool leftDir, uint8_t valueLeft, bool rightDir, uint8_t valueRight)
{
    // left
    digitalWrite(dirLeftA, !leftDir);
    digitalWrite(dirLeftB, leftDir);

    // right
    digitalWrite(dirRightA, !rightDir);
    digitalWrite(dirRightB, rightDir);

    HwPWMx[0]->writePin(enableLeft, valueLeft, false);
    HwPWMx[1]->writePin(enableRight, valueRight, false);
}

void CancelControl() // set zero to all motors
{
    HwPWMx[0]->writePin(enableLeft, 0, false);
    HwPWMx[1]->writePin(enableRight, 0, false);
}

void ManualControl(uint8_t val)
{
    if(val == 0x57 || val == 0x77) // 'W/w' FORWARD
    {
        GoStraight(true); 
        Serial.println("Received W/w");
        // what if 'W'
    }
    else if(val == 0x41 || val == 0x61) // 'A/a' LEFT
    {
        Turn(false);
        Serial.println("Received A/a");
        // what if 'A'
    }
    else if(val == 0x53 || val == 0x73) // 'S/s' BACKWARD
    {
        GoStraight(false);
        Serial.println("Received S/s");
        // what if 'S'
    }
    else if(val == 0x44 || val == 0x64) // 'D/d' RIGHT
    {
        Turn(true);
        Serial.println("Received D/d");
        // what if 'D'
    }
    else if(val == 0x48 || val == 0x68)
    {
        CancelControl();
        Serial.println("Received key release sign");
    }
    else
    {
        Serial.println("Received something else");
        Serial.println(val);
        Serial.println(val, HEX);
        // what if something else
    }

}

void GetControl(byte msg[])
{
    uint8_t val;
    
    if(msg[1] == 0x0A)
    {
        val = msg[0];
        ManualControl(val);
    }
}

/*****************************************************************************************
 * 
 * Reading Encoders
 * 
 * **************************************************************************************/

bool digitalizeAnalog(int value)
{
    if(value < 1000) // kolor bialy
    {
        return false;
    }
    else
    {
        return true;
    }
}

//void readEnc_Timer_callback(TimerHandle_t xTimerID)
void readEncoders()
{
    //Serial.println(analogRead(encoderLeft));
    
    if (digitalizeAnalog(analogRead(encoderLeft)) != encoderLeftLast)
    {
        if(leftWheelDirection)
        {
            encoderLeftTicks++;
            encCurrLeftCount++;
            //Serial.print("l tick + : ");
            //Serial.println(encoderLeftTicks);
        }
        else
        {
            encoderLeftTicks--;
            encCurrLeftCount--;
            //Serial.print("l tick - : ");
            //Serial.println(encoderLeftTicks);
        }
        if(encoderLeftTicks % 32 == 0)
        {
            encoderLeftRevs++;
        }
        encoderLeftLast = !encoderLeftLast;
    }

    if (digitalizeAnalog(analogRead(encoderRight)) != encoderRightLast)
    {
        if(rightWheelDirection)
        {
            encoderRightTicks++;
            encCurrRightCount++;
            //Serial.print("p tick + : ");
            //Serial.println(encoderRightTicks);
        }
        else
        {
            encoderRightTicks--;
            encCurrRightCount--;
            //Serial.print("p tick - : ");
            //Serial.println(encoderRightTicks);
        }
        if(encoderRightTicks % 32 == 0)
        {
            encoderRightRevs++;
        }
        encoderRightLast = !encoderRightLast;
    }


}


void Regulator()
{
    float differ;
    
    int currRegLeftValue;

    if (speedLeft < speedRight)
    {
        differ = speedRight - speedLeft;

        if (leftWheelControl == maxLeftValue)
        {
            rightWheelControl = maxValue;
        }
        else
        {
            leftWheelControl += 15;
            rightWheelControl = maxValue;
        }
        

    }

    else if (speedRight < speedLeft)
    {
        differ = speedLeft - speedRight;

        if(rightWheelControl == maxValue)
        {
            leftWheelControl -= 20;
        }
        else
        {
            rightWheelControl = maxValue;
            leftWheelControl -= 15;
        }
    }

    Serial.print("left : ");
    Serial.print(leftWheelControl);
    Serial.print(" __ right : ");
    Serial.println(rightWheelControl);
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

/*********************************************************************************
 * 
 * Obliczanie sieci
 * 
 * ******************************************************************************/

void CalculateNet()
{
    readSensors();
    feedForward();
    leftWheelDirection = GetDirection(OutputVal[2]);
    rightWheelDirection = GetDirection(OutputVal[3]);
    
    uint8_t valueLeft = GetLeftValue(OutputVal[0]);
    uint8_t valueRight = GetRightValue(OutputVal[1]);
    
    uint8_t t[2];
    t[0] = valueLeft;
    t[1] = valueRight;

    bleuart.write(t, sizeof(t));


    //(uint8_t)OutputVal[0];
    //uint8_t valueRight = (uint8_t)OutputVal[1];
    
    if(!leftWheelDirection && !rightWheelDirection)
    {
        F0 = false;
    }


    DriveNN(leftWheelDirection, valueLeft, rightWheelDirection, valueRight);

}





/* ************************************
 * 
 * Neural Network Setup
 * 
 * ***********************************/

bool GetDirection(float output)
{
    if (output >= 0)
    {
        return true;
    }
    else
    {
        return false;
    }
    
}



void setupNeuralNet() {
    inputLayer = 2;
    hiddenLayer = 8;
    outputLayer = 4;

    inputWeight[0][0] =   6.80969431857316;
    inputWeight[0][1] =   0.26855543595975;
    inputWeight[0][2] =   1.58337760007558;
    inputWeight[0][3] =   4.15381039169684;
    inputWeight[0][4] =  -1.24736318981999;
    inputWeight[0][5] =  -0.80660951489351;
    inputWeight[0][6] =  -1.09198154513042;
    inputWeight[0][7] =  -0.51179944834344;

    inputWeight[1][0] =  -1.27131998649567;
    inputWeight[1][1] =  -2.38790438490293;
    inputWeight[1][2] =  -1.76863075111441;
    inputWeight[1][3] =   0.42292019751158;
    inputWeight[1][4] =  -4.47865242465974;
    inputWeight[1][5] =  -0.82858158409804;
    inputWeight[1][6] =   5.41562559946484;
    inputWeight[1][7] =  -5.66721739487875;

    inputWeight[2][0] =  -5.92721406344743;
    inputWeight[2][1] =  -1.31242929215533;
    inputWeight[2][2] =   0.10322036466481;
    inputWeight[2][3] =  -0.72960428835476;
    inputWeight[2][4] =   0.73711697680585;
    inputWeight[2][5] =   0.85835855737112;
    inputWeight[2][6] =  -4.77598570912289;
    inputWeight[2][7] =  -3.83441947021448;

    hiddenWeight[0][0] =  -0.3169358493330;
    hiddenWeight[0][1] =   0.0270309828736;
    hiddenWeight[0][2] =   1.1197331911138;
    hiddenWeight[0][3] =   1.1197194585690;

    hiddenWeight[1][0] =  -0.1055958176710;
    hiddenWeight[1][1] =   0.3425887318713;
    hiddenWeight[1][2] =  -0.4031500659192;
    hiddenWeight[1][3] =  -0.4032282238495;

    hiddenWeight[2][0] =  -0.3414006595213;
    hiddenWeight[2][1] =   0.3985953937779;
    hiddenWeight[2][2] =   0.0506213761853;
    hiddenWeight[2][3] =   0.0506297346959;

    hiddenWeight[3][0] =  -0.2013885043364;
    hiddenWeight[3][1] =  -0.0563183708253;
    hiddenWeight[3][2] =  -0.3155347573760;
    hiddenWeight[3][3] =  -0.3155188194427;

    hiddenWeight[4][0] =   0.0242268534044;
    hiddenWeight[4][1] =   0.1345324175126;
    hiddenWeight[4][2] =   0.5284387113000;
    hiddenWeight[4][3] =   0.5284461922020;

    hiddenWeight[5][0] =   0.7037253590199;
    hiddenWeight[5][1] =   0.5190035020948;
    hiddenWeight[5][2] =  -2.0324373118065;
    hiddenWeight[5][3] =  -2.0323728108361;

    hiddenWeight[6][0] =   0.0923821107533;
    hiddenWeight[6][1] =  -0.3437614594310;
    hiddenWeight[6][2] =   1.1390938101161;
    hiddenWeight[6][3] =   1.1391045927481;

    hiddenWeight[7][0] =   0.0071265018911;
    hiddenWeight[7][1] =  -0.2518332154308;
    hiddenWeight[7][2] =   0.3518284725672;
    hiddenWeight[7][3] =   0.3518080730975;

    hiddenWeight[8][0] =  -0.4643403908826;
    hiddenWeight[8][1] =  -0.3615915214822;
    hiddenWeight[8][2] =   2.3504296298988;
    hiddenWeight[8][3] =   2.3503695150670;
 }



 void readSensors() {
    InputVal[0] = analogRead(sensorLeft);
    InputVal[1] = analogRead(SensorRight);

    Serial.print("A0: ");
    Serial.print(InputVal[0]);
    Serial.print("  ");
    Serial.print("A1: ");
    Serial.println(InputVal[1]);
}


float tansig(float value) {
    float_t wyk = (-2.0) * value;
    return (2.0 / (1.0 + exp(wyk))) - 1.0;
}


uint8_t GetRightValue(float outputValue)
{
    if(outputValue >= 3.5)
    {
        return maxValue;
    }
    else if(outputValue < 3.5 && outputValue >= 2.0)
    {
        return maxValue - 70;
    }
    else
    {
        return 0;
    }
}

uint8_t GetLeftValue(float outputValue)
{
    if(outputValue >= 3.5)
    {
        return maxLeftValue;
    }
    else if(outputValue < 3.5 && outputValue >= 2.0)
    {
        return maxLeftValue - 70;
    }
    else
    {
        return 0;
    }
}


void feedForward() {
    Serial.println(InputVal[0]);
    Serial.println(InputVal[1]);


    inputNeuron[0] = (1.0 + 1.0)*(InputVal[0] - 3000.0) / (4095.0 - 3000.0) - 1.0;
    inputNeuron[1] = (1.0 + 1.0)*(InputVal[1] - 3000.0) / (4095.0 - 3000.0) - 1.0;

    Serial.print("input Neuron left - ");
    Serial.println(inputNeuron[0]);

    Serial.print("input Neuron right - ");
    Serial.println(inputNeuron[0]);


    /*
    Serial.print("inputNeuron: ");
    Serial.print(inputNeuron[0]);
    Serial.print(" ");
    Serial.println(inputNeuron[1]);
    */

    sum = 0.0;
    sum = inputNeuron[0] * inputWeight[0][0] + inputNeuron[1] * inputWeight[1][0] + inputWeight[2][0];
    hiddenNeuron[0] = tansig(sum);

    sum = 0.0;
    sum = inputNeuron[0] * inputWeight[0][1] + inputNeuron[1] * inputWeight[1][1] + inputWeight[2][1];
    hiddenNeuron[1] = tansig(sum);
  
    sum = 0.0;
    sum = inputNeuron[0] * inputWeight[0][2] + inputNeuron[1] * inputWeight[1][2] + inputWeight[2][2];
    hiddenNeuron[2] = tansig(sum);
  
    sum = 0.0;
    sum = inputNeuron[0] * inputWeight[0][3] + inputNeuron[1] * inputWeight[1][3] + inputWeight[2][3];
    hiddenNeuron[3] = tansig(sum);
  
    sum = 0.0;
    sum = inputNeuron[0] * inputWeight[0][4] + inputNeuron[1] * inputWeight[1][4] + inputWeight[2][4];
    hiddenNeuron[4] = tansig(sum);
  
    sum = 0.0;
    sum = inputNeuron[0] * inputWeight[0][5] + inputNeuron[1] * inputWeight[1][5] + inputWeight[2][5];  
    hiddenNeuron[5] = tansig(sum);
  
    sum = 0.0;
    sum = inputNeuron[0] * inputWeight[0][6] + inputNeuron[1] * inputWeight[1][6] + inputWeight[2][6];
    hiddenNeuron[6] = tansig(sum);
  
    sum = 0.0;
    sum = inputNeuron[0] * inputWeight[0][7] + inputNeuron[1] * inputWeight[1][7] + inputWeight[2][7];
    hiddenNeuron[7] = tansig(sum);  

    
    Serial.print("hiddenNeuron: ");
    Serial.print(hiddenNeuron[0]);
    Serial.print(" ");
    Serial.print(hiddenNeuron[1]);
    Serial.print(" ");
    Serial.print(hiddenNeuron[2]);
    Serial.print(" ");
    Serial.print(hiddenNeuron[3]);
    Serial.print(" ");
    Serial.print(hiddenNeuron[4]);
    Serial.print(" ");
    Serial.print(hiddenNeuron[5]);
    Serial.print(" ");
    Serial.print(hiddenNeuron[6]);
    Serial.print(" ");
    Serial.println(hiddenNeuron[7]);
    

    outputNeuron[0] = hiddenNeuron[0] * hiddenWeight[0][0] + hiddenNeuron[1] * hiddenWeight[1][0]
                    + hiddenNeuron[2] * hiddenWeight[2][0] + hiddenNeuron[3] * hiddenWeight[3][0]
                    + hiddenNeuron[4] * hiddenWeight[4][0] + hiddenNeuron[5] * hiddenWeight[5][0]
                    + hiddenNeuron[6] * hiddenWeight[6][0] + hiddenNeuron[7] * hiddenWeight[7][0]
                    + hiddenWeight[8][0];
    outputNeuron[1] = hiddenNeuron[0] * hiddenWeight[0][1] + hiddenNeuron[1] * hiddenWeight[1][1]
                    + hiddenNeuron[2] * hiddenWeight[2][1] + hiddenNeuron[3] * hiddenWeight[3][1]
                    + hiddenNeuron[4] * hiddenWeight[4][1] + hiddenNeuron[5] * hiddenWeight[5][1]
                    + hiddenNeuron[6] * hiddenWeight[6][1] + hiddenNeuron[7] * hiddenWeight[7][1]
                    + hiddenWeight[8][1];
    outputNeuron[2] = hiddenNeuron[0] * hiddenWeight[0][2] + hiddenNeuron[1] * hiddenWeight[1][2]
                    + hiddenNeuron[2] * hiddenWeight[2][2] + hiddenNeuron[3] * hiddenWeight[3][2]
                    + hiddenNeuron[4] * hiddenWeight[4][2] + hiddenNeuron[5] * hiddenWeight[5][2]
                    + hiddenNeuron[6] * hiddenWeight[6][2] + hiddenNeuron[7] * hiddenWeight[7][2]
                    + hiddenWeight[8][2];
    outputNeuron[3] = hiddenNeuron[0] * hiddenWeight[0][3] + hiddenNeuron[1] * hiddenWeight[1][3]
                    + hiddenNeuron[2] * hiddenWeight[2][3] + hiddenNeuron[3] * hiddenWeight[3][3]
                    + hiddenNeuron[4] * hiddenWeight[4][3] + hiddenNeuron[5] * hiddenWeight[5][3]
                    + hiddenNeuron[6] * hiddenWeight[6][3] + hiddenNeuron[7] * hiddenWeight[7][3]
                    + hiddenWeight[8][3];

    
    Serial.print("outputNeuron: ");
    Serial.print(outputNeuron[0]);
    Serial.print(" ");
    Serial.print(outputNeuron[1]);
    Serial.print(" ");
    Serial.print(outputNeuron[2]);
    Serial.print(" ");
    Serial.println(outputNeuron[3]);
    
  
    OutputVal[0] = round((1.0 - 4.0)*(outputNeuron[0] + 1.0)/(1.0 + 1.0) + 4.0);
    OutputVal[1] = round((1.0 - 4.0)*(outputNeuron[1] + 1.0)/(1.0 + 1.0) + 4.0);
    OutputVal[2] = ((1.0 + 1.0)*(outputNeuron[2] + 1.0)/(1.0 + 1.0) - 1.0);
    OutputVal[3] = ((1.0 + 1.0)*(outputNeuron[3] + 1.0)/(1.0 + 1.0) - 1.0);

    Serial.print("O0: ");
    Serial.print(OutputVal[0]);
    Serial.print("  ");
    Serial.print("O1: ");
    Serial.print(OutputVal[1]);
    Serial.print("  ");
    Serial.print("O2: ");
    Serial.print(OutputVal[2]);
    Serial.print("  ");
    Serial.print("O3: ");
    Serial.println(OutputVal[3]);
}

void connect_callback(uint16_t conn_handle)
{
    char central_name[32] = { 0 };
    Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

    Serial.print("Connected to ");
    Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
    (void) conn_handle;
    (void) reason;

    Serial.println();
    Serial.println("Disconnected");
}

/**
 * Software Timer callback is invoked via a built-in FreeRTOS thread with
 * minimal stack size. Therefore it should be as simple as possible. If
 * a periodically heavy task is needed, please use Scheduler.startLoop() to
 * create a dedicated task for it.
 * 
 * More information http://www.freertos.org/RTOS-software-timer.html
 */
void blink_timer_callback(TimerHandle_t xTimerID)
{
    
    bleuart.write(omegaLeft, sizeof(omegaLeft));
    omegaLeft[0]++;
    (void) xTimerID;
    digitalToggle(LED_RED);
}

/**
 * RTOS Idle callback is automatically invoked by FreeRTOS
 * when there are no active threads. E.g when loop() calls delay() and
 * there is no bluetooth or hw event. This is the ideal place to handle
 * background data.
 * 
 * NOTE: FreeRTOS is configured as tickless idle mode. After this callback
 * is executed, if there is time, freeRTOS kernel will go into low power mode.
 * Therefore waitForEvent() should not be called in this callback.
 * http://www.freertos.org/low-power-tickless-rtos.html
 * 
 * WARNING: This function MUST NOT call any blocking FreeRTOS API 
 * such as delay(), xSemaphoreTake() etc ... for more information
 * http://www.freertos.org/a00016.html
 */
void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
}

