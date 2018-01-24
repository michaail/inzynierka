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
byte encoderLeftTicks = 0;
byte encoderRightTicks = 0;

// number of wheel revs
int encoderLeftRevs = 0;
int encoderRightRevs = 0;

// previous encoders values
bool encoderLeftLast = 0;
bool encoderRightLast = 0;

// ref voltage for ADC
float mv_per_lsb = 3000.0f/256.0f; // 8-bit ADC z 3.3V input range

const byte maxValue = bit(8)-1;

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


// Siec
uint32_t InputVal[2];
float OutputVal[4];
float sum;

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


// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;

void setupPWM()
{
    // initialize PWM module
    HwPWMx[0]->addPin(enableLeft);
    HwPWMx[0]->addPin(enableRight);

    HwPWM0.begin();
    HwPWM0.setResolution(8); // max value 255
    HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_1);
}

void setup()
{
    analogReadResolution(8);
    analogReference(AR_INTERNAL_3_0);

    omegaLeft[0] = 1;
    setupPWM();

    delay(1);

    pinMode(dirLeftA, OUTPUT);
    pinMode(dirLeftB, OUTPUT);

    pinMode(dirRightA, OUTPUT);
    pinMode(dirRightB, OUTPUT);

    
    encoderLeftLast = 0;
    encoderRightLast = 0;

    Serial.begin(115200);
    Serial.println("BugBoard Manual Control example");
    Serial.println("---------------------------\n");

    // to myślę że można wywalić ale pewny nie jestem
    // Initialize blinkTimer for 1000 ms and start it
    blinkTimer.begin(1000, blink_timer_callback);
    blinkTimer.start();

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
    digitalWrite(dirLeftA, dir);
    digitalWrite(dirLeftB, !dir);

    digitalWrite(dirRightA, dir);
    digitalWrite(dirRightB, !dir);

    HwPWMx[0]->writePin(enableLeft, maxValue, false);
    HwPWMx[0]->writePin(enableRight, maxValue, false);

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
    HwPWMx[0]->writePin(enableRight, maxValue, false);

}

void CancelControl() // set zero to all motors
{
    HwPWMx[0]->writePin(enableLeft, 0, false);
    HwPWMx[0]->writePin(enableRight, 0, false);
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

bool digitalizeAnalog(int value)
{
    if(value < 90) // kolor bialy
    {
        return false;
    }
    else
    {
        return true;
    }
}


void loop()
{
    // Forward data from HW Serial to BLEUART
    // timer który bedzie wysyłał dane o prękości kół i tak dalej do apki
  
/*
    while (Serial.available())
    {
        // Delay to wait for enough input, since we have a limited transmission buffer
        delay(2);

        bleuart.write( buf, count );
    }
*/
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
// odczyt enkoderów //////////////////////////////////////////////////////////
    bool encLeft = digitalizeAnalog( analogRead(encoderLeft) );
    int encRight = digitalizeAnalog( analogRead(encoderRight) );

    if (encLeft != encoderLeftLast)
    {
        encoderLeftTicks++;
        encoderLeftLast = !encoderLeftLast;
    }
    if (encRight != encoderRightLast)
    {
        encoderRightTicks++;
        encoderLeftLast = !encoderRightLast;
    }

// liczenie prękości kątowych kół///////////////////////////////////////////





// Sieć














// wysyłanie danych:


















// tutaj ważny będzie czas ten z odbioru

    // Request CPU to enter low-power mode until an event/interrupt occurs
    waitForEvent();
}


/* ************************************
 * 
 * Neural Network Setup
 * 
 * ***********************************/

void SetupNeuralNet()
{
    inputLayer = 2;
    hiddenLayer = 8;
    outputLayer = 4;

    inputWeight[0][0] =   3.97681621216933;
    inputWeight[0][1] =  -3.43496298788826;
    inputWeight[0][2] =   0.51806207013445;
    inputWeight[0][3] =   0.82171043263078;
    inputWeight[0][4] =   0.71482567750189;
    inputWeight[0][5] =   3.13587278595063;
    inputWeight[0][6] =  -1.13347989568598;
    inputWeight[0][7] =   2.75103671091913;

    inputWeight[1][0] =  -1.12567585837059;
    inputWeight[1][1] =  -5.60379820912743;
    inputWeight[1][2] =  -1.67260335217006;
    inputWeight[1][3] =  -0.59083546705418;
    inputWeight[1][4] =   2.04527923010639;
    inputWeight[1][5] =   2.53399406174777;
    inputWeight[1][6] =  -4.57620518303079;
    inputWeight[1][7] =   1.37198169402332;

    inputWeight[2][0] =  -3.51380189962795;
    inputWeight[2][1] =   5.82953064721858;
    inputWeight[2][2] =   0.13605362831278;
    inputWeight[2][3] =   0.29849016329210;
    inputWeight[2][4] =  -1.17434690710991;
    inputWeight[2][5] =   1.96714436336079;
    inputWeight[2][6] =  -1.62541860897994;
    inputWeight[2][7] =   2.79152826743692;

    hiddenWeight[0][0] =  -0.374539869053815;
    hiddenWeight[0][1] =   0.005209587497091;
    hiddenWeight[0][2] =   1.450415974891660;
    hiddenWeight[0][3] =   1.450512795351500;

    hiddenWeight[1][0] =   0.045091367937266;
    hiddenWeight[1][1] =   0.105106685428046;
    hiddenWeight[1][2] =  -0.730598594275543;
    hiddenWeight[1][3] =  -0.730520130391922;

    hiddenWeight[2][0] =   0.163770796582358;
    hiddenWeight[2][1] =  -0.199147228985666;
    hiddenWeight[2][2] =   3.122170139602460;
    hiddenWeight[2][3] =   3.123815518815370;

    hiddenWeight[3][0] =  -1.270754271837920;
    hiddenWeight[3][1] =   0.710953002540675;
    hiddenWeight[3][2] =  -3.389839440958070;
    hiddenWeight[3][3] =  -3.391039100880490;

    hiddenWeight[4][0] =  -0.242669899517994;
    hiddenWeight[4][1] =  -0.646304722160610;
    hiddenWeight[4][2] =   2.148537956459040;
    hiddenWeight[4][3] =   2.149746437647770;

    hiddenWeight[5][0] =  -0.158782162500928;
    hiddenWeight[5][1] =  -0.146909674635699;
    hiddenWeight[5][2] =  -0.009617146295958;
    hiddenWeight[5][3] =  -0.008957030265926;

    hiddenWeight[6][0] =   0.120545026126452;
    hiddenWeight[6][1] =   0.269719021014171;
    hiddenWeight[6][2] =  -0.403459385194824;
    hiddenWeight[6][3] =  -0.403554762692929;

    hiddenWeight[7][0] =   0.221448389101420;
    hiddenWeight[7][1] =   0.086036007781647;
    hiddenWeight[7][2] =   0.826170858432722;
    hiddenWeight[7][3] =   0.823929019418342;

    hiddenWeight[8][0] =  -0.145356096680302;
    hiddenWeight[8][1] =  -0.346164769000769;
    hiddenWeight[8][2] =   2.12361827932047;
    hiddenWeight[8][3] =   2.12594115099956;
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

