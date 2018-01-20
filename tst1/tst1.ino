//Rx, Tx -> P0.08, P0.06
//RESET -> ??
//32.768kHz -> P0.00, P0.01
//SWDIO, SWCLK, SWO -> PIN 37, 36, P0.18/32
#define ARM_MATH_CM4

#include <bluefruit.h>
#include <arm_math.h>

BLEDis bledis;
BLEHidAdafruit blehid;

//Nazwy zgodne z oznaczeniem sterownika silników
#define PWR_LED 17

#define AIN0 A0  //PIN P0.02
#define AIN1 A1  //PIN P0.03

#define EN12 A2  //PIN P0.04
#define EN34 A3  //PIN P0.05

#define IN1  27  //PIN P0.13
#define IN2  28  //PIN P0.14
#define IN3  29  //PIN P0.15
#define IN4  30  //PIN P0.16

#define PWM_RES 100 //  PWM resolution

uint32_t InputVals[2];
float OutputVals[4];
float sum;

uint8_t inputLayer;
uint8_t hiddenLayer;
uint8_t outputLayer;

float inputWeights[3][8];
float hiddenWeights[9][4];

float inputNeurons[2];
float hiddenNeurons[8];
float outputNeurons[4];

void setup() {
  pinMode(PWR_LED, OUTPUT);
  digitalWrite(PWR_LED, HIGH);
  
  Serial.begin(115200);
  Serial.println("Initielizing...");

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("R2D2");
  Bluefruit.setConnInterval(9, 12);
  
  bledis.setManufacturer("Mustafar - Droid Planet");
  bledis.setModel("Astrodroid Series");

  //blehid.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  //Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
  //Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(122, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(180);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  HwPWMx[0] -> addPin(EN12);
  HwPWMx[1] -> addPin(EN34);
  HwPWM0.begin();
  HwPWM0.setResolution(PWM_RES);
  HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_16);  //freq: = 1MHz

  Serial.println("Initielized.");
  Serial.println("Application start...");

  inputLayer = 2;
  hiddenLayer = 8;
  outputLayer = 4;

  inputWeights[0][0] =   3.97681621216933;
  inputWeights[0][1] =  -3.43496298788826;
  inputWeights[0][2] =   0.51806207013445;
  inputWeights[0][3] =   0.82171043263078;
  inputWeights[0][4] =   0.71482567750189;
  inputWeights[0][5] =   3.13587278595063;
  inputWeights[0][6] =  -1.13347989568598;
  inputWeights[0][7] =   2.75103671091913;

  inputWeights[1][0] =  -1.12567585837059;
  inputWeights[1][1] =  -5.60379820912743;
  inputWeights[1][2] =  -1.67260335217006;
  inputWeights[1][3] =  -0.59083546705418;
  inputWeights[1][4] =   2.04527923010639;
  inputWeights[1][5] =   2.53399406174777;
  inputWeights[1][6] =  -4.57620518303079;
  inputWeights[1][7] =   1.37198169402332;

  inputWeights[2][0] =  -3.51380189962795;
  inputWeights[2][1] =   5.82953064721858;
  inputWeights[2][2] =   0.13605362831278;
  inputWeights[2][3] =   0.29849016329210;
  inputWeights[2][4] =  -1.17434690710991;
  inputWeights[2][5] =   1.96714436336079;
  inputWeights[2][6] =  -1.62541860897994;
  inputWeights[2][7] =   2.79152826743692;

  hiddenWeights[0][0] =  -0.374539869053815;
  hiddenWeights[0][1] =   0.005209587497091;
  hiddenWeights[0][2] =   1.450415974891660;
  hiddenWeights[0][3] =   1.450512795351500;

  hiddenWeights[1][0] =   0.045091367937266;
  hiddenWeights[1][1] =   0.105106685428046;
  hiddenWeights[1][2] =  -0.730598594275543;
  hiddenWeights[1][3] =  -0.730520130391922;

  hiddenWeights[2][0] =   0.163770796582358;
  hiddenWeights[2][1] =  -0.199147228985666;
  hiddenWeights[2][2] =   3.122170139602460;
  hiddenWeights[2][3] =   3.123815518815370;

  hiddenWeights[3][0] =  -1.270754271837920;
  hiddenWeights[3][1] =   0.710953002540675;
  hiddenWeights[3][2] =  -3.389839440958070;
  hiddenWeights[3][3] =  -3.391039100880490;

  hiddenWeights[4][0] =  -0.242669899517994;
  hiddenWeights[4][1] =  -0.646304722160610;
  hiddenWeights[4][2] =   2.148537956459040;
  hiddenWeights[4][3] =   2.149746437647770;

  hiddenWeights[5][0] =  -0.158782162500928;
  hiddenWeights[5][1] =  -0.146909674635699;
  hiddenWeights[5][2] =  -0.009617146295958;
  hiddenWeights[5][3] =  -0.008957030265926;

  hiddenWeights[6][0] =   0.120545026126452;
  hiddenWeights[6][1] =   0.269719021014171;
  hiddenWeights[6][2] =  -0.403459385194824;
  hiddenWeights[6][3] =  -0.403554762692929;

  hiddenWeights[7][0] =   0.221448389101420;
  hiddenWeights[7][1] =   0.086036007781647;
  hiddenWeights[7][2] =   0.826170858432722;
  hiddenWeights[7][3] =   0.823929019418342;

  hiddenWeights[8][0] =  -0.145356096680302;
  hiddenWeights[8][1] =  -0.346164769000769;
  hiddenWeights[8][2] =   2.12361827932047;
  hiddenWeights[8][3] =   2.12594115099956;
}

void loop() {
  unsigned long czas = millis();
  InputVals[0] = analogRead(A0);
  InputVals[1] = analogRead(A1);

  Serial.print("Measured: ");
  Serial.print(InputVals[0]);
  Serial.print(", ");
  Serial.println(InputVals[1]);

  feedForward();
  Serial.print("InputNeurons: ");
  Serial.print(inputNeurons[0]);
  Serial.print(", ");
  Serial.print(inputNeurons[1]);
  Serial.println();
  
  Serial.print("HiddenNeurons: ");
  Serial.print(hiddenNeurons[0]);
  Serial.print(", ");
  Serial.print(hiddenNeurons[1]);
  Serial.print(", ");
  Serial.print(hiddenNeurons[2]);
  Serial.print(", ");
  Serial.print(hiddenNeurons[3]);
  Serial.print(", ");
  Serial.print(hiddenNeurons[4]);
  Serial.print(", ");
  Serial.print(hiddenNeurons[5]);
  Serial.print(", ");
  Serial.print(hiddenNeurons[6]);
  Serial.print(", ");
  Serial.print(hiddenNeurons[7]);
  Serial.println();
  
  Serial.print("OutputNeurons: ");
  Serial.print(outputNeurons[0]);
  Serial.print(", ");
  Serial.print(outputNeurons[1]);
  Serial.print(", ");
  Serial.print(outputNeurons[2]);
  Serial.print(", ");
  Serial.print(outputNeurons[3]);
  Serial.println();

  Serial.print("Set: ");
  Serial.print(OutputVals[0]);
  Serial.print(", ");
  Serial.print(OutputVals[1]);
  Serial.print(", ");
  Serial.print(OutputVals[2]);
  Serial.print(", ");
  Serial.print(OutputVals[3]);
  Serial.println();

  if(OutputVals[2] < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  if(OutputVals[3] < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  HwPWMx[0] -> writePin(EN12, OutputVals[0], false);
  HwPWMx[0] -> writePin(EN34, OutputVals[1], false);
  unsigned long tm = millis() - czas;
  Serial.println("czas obliczen: ");
  Serial.print(tm);
  delay(1);
}

void feedForward() {
  inputNeurons[0] = (1.0 + 1.0)*(InputVals[0] - 1000.0) / (1900.0 - 1000.0) - 1.0;
  inputNeurons[1] = (1.0 + 1.0)*(InputVals[1] - 1000.0) / (1900.0 - 1000.0) - 1.0;

  sum = inputNeurons[0] * inputWeights[0][0] + inputNeurons[1] * inputWeights[1][0] + inputWeights[2][0];
  hiddenNeurons[0] = tansig(sum);
  sum = inputNeurons[1] * inputWeights[0][1] + inputNeurons[1] * inputWeights[1][1] + inputWeights[2][1];
  hiddenNeurons[1] = tansig(sum);
  sum = inputNeurons[2] * inputWeights[0][2] + inputNeurons[1] * inputWeights[1][2] + inputWeights[2][2];
  hiddenNeurons[2] = tansig(sum);
  sum = inputNeurons[3] * inputWeights[0][3] + inputNeurons[1] * inputWeights[1][3] + inputWeights[2][3];
  hiddenNeurons[3] = tansig(sum);
  sum = inputNeurons[4] * inputWeights[0][4] + inputNeurons[1] * inputWeights[1][4] + inputWeights[2][4];
  hiddenNeurons[4] = tansig(sum);
  sum = inputNeurons[5] * inputWeights[0][5] + inputNeurons[1] * inputWeights[1][5] + inputWeights[2][5];
  hiddenNeurons[5] = tansig(sum);
  sum = inputNeurons[6] * inputWeights[0][6] + inputNeurons[1] * inputWeights[1][6] + inputWeights[2][6];
  hiddenNeurons[6] = tansig(sum);
  sum = inputNeurons[7] * inputWeights[0][7] + inputNeurons[1] * inputWeights[1][7] + inputWeights[2][7];
  hiddenNeurons[7] = tansig(sum);
  
  outputNeurons[0] = hiddenNeurons[0] * hiddenWeights[0][0] + hiddenNeurons[1] * hiddenWeights[1][0]
                    + hiddenNeurons[2] * hiddenWeights[2][0] + hiddenNeurons[3] * hiddenWeights[3][0]
                    + hiddenNeurons[4] * hiddenWeights[4][0] + hiddenNeurons[5] * hiddenWeights[5][0]
                    + hiddenNeurons[6] * hiddenWeights[6][0] + hiddenNeurons[7] * hiddenWeights[7][0]
                    + hiddenWeights[8][0];
  outputNeurons[1] = hiddenNeurons[0] * hiddenWeights[0][1] + hiddenNeurons[1] * hiddenWeights[1][1]
                    + hiddenNeurons[2] * hiddenWeights[2][1] + hiddenNeurons[3] * hiddenWeights[3][1]
                    + hiddenNeurons[4] * hiddenWeights[4][1] + hiddenNeurons[5] * hiddenWeights[5][1]
                    + hiddenNeurons[6] * hiddenWeights[6][1] + hiddenNeurons[7] * hiddenWeights[7][1]
                    + hiddenWeights[8][1];
  outputNeurons[2] = hiddenNeurons[0] * hiddenWeights[0][2] + hiddenNeurons[1] * hiddenWeights[1][2]
                    + hiddenNeurons[2] * hiddenWeights[2][2] + hiddenNeurons[3] * hiddenWeights[3][2]
                    + hiddenNeurons[4] * hiddenWeights[4][2] + hiddenNeurons[5] * hiddenWeights[5][2]
                    + hiddenNeurons[6] * hiddenWeights[6][2] + hiddenNeurons[7] * hiddenWeights[7][2]
                    + hiddenWeights[8][2];
  outputNeurons[3] = hiddenNeurons[0] * hiddenWeights[0][3] + hiddenNeurons[1] * hiddenWeights[1][3]
                    + hiddenNeurons[2] * hiddenWeights[2][3] + hiddenNeurons[3] * hiddenWeights[3][3]
                    + hiddenNeurons[4] * hiddenWeights[4][3] + hiddenNeurons[5] * hiddenWeights[5][3]
                    + hiddenNeurons[6] * hiddenWeights[6][3] + hiddenNeurons[7] * hiddenWeights[7][3]
                    + hiddenWeights[8][3];

  OutputVals[0] = (uint32_t)round((50.0 - 25.0)*(outputNeurons[0] + 1.0)/(1.0 + 1.0) + 25.0);
  OutputVals[1] = (uint32_t)round((50.0 - 25.0)*(outputNeurons[1] + 1.0)/(1.0 + 1.0) + 25.0);
  OutputVals[2] = (1.0 + 1.0)*(outputNeurons[2] + 1.0)/(1.0 + 1.0) - 1.0;
  OutputVals[3] = (1.0 + 1.0)*(outputNeurons[3] + 1.0)/(1.0 + 1.0) - 1.0;
  
}

float tansig(float value) {
  float wyk = (-2.0) * value;
  return (2.0 / (1.0 + exp(wyk))) - 1.0;
}

