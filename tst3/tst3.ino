/* **********************************
Realizacja wykorzystania enkoderów
do sterowania obrotem kół robota Bug
Wykonywanie pełnego obrotu obu kół
Wyraźne opóźnienie jednego z kół

by: Michał Kłos @
https://github.com/michaail
*************************************/

#define ARM_MATH_CM4

#include <bluefruit.h>
#include <arm_math.h>

// ****** SILNIKI ****** //

//enable pins
#define PWM1 14
#define PWM2 13

// wejscia kierunkowe dla silnika 1
#define INA1 15
#define INB1 16

// wejscia kierunkowe dla silnika 2
#define INA2 12
#define INB2 11

// ***** ENKODERY ****** //

// piny wyjściowe z enkoderów
#define ELC A2
#define EPC A3

// liczba krawędzi na obrót koła
#define encResolution 32

// wartości analogowe enkoderów
int lEncIter = 0;
int pEncIter = 0;

// liczba obrotów kół
int lEncRevolutions = 0;
int pEncRevolutions = 0;

// poprzednie wartości enkoderów
bool lEncLastValue = 0;
bool pEncLastValue = 0;

// napięcie referencyjne ADC
float mv_per_lsb = 3000.0f/256.0f; // 8-bit ADC z 3.3V input range

const int maxValue = bit(8)-1;

// flaga jazdy na wprost
bool straightF = 0;

// flagi etapow
bool f1 = false;
bool f2 = false;

// chwile czasu
unsigned long t1 = 0;
unsigned long t2 = 0;

void setup()
{
    //pinMode(PWM1, OUTPUT);
    //pinMode(PWM2, OUTPUT);

    analogReadResolution(8); // range 0 - 255
    analogReference(AR_INTERNAL_3_0);
    delay(1);

    Serial.begin(115200);

    pinMode(INA1, OUTPUT);
    pinMode(INB1, OUTPUT);

    pinMode(INA2, OUTPUT);
    pinMode(INB2, OUTPUT);

    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);

    digitalWrite(INB2, HIGH);
    digitalWrite(INA2, LOW);

    HwPWMx[0]->addPin(PWM1);
    HwPWMx[0]->addPin(PWM2);

    HwPWM0.begin();
    HwPWM0.setResolution(8);
    HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_1);


    HwPWMx[0]->writePin(PWM1, maxValue, false);
    HwPWMx[0]->writePin(PWM2, maxValue, false);
    straightF = 1;

    //digitalWrite(PWM2, LOW);

    lEncLastValue = digitalizeAnalog(ELC);
    pEncLastValue = digitalizeAnalog(EPC);
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
    //delay(1);
    int vl = analogRead(ELC);
    int vp = analogRead(EPC);
    Serial.print("wartosc na L - wartosc: ");
    Serial.println(vl);
    
    Serial.print("wartosc na P - wartosc: ");
    Serial.println(vp);
//    delay(1);


if(lEncIter >= 31)
{
    if(!f1)
    {
        HwPWMx[0]->writePin(PWM1, 0, false);
        t1 = millis();
        f1 = true;
    }
    if(millis() - t1 > 2000)
    {
        HwPWMx[0]->writePin(PWM1, maxValue, false);
        lEncIter = 0;
        f1 = false;
    }
    
}

if(pEncIter >= 31)
{
  Serial.println(pEncIter);  
    if(!f2)
    {
        Serial.println("WESZLO**********");
        HwPWMx[0]->writePin(PWM2, 0, false);
        t2 = millis();
        f2 = true;
    }
    if(millis() - t2 > 2000)
    {
        Serial.println("********WYSZLO");
        HwPWMx[0]->writePin(PWM2, maxValue, false);
        pEncIter = 0;
        f2 = false;
    }
    
}

    if(digitalizeAnalog(vl)!= lEncLastValue)
    {
        lEncIter++;
        lEncLastValue = !lEncLastValue;
    }
    if(digitalizeAnalog(vp)!= pEncLastValue)
    {
        pEncIter++;
        pEncLastValue = !pEncLastValue;
    }

    

    //Serial.println(analogRead(ELC));
    /*
    if(straightF)
    {
        // odczyt wartości analogowych enkoderów
        if(digitalizeAnalog(analogRead(ELC)) != lEncLastValue)
        {
            lEncIter++;
            lEncLastValue = !lEncLastValue;
        }
        if(digitalizeAnalog(analogRead(EPC)) != pEncLastValue)
        {
            pEncIter++;
            pEncLastValue = !pEncLastValue;
        }
    
    
        if(lEncIter == 32)
        {


        }
        else if(pEncIter == 32)
        {


        }
    }

*/

}