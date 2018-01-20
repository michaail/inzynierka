#define ARM_MATH_CM4

#include <bluefruit.h>
#include <arm_math.h>

// ****** SILNIKI ****** //

//enable pins
#define PWM1 14
#define PWM2 16

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
float mv_per_lsb = 3600.0f/1024.0f; // 10-bit ADC z 3.3V input range

// flaga jazdy na wprost
bool straightF = 0;

void setup()
{
    //pinMode(PWM1, OUTPUT);
    //pinMode(PWM2, OUTPUT);

    pinMode(INA1, OUTPUT);
    pinMode(INB1, OUTPUT);

    pinMode(INA2, OUTPUT);
    pinMode(INB2, OUTPUT);

    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);

    digitalWrite(INA2, HIGH);
    digitalWrite(INB2, LOW);

    HwPWMx[0]->addPin(PWM1);
    HwPWMx[0]->addPin(PWM2);

    HwPWM0.begin();
    HwPWM0.setResolution(8);
    HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_1);

    const int maxValue = bit(8)-1;
    HwPWMx[0]->writePin(PWM1, maxValue, false);
    HwPWMx[0]->writePin(PWM2, maxValue, false);
    straightF = 1;
}

bool digitalizeAnalog(int analogValue)
{
    if(analogValue < 500)
    {
        return false;
    }
    else(analogValue >= 500)
    {
        return true;
    }
}


void loop()
{
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



}