
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