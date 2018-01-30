/*********************************************************************
 * Based on BLUEUart for Feather nRF52 by Adafruit 
 * 
 * Neural Network file
 * 
 * by:
 * Dawid Borowczak
 * Michał Kłos
 * Adam Neubauer
 * Wiktor Siwek
 * 
*********************************************************************/

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

    //bleuart.write(t, sizeof(t));


    //(uint8_t)OutputVal[0];
    //uint8_t valueRight = (uint8_t)OutputVal[1];
    
    if(!leftWheelDirection && !rightWheelDirection)
    {
        F0 = false;
    }


    DriveNN(leftWheelDirection, valueLeft, rightWheelDirection, valueRight);

}

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
