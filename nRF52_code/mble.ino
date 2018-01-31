/*********************************************************************
 * Based on BLUEUart for Feather nRF52 by Adafruit 
 * 
 * bug_board_v1.0
 * 
 * Bluetooth conectivity handler file
 * 
 * by:
 * Dawid Borowczak
 * Michał Kłos
 * Adam Neubauer
 * Wiktor Siwek
 * 
*********************************************************************/

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


void GetControl(byte msg[])
{
    uint8_t val;
    
    if(msg[1] == 0x0A)
    {
        val = msg[0];
        ManualControl(val);
    }
}

void receiveData()
{
    //while ( bleuart.available() ) // tu nie może być while
    //{
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
        
    //}
    
}

void sendData()
{
    unsigned char buf[20];
    
    /*
    buf[0] = omgLeftToSend.b[0];
    buf[1] = omgLeftToSend.b[1];
    buf[2] = omgLeftToSend.b[2];
    buf[3] = omgLeftToSend.b[3];
    Serial.println(buf[0], HEX);
    Serial.println(buf[1], HEX);
    Serial.println(buf[2], HEX);
    Serial.println(buf[3], HEX);
    buf[4] = leftWheelDirection;
    Serial.println(buf[4], HEX);
    buf[5] = omgRightToSend.b[0];
    buf[6] = omgRightToSend.b[1];
    buf[7] = omgRightToSend.b[2];
    buf[8] = omgRightToSend.b[3];

    buf[9] = rightWheelDirection;
*/

    Serial.println(speLeft);
    Serial.println(speRight);

    buf[0] = speLeft[0];
    buf[1] = speLeft[1];
    buf[2] = speLeft[2];
    buf[3] = speLeft[3];
    buf[4] = speLeft[4];
    buf[5] = speLeft[5];
    
    

    buf[6] = 0x20;

    buf[7] = speRight[0];
    buf[8] = speRight[1];
    buf[9] = speRight[2];
    buf[10] = speRight[3];
    buf[11] = speRight[4];
    buf[12] = speRight[5];
    
    buf[13] = 0x20;

    buf[14] = char(leftWheelDirection);
    if (leftWheelDirection)
    {
        buf[14] = 0x31;
    }
    else
    {
        buf[15] = 0x30;
    }

    if (rightWheelControl)
    {
        buf[15] = 0x31;
    }
    else
    {
        buf[15] = 0x30;
    }
    buf[15] = char(rightWheelDirection);

    bleuart.write(buf, sizeof(buf));

}



