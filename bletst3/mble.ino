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
    
}

void sendData()
{
    unsigned char buf[20];
    
    buf[0] = omgLeftToSend.b[0];
    buf[1] = omgLeftToSend.b[1];
    buf[2] = omgLeftToSend.b[2];
    buf[3] = omgLeftToSend.b[3];

    buf[4] = leftWheelDirection;
    
    buf[5] = omgRightToSend.b[0];
    buf[6] = omgRightToSend.b[1];
    buf[7] = omgRightToSend.b[2];
    buf[8] = omgRightToSend.b[3];

    buf[9] = rightWheelDirection;

    bleuart.write(buf, sizeof(buf));

}



