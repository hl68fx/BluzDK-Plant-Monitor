bool sendResponse = false;
void dataCallbackHandler(uint8_t *data, uint16_t length) {
    sendResponse = true;

}

byte powerPin = D6;
byte LEDpin = D7;
byte moisturePin = A1;

byte uvPin = A3;
byte lightPin = A2;

long uv = 0.0;
float light = 0.0;
float temp = 0.0;
float pressure = 0.0;



#define TIME_TO_WAIT 60000 /* milliseconds */

typedef enum {
    WAIT_ONLINE,
    ALIVE,
    PUBLISH,
    SET_TIMER,
    WAIT,
    SWITCH_POWER_ON,
    SWITCH_POWER_OFF,
    POWER_WAIT,
    READ_SENSORS,
    CONVERT
} FSM_state_t;

void setup()
{   
    BLE.registerDataCallback(dataCallbackHandler);

	//InitializeBMP085();

    pinMode(D3, OUTPUT);
    digitalWrite(D3, LOW);
    pinMode(powerPin, OUTPUT);
    digitalWrite(powerPin, LOW);
}

void loop() {

    static FSM_state_t myState = WAIT_ONLINE;
    static uint32_t saveTime;
    static uint32_t switchedON;
    static uint32_t moisture;
    static byte rsp[2];
    
    switch (myState) {

        case WAIT_ONLINE: // stay in this state  until we're connected to the big old cloud in the sky
            if (Particle.connected()) myState = ALIVE;
            break;

        case ALIVE: // we're alive! shout about it
            Particle.publish("Came online at", String(millis()) );
            myState = SET_TIMER; // next time through loop(), we'll set up our custom timer
            break;

        case PUBLISH: // it's been TIME_TO_WAIT milliseconds -- make some noise!
            if ( !Particle.connected() ) { // first though ... did we get disconnected somehow?
                myState = WAIT_ONLINE;
            } else {

                BLE.sendData(rsp, 2);

                Particle.publish("PING! at ", String(millis()) );
                myState = SET_TIMER;
            }
            break;

        case SET_TIMER: // record the current time for the WAIT state to reference back to
            saveTime = millis();
            myState = WAIT;
            break;

        case WAIT: // stay in this state until TIME_TO_WAIT milliseconds have gone by since the SET_TIMER state
            if ( millis() > (saveTime + TIME_TO_WAIT) ) {
                
                // Time's up! But we'll drop out of loop() for and get it done next time through
                // This allows all the background network stuff the best chance of keeping up with business
                myState = SWITCH_POWER_ON; 
            }
            break;
            
        case SWITCH_POWER_ON:
            digitalWrite(powerPin, HIGH);
            switchedON = millis();
            myState = POWER_WAIT;
            break;
            
        case SWITCH_POWER_OFF:
            digitalWrite(powerPin, LOW);
            myState = CONVERT;
            break;
            
        case POWER_WAIT:
            if( millis() > (switchedON + 1000)) {
                
                myState = READ_SENSORS;
            }
            break;
        
        case READ_SENSORS:
            moisture = readMoisture(); 
            
            //uv = readUV();
        
            //light = readLight();
        
            //temp = readTemp();
        
            //pressure = readPressure();
            
            myState = SWITCH_POWER_OFF;
            
            break;
            
        case CONVERT:
            
            convertToHex(moisture, rsp);
            myState = PUBLISH;
            break;
        
        default:
            myState = WAIT_ONLINE;
    }          

    // We can save MUCH more battery drain this way too! ...
    System.sleep(SLEEP_MODE_CPU);

}

/*

void loop() 
{
    
    //this, and only this...
    System.sleep(SLEEP_MODE_CPU);
    
    unsigned long currentMillis = millis();
  
    if(currentMillis - previousMillis >= interval)
    {
        digitalWrite(powerPin, HIGH);
        
        if(currentMillis - previousMillis >= (interval + 1000) && Particle.connected)
        {
            int moisture = readMoisture(); 
            Particle.publish("influx_write", String(moisture), PRIVATE);

            digitalWrite(powerPin, LOW);
            
        
            //temp = readTemp();
            //Particle.publish(String(moisture));
            
            //Particle.publish("Plant", String::format("{\"Moisture\":%.0d}", moisture), 60, PRIVATE); 
            byte rsp[2];
            
            convertToHex(moisture, rsp);

            previousMillis = currentMillis;
        }
    }
}
*/

void convertToHex(int value, byte pdata[]) {

  pdata[0] = (value & 0x00ff);
  pdata[1] = (value & 0xff00)>>8;

  return;
}

int readMoisture() {
    return analogRead(moisturePin);
}

float readUV() {
    int sensorValue;
        
    long data = 0.0;
    for(int i=0;i<1024;i++)
    {  
        sensorValue=analogRead(uvPin);
        data +=sensorValue;
        delay(2);
    }   
    data = data >> 10;
    //Serial.print("The voltage value:");
    data = data * 4980.0 / 1023.0;
    //Serial.println("mV");
    return data;
}

float readLight() {
    int sensorValue = analogRead(lightPin); 
    return (float)(1023-sensorValue)*10/sensorValue;
}
/*
float readTemp() {
    return bmp.readTemperature();
}

float readPressure() {
    return bmp.readPressure();
}

// Initialize BMP085
void InitializeBMP085(){
	if (!bmp.begin()) {
		//Serial.println("Could not find a valid BMP085 sensor, check wiring!");
		while (1) {}
	}
}*/
