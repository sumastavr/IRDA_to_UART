#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_AS7341.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <SparkFunMPU9250-DMP.h>

#include "SparkFun_VL53L1X.h"
//#include <vl53l1x_class.h>
//#include <vl53l1_error_codes.h>

#define DEBUG

// ToF Sensor initialization
//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 5
#define INTERRUPT_PIN 4

SFEVL53L1X distanceSensor;

// Environmental Sensor Initialization
Adafruit_BME280 bme; 

// Inertial measurement unit initialization
MPU9250_DMP imu;

Adafruit_AS7341 as7341;
uint16_t bufferLightReadings[12];

/*
 * Specify which protocol(s) should be used for decoding.
 * If no protocol is defined, all protocols are active
 * This must be done before the #include <IRremote.hpp>
 */


//#define NO_LED_FEEDBACK_CODE // saves 92 bytes program memory
#if FLASHEND <= 0x1FFF  // For 8k flash or less, like ATtiny85. Exclude exotic protocols.
#define EXCLUDE_EXOTIC_PROTOCOLS
#  if !defined(DIGISTUMPCORE) // ATTinyCore is bigger than Digispark core
#define EXCLUDE_UNIVERSAL_PROTOCOLS // Saves up to 1000 bytes program memory.
#  endif
#endif

#define EXCLUDE_UNIVERSAL_PROTOCOLS // Saves up to 1000 bytes program memory.
#define EXCLUDE_EXOTIC_PROTOCOLS // saves around 650 bytes program memory if all other protocols are active
//#define _IR_MEASURE_TIMING

#include <IRremote.hpp>

#define IR_RECEIVE_PIN  1

#if defined(APPLICATION_PIN)
#define DEBUG_BUTTON_PIN    APPLICATION_PIN // if low, print timing for each received data set
#else
#define DEBUG_BUTTON_PIN   6
#endif

// On the Zero and others we switch explicitly to SerialUSB
#if defined(ARDUINO_ARCH_SAMD)
#define Serial SerialUSB
#endif

//Uart mySerial (&sercom2, 6, 7, SERCOM_RX_PAD_1, UART_TX_PAD_2);

//void SERCOM2_Handler(){
//  mySerial.IrqHandler();
//}

bool panortilt = true;	   // true = adjusting the pan, while false = adjusting the tilt

const int socResetPin = 8; // reset pins connected to the SoC
const int socResetCode = 12;

const int togglePTCode = 8;
const int savePIDCode = 190;

const int increasePCode = 1;
const int increaseICode = 2;
const int increaseDCode = 3;

const int decreasePCode = 4;
const int decreaseICode = 5;
const int decreaseDCode = 6;

const int commandIncreasePan_P = 240;
const int commandIncreasePan_I = 241;
const int commandIncreasePan_D = 242;

const int commandDecreasePan_P = 243;
const int commandDecreasePan_I = 244;
const int commandDecreasePan_D = 245;

const int commandIncreaseTilt_P = 230;
const int commandIncreaseTilt_I = 231;
const int commandIncreaseTilt_D = 232;

const int commandDecreaseTilt_P = 233;
const int commandDecreaseTilt_I = 234;
const int commandDecreaseTilt_D = 235;

long timerReadTof=millis();
int intervalReadTof=100;

void setup() {

delay(10000);
Wire.begin();
pinMode(socResetPin, OUTPUT);
digitalWrite(socResetPin, HIGH);

#if FLASHEND >= 0x3FFF  // For 16k flash or more, like ATtiny1604. Code does not fit in program memory of ATtiny85 etc.
    pinMode(DEBUG_BUTTON_PIN, INPUT_PULLUP);
#endif

    Serial.begin(57600);
    Serial1.begin(57600);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
// Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

// In case the interrupt driver crashes on setup, give a clue
// to the user what's going on.
    Serial.println(F("Enabling IRin..."));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    //Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));

#if FLASHEND >= 0x3FFF  // For 16k flash or more, like ATtiny1604. Code does not fit in program memory of ATtiny85 etc.
    Serial.print(F("Debug button pin is "));
    Serial.println(DEBUG_BUTTON_PIN);

    // infos for receive
    Serial.print(RECORD_GAP_MICROS);
    Serial.println(F(" us is the (minimum) gap, after which the start of a new IR packet is assumed"));
    Serial.print(MARK_EXCESS_MICROS);
    Serial.println(F(" us are subtracted from all marks and added to all spaces for decoding"));
#endif

// ToF Sensor initialization process

  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1){
      Serial.println("TOF not connecting.");
      delay(1000);
    }
      
  } else {
    Serial.println("ToF Sensor online!");
    distanceSensor.startRanging();
  }

    unsigned status;
    status = bme.begin(0x76);  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(1000);
    }

  if (imu.begin() != INV_SUCCESS){
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

  if (!as7341.begin()){
    Serial.println("Could not find AS7341");
    while (1) { 
      Serial.println("CAN't initialise AS7341");
      delay(1000); 
      }
  }

  as7341.setATIME(100);
  as7341.setASTEP(999); //This combination of ATIME and ASTEP gives an integration time of about 1sec, so with two integrations, that's 2 seconds for a complete set of readings
  as7341.setGain(AS7341_GAIN_8X);
  
  as7341.startReading();

}

String getToFDistance(){
    //distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    
    while (!distanceSensor.checkForDataReady())
    {
        delay(1);
    }

    String distance = String(distanceSensor.getDistance()); //Get the result of the measurement from the sensor
    distance+="mm";
    //distanceSensor.clearInterrupt();
    //distanceSensor.stopRanging();

    return distance;

    /*
    Serial.print("Distance(mm): ");
    Serial.print(distance);

    float distanceInches = distance * 0.0393701;
    float distanceFeet = distanceInches / 12.0;

    Serial.print("\tDistance(ft): ");
    Serial.print(distanceFeet, 2);

    Serial.println();
    */
}

String getTemperatureStr(){
    String temp=String(bme.readTemperature(),2);
    temp+='C';
    return temp;
}

String getHumidityStr(){
    String temp=String(bme.readHumidity(),2);
    temp+='%';
    return temp;
}

String getPressureStr(){
    String temp=String(bme.readPressure(),2);
    temp+="Pa";
    return temp;
}

String getAltitudeStr(){
    #define SEALEVELPRESSURE_HPA (1013.25)
    String temp=String(bme.readAltitude(SEALEVELPRESSURE_HPA));
    temp+="M";
    return temp;
}

String getYawStr(){
    String temp=String(imu.yaw);
    temp+="Y";
    return temp;
}

String getPitchStr(){
    String temp=String(imu.pitch);
    temp+="P";
    return temp;
}

String getRollStr(){
    String temp=String(imu.roll);
    temp+="R";
    return temp;
}

void getLightSensorReadings(){
    uint16_t readings[12];
    as7341.getAllChannels(readings);  //Calling this any other time may give you old data

    for (int i=0;i<12;i++){
        bufferLightReadings[i]=readings[i]; 
    }
    
    as7341.startReading();
}

bool yourTimeOutCheck()
{
  //Fill this in to prevent the possibility of getting stuck forever if you missed the result, or whatever
  return false;
}

String getDateNow(){
    String dateNow="01-03-2022";

    return dateNow;
}

String getTimeNow(){
    String timeNow="10-23-45";

    return timeNow;
}

void loop() {

    /*
     * Check if received data is available and if yes, try to decode it.
     * Decoded result is in the IrReceiver.decodedIRData structure.
     *
     * E.g. command is in IrReceiver.decodedIRData.command
     * address is in command is in IrReceiver.decodedIRData.address
     * and up to 32 bit raw data in IrReceiver.decodedIRData.decodedRawData
     */

    if (IrReceiver.decode()) {
        Serial.println();
#if FLASHEND >= 0x3FFF  // For 16k flash or more, like ATtiny1604
        if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
            Serial.println(F("Overflow detected"));
            Serial.println(F("Try to increase the \"RAW_BUFFER_LENGTH\" value of " STR(RAW_BUFFER_LENGTH) " in " __FILE__));
            // see also https://github.com/Arduino-IRremote/Arduino-IRremote#modifying-compile-options-with-sloeber-ide
#  if !defined(ESP8266) && !defined(NRF5)
            /*
             * do double beep
             */
#    if !defined(ESP32)
            IrReceiver.stop(); // ESP32 uses another timer for tone()
#    endif
            tone(TONE_PIN, 1100, 10);
            delay(50);
            tone(TONE_PIN, 1100, 10);
            delay(50);
#    if !defined(ESP32)
            IrReceiver.start(100000); // to compensate for 100 ms stop of receiver. This enables a correct gap measurement.
#    endif
#  endif

        } else {
            // Print a short summary of received data
            IrReceiver.printIRResultShort(&Serial);

            if (IrReceiver.decodedIRData.protocol == UNKNOWN || digitalRead(DEBUG_BUTTON_PIN) == LOW) {
                // We have an unknown protocol, print more info
                IrReceiver.printIRResultRawFormatted(&Serial, true);
            }
        }

        // tone on esp8266 works once, then it disables the successful IrReceiver.start() / timerConfigForReceive().
#  if !defined(ESP8266) && !defined(NRF5)
        if (IrReceiver.decodedIRData.protocol != UNKNOWN) {
            /*
             * If a valid protocol was received, play tone, wait and restore IR timer.
             * Otherwise do not play a tone to get exact gap time between transmissions.
             * This will give the next CheckForRecordGapsMicros() call a chance to eventually propose a change of the current RECORD_GAP_MICROS value.
             */
#    if !defined(ESP32)
            IrReceiver.stop(); // ESP32 uses another timer for tone()
#    endif
            tone(TONE_PIN, 2200, 8);
#    if !defined(ESP32)
            delay(8);
            IrReceiver.start(8000); // to compensate for 8 ms stop of receiver. This enables a correct gap measurement.
#    endif
        }
#  endif
#else
        // Print a minimal summary of received data
        //IrReceiver.printIRResultMinimal(&Serial);
        //IrReceiver.printIRResultMinimal(&Serial1);
#endif // FLASHEND

        /*
         * !!!Important!!! Enable receiving of the next value,
         * since receiving has stopped after the end of the current received data packet.
         */

        IrReceiver.resume();

        /*
         * Finally check the received data and perform actions according to the received address and commands
         */
        if (IrReceiver.decodedIRData.address == 0) {
            
            //Serial1.write(IrReceiver.decodedIRData.command);
            //Serial.print(IrReceiver.decodedIRData.command);

            				// if code detected is special code for adjusting the PID of either the pan or the tilt
            if  (IrReceiver.decodedIRData.command == togglePTCode) 
			{
				                 // do something
				panortilt = !panortilt;
                delay(500);
				if (panortilt)
				{
					Serial.print("adjusting the pan");
				}
				else
				{
					Serial.print("adjusting the tilt");
				}
				            
			}
			 else if  (IrReceiver.decodedIRData.command == increasePCode) 
			{
				                 // send command to increase Proportional gain
				
                Serial.print("Increase Proportional");
                if (panortilt)
				{
					Serial1.write(commandIncreasePan_P);
				}
				else
				{
					Serial1.write(commandIncreaseTilt_P);
				}
				            
			}
			else if  (IrReceiver.decodedIRData.command == increaseICode) 
			{
				                 // send command to increase Integral gain
				Serial.print("Increase Integral");
                if (panortilt)
				{
					Serial1.write(commandIncreasePan_I);
				}
				else
				{
					Serial1.write(commandIncreaseTilt_I);
				}
				            
			}
			else if  (IrReceiver.decodedIRData.command == increaseDCode) 
			{
				                 // send command to increase derivative gain 
                Serial.print("Increase Derivative");
                if (panortilt)
				{
					Serial1.write(commandIncreasePan_D);
				}
				else
				{
					Serial1.write(commandIncreaseTilt_D);
				}
				         
			}
			else if  (IrReceiver.decodedIRData.command == decreasePCode) 
			{
				                 // send command to DECREASE Proportional gain
				Serial.print("Decrease Proportional");
                if (panortilt)
				{
					Serial1.write(commandDecreasePan_P);
				}
				else
				{
					Serial1.write(commandDecreaseTilt_P);
				}
				            
			}
			else if  (IrReceiver.decodedIRData.command == decreaseICode) 
			{
				                 // send command to DECREASE Integral gain
                Serial.print("Decrease Integral");
				if (panortilt)
				{
					Serial1.write(commandDecreasePan_I);
				}
				else
				{
					Serial1.write(commandDecreaseTilt_I);
				}
				            
			}
			else if  (IrReceiver.decodedIRData.command == decreaseDCode) 
			{
				                 // send command to DECREASE derivative gain 
                Serial.print("Decrease Derivative");
                if (panortilt)
				{
					Serial1.write(commandDecreasePan_D);
				}
				else
				{
					Serial1.write(commandDecreaseTilt_D);
				}
				         
			}
			else if  (IrReceiver.decodedIRData.command == socResetCode) 
			{
				Serial.print("Resetting SOC");
                digitalWrite(socResetPin, LOW);
				delay(200);
				digitalWrite(socResetPin, HIGH);
				   
			}
            else if  (IrReceiver.decodedIRData.command == savePIDCode) 
			{
				Serial.print("Save PID Constants");
                Serial1.write(IrReceiver.decodedIRData.command);
                Serial.print(IrReceiver.decodedIRData.command);
			}
			else
			{
				// if no specific functions code is detected then forward the code directly to SoC
				Serial1.write(IrReceiver.decodedIRData.command);
                Serial.print(IrReceiver.decodedIRData.command);
			}

        }
    } 
    
    // if (IrReceiver.decode())

    /*
     * Your code here
     * For all users of the FastLed library, use this code for strip.show() to improve receiving performance (which is still not 100%):
     * if (IrReceiver.isIdle()) {
     *     strip.show();
     * }
     */

        // Check for new data in the FIFO
    if ( imu.fifoAvailable() )
    {
        // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
        if ( imu.dmpUpdateFifo() == INV_SUCCESS)
        {
        // computeEulerAngles can be used -- after updating the
        // quaternion values -- to estimate roll, pitch, and yaw
        imu.computeEulerAngles();
        //printIMUData();
        }
    }

  bool timeOutFlag = yourTimeOutCheck();
  if(as7341.checkReadingProgress() || timeOutFlag ){
    if(timeOutFlag)
    {} //Recover/restart/retc.
    getLightSensorReadings();
    
/*
    Serial.print("ADC0/F1 415nm : ");
    Serial.println(readings[0]);
    Serial.print("ADC1/F2 445nm : ");
    Serial.println(readings[1]);
    Serial.print("ADC2/F3 480nm : ");
    Serial.println(readings[2]);
    Serial.print("ADC3/F4 515nm : ");
    Serial.println(readings[3]);
    Serial.print("ADC0/F5 555nm : ");  
    Serial.println(readings[6]);
    Serial.print("ADC1/F6 590nm : ");
    Serial.println(readings[7]);
    Serial.print("ADC2/F7 630nm : ");
    Serial.println(readings[8]);
    Serial.print("ADC3/F8 680nm : ");
    Serial.println(readings[9]);
    Serial.print("ADC4/Clear    : ");
    Serial.println(readings[10]);
    Serial.print("ADC5/NIR      : ");
    Serial.println(readings[11]); 
*/   

  }

    //if (millis()-timerReadTof>intervalReadTof){
    
    byte in = 0;
    if (Serial1.available()>0){
        in=Serial1.read();
    }
    
    if (in==97){

        Serial1.print(getToFDistance());
        Serial1.print(" : ");
        
        Serial1.print(getTemperatureStr());
        Serial1.print(" : ");

        Serial1.print(getHumidityStr());
        Serial1.print(" : ");

        Serial1.print(getPressureStr());
        Serial1.print(" : ");

        Serial1.print(getAltitudeStr());
        Serial1.print(" : ");

        Serial1.print(getYawStr());
        Serial1.print(" : ");

        Serial1.print(getPitchStr());
        Serial1.print(" : ");

        Serial1.print(getRollStr());
        Serial1.print(" : ");

        for (int i=0;i<12;i++){
            Serial1.print(bufferLightReadings[i]);
            Serial1.print(" : ");
        }
        
        Serial1.println();
        //Serial.println(millis()-timerReadTof);
        timerReadTof=millis();
    }

    // Debug enabled via Serial port to the computer
    // Otherwise only listen from the Serial 1 that is connected to the K210 SoC
    #ifdef DEBUG

        if (Serial.available()>0){
            in=Serial.read();
        }
        
        if (in==97){

            Serial.print(getToFDistance());
            Serial.print(" : ");
            
            Serial.print(getTemperatureStr());
            Serial.print(" : ");

            Serial.print(getHumidityStr());
            Serial.print(" : ");

            Serial.print(getPressureStr());
            Serial.print(" : ");

            Serial.print(getAltitudeStr());
            Serial.print(" : ");

            Serial.print(getYawStr());
            Serial.print(" : ");

            Serial.print(getPitchStr());
            Serial.print(" : ");

            Serial.print(getRollStr());
            Serial.print(" : ");

            for (int i=0;i<12;i++){
                Serial.print(bufferLightReadings[i]);
                Serial.print(" : ");
            }
            
            //Serial.print();
            Serial.print(millis());
            timerReadTof=millis();

            Serial.print(" : ");

            Serial.print(getDateNow());
            Serial.print(" : ");

            Serial.print(getTimeNow());
            Serial.println(" : ");

        }

    #endif

}

