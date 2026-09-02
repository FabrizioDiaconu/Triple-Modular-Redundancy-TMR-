/*
===============================================================================
                    Triple Modular Redundancy (TMR)
                            Second Node 
                        -by Diaconu Fabrizio-
===============================================================================
*/
// Including all the libraries needed
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>
// Defining all the CAN bus mcp2515 pins and constants needed
#define CS_PIN 5
#define SCK_PIN 18
#define MOSI_PIN 23
#define MISO_PIN 19
#define INT_PIN 4 
#define FIRST_ID 0x100
#define SECOND_ID 0x200
#define THIRD_ID 0x300
#define DATA_LENGTH 4 // This is the CAN message data length 
#define SENDING_TIME 20 // I used 20 ms as a time reference to send messages
// Defining the MPU6050 sensor's pin
#define SDA_PIN 21
#define SCL_PIN 22
// Defining the MPU6050 I2C address
#define MPU_I2C_ADDR 0x68 
// Defining all the servomotor pins and constants needed
#define SERVO_PIN 25
#define RESTING_POSITION 90
// Defining the led pin that shows if the system is on or off
#define ONOFF_LED_PIN 14
// Defining the button pin and constants needed
#define BUTTON_PIN 27
#define BUTTON_TIME 100 //I used 100 ms as a time reference to avoid any problems with the button
// Defining the calibration led pin and constants needed
#define CALIBRATION_LED_PIN 15
#define CALIBRATION_SAMPLES 1000
#define CALIBRATION_TIME 2 // I used 2 ms as a time reference for calibrating the sensor
// Defining the warning constants
#define RESTARTING_TIME 2000
#define SECOND_MPU_WARNING_LED_PIN 32
#define COMMUNICATION_WARNING_LED_PIN 33
#define MCP_CONTROL_TIME 500
// Defining the length of the array where the three readings of the MPUs will be in
#define ARRAY_LENGTH 3

MCP2515 secondMCP(CS_PIN);// Creating an instance for the can bus module
// Declaring can frames
struct can_frame txMsg; // Transmitting frame
struct can_frame rxMsg; // Receiving frame
union Sender // Union for the sent value
{
    float readingValueTX; // Float value of the data sent
    uint8_t readingValueBytesTX[DATA_LENGTH]; /* Value of the data sent in bytes
                                                 I used uint8_t to avoid the little/big endian problem,
                                                 which would've occurred in the for loop */
};
Sender secondCAN_data;
union Receiver // Union for the received value
{
    float readingValueRX; // Float value of the data received
    uint8_t readingValueBytesRX[DATA_LENGTH]; /* Value of the data received in bytes
                                                 I used uint8_t to avoid the little/big endian problem,
                                                 which would've occurred inside the for loop */
};
Receiver firstCAN_data, thirdCAN_data;

Adafruit_MPU6050 secondMPU;// Creating an instance for the mpu6050 sensor

Servo secondServo;// Creating an instance for the servomotor

unsigned long int currentClockTime = 0; /* Variable used to create a time controlled system, 
                                           this allowed to avoid the delay which would have
                                           blocked the code. */ 
int iterations1 = 0, iterations2 = 0;
bool button = LOW, oneShotButton = LOW, buttonMemory = LOW;
unsigned long int lastButtonTime = 0; /* Variable that checks if the button has been
                                         pressed after the chosen time gap */
bool ledState = LOW; // Led's on/off 
float inclination; // Float value based on the readings of the three sensors 
int servoAngle;
volatile bool messageReceived = false; 
unsigned long int lastSendingTime = 0; // Variable which allows the MCP to send a message every 20ms
float sum = 0, offset = 0; // Variables used to calculate the offset
int counter = 0;
bool isCalibrated = false;
long unsigned int lastCalibrationTime = 0;
// Variables made for the fault tolerance system regarding this ESP32
bool secondMPUWorking = false, secondMPUReading = false, warningLed = LOW;
long unsigned int lastRestartingTime = 0;
// Variables made for the fault tolerance system regarding the other ESP32s
long unsigned int firstCANTime = 0, thirdCANTime = 0;
bool firstMPUWorking = false, thirdMPUWorking = false;
bool communicationWarningLed = LOW;
float dataArray[ARRAY_LENGTH]; // This array holds the values of the three readings made by the MPUs,
float sortedArray[ARRAY_LENGTH]; /* This is the array that holds the sorted values
                                    of the first one, I used it to calculate the correct inclination */
float temporaryValue; // Variable used in the bubblesort algorithm
/* CAN interrupt service routine
  Sets the flag when a CAN message 
  interrupt occurs */
void IRAM_ATTR canISR()
{
    messageReceived = true;
}

void setup()
{
    Serial.begin(115200); // Setting monitor speed
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    secondMCP.reset();
    secondMCP.setBitrate(CAN_125KBPS, MCP_8MHZ);
    secondMCP.setNormalMode();
    pinMode(INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), canISR, FALLING);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setTimeOut(3); /* Setting 3 ms as a time reference for the mpu i2c communication,
                            that time passes by and the ESP32 hasn't received new data yet,
                            it will use the one from the other MPU*/
    // Check if the MPU6050 sensor is physically connected to the I2C bus
    Wire.beginTransmission(MPU_I2C_ADDR);
    if(Wire.endTransmission() == 0) 
    {
        secondMPUWorking = secondMPU.begin(); /* If the I2C communication is successful, 
                                        try to initialize the sensor library */
    } 
    else 
    {
        secondMPUWorking = false; // If the I2C transmission failed, mark the local MPU as not working
    }
    if(!secondMPUWorking) // Handling the fault tolerance
    {
        isCalibrated = true;  // Skip calibration since there is no sensor to calibrate
        warningLed = HIGH;    // Turn on the warning LED to signal the hardware fault
    }

    secondServo.attach(SERVO_PIN);
    secondServo.setPeriodHertz(50);

    pinMode(BUTTON_PIN, INPUT);
    pinMode(ONOFF_LED_PIN, OUTPUT);
    pinMode(CALIBRATION_LED_PIN, OUTPUT);
    pinMode(SECOND_MPU_WARNING_LED_PIN, OUTPUT);
    pinMode(COMMUNICATION_WARNING_LED_PIN, OUTPUT);
}

void loop()
{
    currentClockTime = millis();
    if(!isCalibrated) /* I created a calibration function to make sure
                         that the collected data is reliable */
    {
        if(counter < CALIBRATION_SAMPLES)
        {
            if(currentClockTime - lastCalibrationTime >= CALIBRATION_TIME) // Calibrating only if a certain time has passed
            {
                lastCalibrationTime = currentClockTime;
                sensors_event_t a, g, temp;
                secondMPU.getEvent(&a, &g, &temp);
                sum += a.acceleration.x;
                counter++;
            }
        }
        else
        {
            offset = sum / CALIBRATION_SAMPLES;
            isCalibrated = true;
            /* Reset the other nodes' timers now that calibration just finished,
               so they aren't immediately flagged as unresponsive */
            firstCANTime = millis();
            thirdCANTime = millis();
        }
        servoAngle = RESTING_POSITION; // while calibrating, the servomotor has to stay to its resting position
    }
    else
    {
        button = digitalRead(BUTTON_PIN);
        oneShotButton = LOW;
        if(button && (currentClockTime - lastButtonTime >= BUTTON_TIME)) /* Checking the if a certain time has passed by
                                                                            to be sure that no false contacts will
                                                                            activate the oneshot */
        {   
            lastButtonTime = currentClockTime;
            if(!buttonMemory)
            {
                oneShotButton = HIGH;
            }
        }
        buttonMemory = button;
        if(oneShotButton)
        {
            ledState = !ledState;
            // Avoid false timeout right after state change
            firstCANTime = millis();
            thirdCANTime = millis();
            lastSendingTime = millis();
        }
        if(ledState)
        {
            sensors_event_t a, g, temp;
            if(secondMPUWorking) // If the MPU was previously marked as working, try to read data
            {
                Wire.beginTransmission(MPU_I2C_ADDR); // Ping the sensor over I2C to ensure the physical connection is still there
                byte i2cError = Wire.endTransmission();
                if (i2cError == 0)
                {
                    secondMPUReading = secondMPU.getEvent(&a, &g, &temp); // Connection is ok, getting the data from the MPU
                }
                else
                {
                    // I2C bus error detected: trigger the fault recovery sequence
                    secondMPUReading = false;
                    warningLed = HIGH;   
                    secondMPUWorking = false;
                    lastRestartingTime = currentClockTime; // Saving timestamp for the next retry
                    // Resetting the I2C bus to unfreeze it from any hardware lockups
                    Wire.end(); 
                    Wire.begin(SDA_PIN, SCL_PIN);
                    Wire.setTimeOut(3);
                }
                // Handling the case where the I2C communication worked but data fetching failed
                if(!secondMPUReading)
                {
                    warningLed = HIGH;
                    secondMPUWorking = false;
                    secondMPUReading = false;
                    lastRestartingTime = currentClockTime;
                }
            }
            // If the MPU isn't currently working, attempt to reconnect it periodically
            else
            {
                /* I chosed to use a non-blocking timer, to avoid using the delay function,
                   the code is checking, every RESTARTING_TIME ms
                   if the I2C communication is working */
                if(currentClockTime - lastRestartingTime >= RESTARTING_TIME)
                {
                    lastRestartingTime = currentClockTime;
                    // Re-initializing the I2C peripheral
                    Wire.begin(SDA_PIN, SCL_PIN);
                    Wire.setTimeOut(3);
                    // Checking if the sensor answers again on the I2C bus
                    Wire.beginTransmission(MPU_I2C_ADDR);
                    byte i2cError = Wire.endTransmission();   
                    if(i2cError == 0)
                    {
                        if(secondMPU.begin()) // If the sensor responds, it has to re-initialize the library
                        {
                            secondMPUWorking = true;
                            warningLed = LOW; // Turning off the warning LED as the fault is resolved
                            secondMPU.getEvent(&a, &g, &temp);
                            secondMPUReading = true;
                        }
                    }
                    else 
                    {
                        // Reconnection failed, remain in fault state
                        secondMPUWorking = false;
                        secondMPUReading = false;
                    }
                }
            }
            if(secondMPUWorking && secondMPUReading)
            {
                /* If the I2C communication is fully working, 
                   save the reading in the structure */ 
                secondCAN_data.readingValueTX = (a.acceleration.x - offset);
            }
            else
            {
                secondCAN_data.readingValueTX = 999.0f; // If the MPU isn't working, send 999 as data
            }
            dataArray[1] = secondCAN_data.readingValueTX; // Setting the array's second value 
            if(currentClockTime - lastSendingTime >= SENDING_TIME) /* This makes the CAN send a message every 20 ms,
                                                                      I had to make this choice, otherwise the busses would've overflowed */
            {
                lastSendingTime = currentClockTime;
                txMsg.can_id = SECOND_ID; // Setting the sender ID
                txMsg.can_dlc = DATA_LENGTH; // Setting the message's data length 
                /* Serialize the float value by copying its byte value 
                  from the union into the CAN message data array */
                for(iterations1 = 0; iterations1 < DATA_LENGTH; iterations1++)
                {
                    txMsg.data[iterations1] = secondCAN_data.readingValueBytesTX[iterations1];
                }
                secondMCP.sendMessage(&txMsg);
                Serial.println("Messaggio inviato da esp2");
            }
            if(messageReceived || (digitalRead(INT_PIN) == LOW)) /* I had to use the second condition to avoid
                                                                     the program from freezing under certain conditions */
            {
                /* Temporarily disable interrupts to safely reset the flag
                 and prevent race conditions with the CAN ISR (Interrupt Service Routine) */
                noInterrupts();
                messageReceived = false;
                interrupts();
                while(secondMCP.readMessage(&rxMsg) == MCP2515::ERROR_OK)
                {
                    /* Deserialize the incoming CAN payload, reconstruct the float value 
                       by copying the bytes into the union structure */
                    if(rxMsg.can_id == FIRST_ID) // Checking if the relevated id matches 0x100
                    {
                        for(iterations1 = 0; iterations1 < DATA_LENGTH; iterations1++)
                        {
                            firstCAN_data.readingValueBytesRX[iterations1] = rxMsg.data[iterations1]; // Transfering the bytes to a local variable
                        }
                        firstCANTime = currentClockTime;
                        if(firstCAN_data.readingValueRX == 999.0f)
                        {
                            firstMPUWorking = false;
                        }
                        else
                        {
                            firstMPUWorking = true;
                            dataArray[0] = firstCAN_data.readingValueRX;
                            Serial.println("Messaggio ricevuto da esp1");
                        }
                    }
                    if(rxMsg.can_id == THIRD_ID) // Checking if the relevated id matches 0x300
                    {
                        /* Deserialize the incoming CAN payload, reconstruct the float value 
                           by copying the bytes into the union structure */
                        for(iterations1 = 0; iterations1 < DATA_LENGTH; iterations1++)
                        {
                            thirdCAN_data.readingValueBytesRX[iterations1] = rxMsg.data[iterations1]; // Transfering the bytes to a local variable
                        }
                        thirdCANTime = currentClockTime;
                        if(thirdCAN_data.readingValueRX == 999.0f)
                        {
                            thirdMPUWorking = false;
                        }
                        else
                        {
                            thirdMPUWorking = true;
                            dataArray[2] = thirdCAN_data.readingValueRX;
                            Serial.println("Messaggio ricevuto da esp3");
                        }
                    }
                }
            }
            /* The first node failed to send a new message within the 
               time limit, mark it as not working */
            if(currentClockTime - firstCANTime >= MCP_CONTROL_TIME)
            {
                firstMPUWorking = false;
                communicationWarningLed = HIGH;
            }
            /* The third node failed to send a new message within the 
               time limit, mark it as not working */
            if(currentClockTime - thirdCANTime >= MCP_CONTROL_TIME)
            {
                thirdMPUWorking = false;
                communicationWarningLed = HIGH;
            }
            // If both the nodes work, set the warning communication led off
            if(firstMPUWorking && thirdMPUWorking)
            {
                communicationWarningLed = LOW;
            }
            // === Triple Modular Redundancy (TMR) & voting logic ===
            if(!firstMPUWorking && !secondMPUWorking && !thirdMPUWorking) /* Fault detected: all of the three nodes aren't functional,
                                                                             set the servomotor at its resting position */
            {
                servoAngle = RESTING_POSITION;
            }
            else
            {
                if(firstMPUWorking && secondMPUWorking && thirdMPUWorking) /* If all the nodes are functional,
                                                                              start the voting logic */
                {
                    for(iterations1 = 0; iterations1 < ARRAY_LENGTH; iterations1++)
                    {
                        sortedArray[iterations1] = dataArray[iterations1];
                    }
                    /* Sorting all the values in the array using the bubblesort algorithm, once it's all done, 
                       choose the median value as the correct inclination of the platform */
                    for(iterations1 = 0; iterations1 < (ARRAY_LENGTH - 1); iterations1++)
                    {
                        for(iterations2 = 0; iterations2 < (ARRAY_LENGTH - iterations1 - 1); iterations2++)
                        {
                            if(sortedArray[iterations2] > sortedArray[iterations2 + 1])
                            {
                                temporaryValue = sortedArray[iterations2];
                                sortedArray[iterations2] = sortedArray[iterations2 + 1];
                                sortedArray[iterations2 + 1] = temporaryValue;
                            }
                        }
                    }
                    inclination = sortedArray[1]; // Giving the inclination its value
                }
                // === Fault Cases ===
                /*                      === First Case ===
                   If one of the three nodes aren't working, calculate the average value
                   of the other two */
                if(firstMPUWorking && secondMPUWorking && !thirdMPUWorking)
                {
                    inclination = (firstCAN_data.readingValueRX + secondCAN_data.readingValueTX) / 2.00f;
                }
                else if(firstMPUWorking && !secondMPUWorking && thirdMPUWorking)
                {
                    inclination = (firstCAN_data.readingValueRX + thirdCAN_data.readingValueRX) / 2.00f;
                }
                else if(!firstMPUWorking && secondMPUWorking && thirdMPUWorking)
                {
                    inclination = (secondCAN_data.readingValueTX + thirdCAN_data.readingValueRX) / 2.00f;
                }
                /*                      === Second Case ===
                   If two of the three nodes aren't working, choose to take the data 
                   from the only working MPU */
                else if(firstMPUWorking && !secondMPUWorking && !thirdMPUWorking)
                {
                    inclination = firstCAN_data.readingValueRX;
                }
                else if(!firstMPUWorking && secondMPUWorking && !thirdMPUWorking)
                {
                    inclination = secondCAN_data.readingValueTX;
                }
                else if(!firstMPUWorking && !secondMPUWorking && thirdMPUWorking)
                {
                    inclination = thirdCAN_data.readingValueRX;
                }
                // Calculating the servo angle which will stabilize the whole system
                servoAngle = 180 - (map((long)(inclination * 1000), -10000, 10000, 0, 180));
                servoAngle = constrain(servoAngle, 0, 180);
            }
        }
        else
        {
            servoAngle = RESTING_POSITION; /* if the system is in an off state the servomotor 
                                              stays in its resting position */
            warningLed = LOW;
            communicationWarningLed = LOW;
        }
    }
    // Writing the hardware outputs
    secondServo.write(servoAngle);
    digitalWrite(ONOFF_LED_PIN, ledState);
    digitalWrite(CALIBRATION_LED_PIN, !isCalibrated);
    digitalWrite(SECOND_MPU_WARNING_LED_PIN, warningLed);
    digitalWrite(COMMUNICATION_WARNING_LED_PIN, communicationWarningLed);
}