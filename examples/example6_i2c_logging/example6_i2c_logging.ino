/*
  example6-I2C Logging

  This example shows how to read the accelerometer as fast as the I2C bus will allow,
  and then log that data to SD card.

  Based on original code by c_feldman
  Reworked by Paul Clark @ SparkFun Electronics, July 2023

  Running the I2C bus at 400kHz, we should be able to run the accelerometer at 1600Hz.
  But, at that rate, we do not have time to check dataReady via I2C. We need to use the
  accelerometer INT1 pin to control the timing.

  Products:

  SparkFun Triple Axis Accelerometer Breakout - KX132:
    https://www.sparkfun.com/products/17871

  SparkFun Triple Axis Accelerometer Breakout - KX134:
    https://www.sparkfun.com/products/17589


  Repository:

    https://github.com/sparkfun/SparkFun_KX13X_Arduino_Library

  SparkFun code, firmware, and software is released under the MIT
  License  (http://opensource.org/licenses/MIT).
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SparkFun_KX13X.h>

#define ACCEL_INT 4 // Define which GPIO pin the accelerometer INT1 pin is connected to. Change this to match your hardware
#define LED_PIN LED_BUILTIN // LED GPIO pin - change this if required
#define SD_CS SS // SD Card Chip Select pin - change this if required
//#define SD_CS 5 // On the SparkFun ESP32 Thing Plus C, CS is pin 5
#define myWire Wire // Define which Wire port to use - change this if required
#define LENGTH_UNSIGNED_LONG 11 // Define the length of an unsigned long in chars, plus a null terminator
#define LENGTH_MILLIS_MICROS (LENGTH_UNSIGNED_LONG + 4) // millis plus decimal point and 3 decimal places of micros

// Define the output data rate
//  Possible values are:
//  0  : 0.781Hz
//  1  : 1.563Hz
//  2  : 3.125Hz
//  3  : 6.25Hz
//  4  : 12.5Hz
//  5  : 25Hz
//  6  : 50Hz
//  7  : 100Hz
//  8  : 200Hz
//  9  : 400Hz
//  10 : 800Hz
//  11 : 1600Hz
//  12 : 3200Hz
//  13 : 6400Hz
//  14 : 12800Hz
//  15 : 25600Hz
#define ODR 11 // 1600Hz works well. 3200Hz ~works but the sampling rate is not regular (on ESP32)

SparkFun_KX134 kxAccel;

File dataFile;
bool recording = false; // Flag to indicate if recording is active
unsigned long startMillis = 0;

const int BATCH_SIZE = 100;  // Number of data entries to accumulate before writing to SD
struct timedOutputData {
  char time[LENGTH_MILLIS_MICROS]; // This will hold millis.micros in text format
  outputData data;
};
timedOutputData dataBatch[BATCH_SIZE];  // Array to store data entries
int dataCount = 0;  // Counter for the number of data entries accumulated

// Flag to show that an accelerometer interrupt has been seen
// Set to true initially just in case the first rising edge is missed
volatile bool accelerometerInterruptFlag = true;

void accelerometerISR() // Accelerometer Interrupt Service Routine
{
  accelerometerInterruptFlag = true;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Turn the LED off

  Wire.begin(); // Initialize I2C communication
  Wire.setClock(400000); // Increase the bus clock speed to 400kHz

  delay(1000);

  Serial.begin(115200); // Initialize serial communication with baud rate 115200
  while (!Serial)
    delay(50);

  Serial.println("KX134 Logging Example");

  //Set up SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed! Freezing.");
    while (true) {
      ; // Infinite loop to halt the program if SD card initialization fails
    }
  }

  // Prompt for creating a new file
  bool createNewFile = promptCreateNewFile();

  if (createNewFile) {
    createUniqueFile();
  } else {
    openExistingFile();
  }

  // Write header for acceleration data in CSV file
  if (recording && dataFile) {
    dataFile.println("Time,X,Y,Z");
    dataFile.flush();
  }

  // Set up the accelerometer interrupt pin
  // Use a jumper wire to connect the accelerometer INT1 pin to ACCEL_INT
  pinMode(ACCEL_INT, INPUT);
  attachInterrupt(ACCEL_INT, &accelerometerISR, RISING); // Interrupt on the rising edge of INT1

  //ACCELEROMETER STARTUP PROCEDURE
  if (!kxAccel.begin(myWire))
  {
    Serial.println("Could not communicate with the the KX13X. Freezing.");
    while (1)
      ;
  }
  Wire.setClock(400000);
  kxAccel.initialize(INT_SETTINGS);     // This is the only way to set the KX13x to high-performance mode (through the library)
  kxAccel.enableAccel(false);           // Disable the accelerometer while we continue to configure it
  kxAccel.setRange(SFE_KX134_RANGE64G); // Set the tange to 64g
  kxAccel.enableDataEngine();           // Enables the bit that indicates data is ready.
  kxAccel.enablePhysInterrupt();        // Enables interrupt pin 1
  kxAccel.routeHardwareInterrupt(0x10); // Routes the data ready bit to pin 1
  kxAccel.setOutputDataRate(ODR);       // Set the output data rate
  kxAccel.enableAccel();                // Finally enable the accelerometer
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "start") {
      startRecording();
    } else if (command == "stop") {
      stopRecording();
    }
  }

  if (recording) {
    // Check for the accelerometer interrupt
    if (accelerometerInterruptFlag) {
      accelerometerInterruptFlag = false; // Clear the flag
      
      // Log the data to the batch array
      timedOutputData timedData;
      rawOutputData myRawData;
      kxAccel.getRawAccelRegisterData(&myRawData);
      kxAccel.convAccelData(&timedData.data, &myRawData); // Manually convert the raw data to floating point
      strncpy(timedData.time, getCurrentDateTime(), sizeof(timedData.time));
      // Store the data entry in the batch array
      dataBatch[dataCount] = timedData;
      dataCount++;

      // Check if the batch is full
      if (dataCount >= BATCH_SIZE) {
        flushDataBatch();
      }
    }
  }
}

// Function to start recording
void startRecording() {
  if (!recording) {
    recording = true;
    startMillis = millis();
    Serial.println("Recording started.");

    // Write a note indicating the start of recording
    if (dataFile) {
      dataFile.println("Recording started");
      dataFile.flush(); // Flush the data to the file
    }
  }
}

// Function to stop recording
void stopRecording() {
  if (recording) {
    recording = false;
    Serial.println("Recording stopped.");

    // Write a note indicating the stop of recording
    if (dataFile) {
      dataFile.println("Recording stopped");
      dataFile.flush(); // Flush the data to the file
    }
  }
}

void createUniqueFile() {
  // Check if a file with the same name already exists
  int fileNumber = 0;
  while (SD.exists("/data_" + String(fileNumber) + ".csv")) {
    fileNumber++;
  }

  // Create a new file with a unique name
  String fileName = "/data_" + String(fileNumber) + ".csv";
  dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile) {
    Serial.println("New file created: " + fileName);

    // Write column labels
    dataFile.println("Time,X,Y,Z");
    dataFile.flush(); // Flush the data to the file
  } else {
    Serial.println("Error creating new file!");
  }
}

// Function to open the existing file
void openExistingFile() {
  dataFile = SD.open("/data.csv", FILE_WRITE);
  if (dataFile) {
    Serial.println("Existing file opened: data.csv");
    // Move the file pointer to the end of the file
    dataFile.seek(dataFile.size());
  } else {
    Serial.println("Error opening file for appending!");
  }
}

// Function to prompt for creating a new file
bool promptCreateNewFile() {
  while (true) {
    Serial.println("Do you want to create a new file? (Y/N)");
    while (!Serial.available()) {
      // Wait for input
    }

    char input = Serial.read();
    if (input == 'Y' || input == 'y') {
      return true;
    } else if (input == 'N' || input == 'n') {
      return false;
    }
  }
}

// Return millis in text format
const char * getCurrentDateTime() {
  static char elapsedMillisStr[LENGTH_MILLIS_MICROS]; //This holds the elapsed millis.micros as text. We return a pointer to it
  
  unsigned long currentMicros = micros();
  unsigned long currentMillis = millis();
  unsigned long elapsedMillis = currentMillis - startMillis;

  // Format the elapsed milliseconds as a string
  snprintf(elapsedMillisStr, sizeof(elapsedMillisStr), "%lu.%03lu", elapsedMillis, currentMicros % 1000);

  return (const char *)elapsedMillisStr;
}

void flushDataBatch() {
  // Log the data batch to the file
  digitalWrite(LED_PIN, HIGH); // Turn the LED on
  for (int i = 0; i < dataCount; i++) {
    dataFile.print(dataBatch[i].time);
    dataFile.print(",");
    dataFile.print(dataBatch[i].data.xData, 4);
    dataFile.print(",");
    dataFile.print(dataBatch[i].data.yData, 4);
    dataFile.print(",");
    dataFile.println(dataBatch[i].data.zData, 4);
  }
  dataFile.flush(); // Flush the data to the file
  digitalWrite(LED_PIN, LOW); // Turn the LED off

  // Reset the counter for the next batch
  dataCount = 0;
}
