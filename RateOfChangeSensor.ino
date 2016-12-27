#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/*
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground
   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
   2015/NOV/13  - Added calibration save and restore
   */

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(1500);
}

/******/
/*
    Display some basic info about the sensor status
    */
/*****/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/******/
/*
    Display sensor calibration status
    */
/******/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
    */
/**************************************************************************/
int GREEN[] = {A5, A4, A3, A2, A1, A0};
int RED[] = {9, 8, 7, 6, 5, 4};
int YELLOW[] = {13};

void setup(void)
{

  pinMode(GREEN[0], OUTPUT);
  pinMode(GREEN[1], OUTPUT);
  pinMode(GREEN[2], OUTPUT);
  pinMode(GREEN[3], OUTPUT);
  pinMode(GREEN[4], OUTPUT);
  pinMode(GREEN[5], OUTPUT);
      
  pinMode(RED[0], OUTPUT);
  pinMode(RED[1], OUTPUT);
  pinMode(RED[2], OUTPUT);
  pinMode(RED[3], OUTPUT);
  pinMode(RED[4], OUTPUT);
  pinMode(RED[5], OUTPUT);
  pinMode(YELLOW[0],OUTPUT);
  
  

  
  
    //delay(20000);
    Serial.begin(115200);
    delay(1000);
    Serial.println("Orientation Sensor Test"); Serial.println("");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

    bno.setExtCrystalUse(true);

    sensors_event_t event;
    bno.getEvent(&event);
    if (foundCalib){
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
    else
    {
        Serial.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            Serial.print("X: ");
            Serial.print(event.orientation.x, 4);
            Serial.print("\tY: ");
            Serial.print(event.orientation.y, 4);
            Serial.print("\tZ: ");
            Serial.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            Serial.println("");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
    delay(1500);

}   


const int dlen = 12;     //number of samples averaged 4 is too short / 10 is better / 20 is to long
const int slen = 24;     //number of samples averaged 4 is too short / 10 is better / 20 is to long
// reduce to 4 for testing
double d[dlen] = {0};    // need a 0 for every sample to average
double s[slen] = {0};
void pushd(double val) {
  for (int i = 1; i < dlen; i++) {
    d[i-1] = d[i]; 
  }
  d[dlen - 1] = val;
}

double avgd() {
  double sum = 0;
  for (int i = 0; i < dlen; i++) {
    sum = sum + d[i];
  }
  double average = sum / (double)dlen;
  return average;
}

void pushs(double val) {
  for (int i = 1; i < slen; i++) {
    s[i-1] = s[i]; 
  }
  s[slen - 1] = val;
}

double avgs() {
  double sum = 0;
  for (int i = 0; i < slen; i++) {
    sum = sum + s[i];
  }
  double average = sum / (double)slen;
  return average;
}

void loop() {
    /* Get a new sensor event */
    
    sensors_event_t event;
    //bno.getEvent(&event);

    imu::Vector<3> v = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    //checkAccelerometer();
        
    /* Display the floating point data */
    //Serial.print("X: ");
    //Serial.print(v[0], 4);
    //Serial.print("\tY: ");
    //Serial.print(v[1], 4);
    //Serial.print("\tZ: ");
    //Serial.print(v[2], 4);
    
    double x = v[0]; // X accel
    double y = v[1]; // Y accel
    double sign = abs(x) / x; // Sign (+1 or -1)
    double ny = abs(y) * (abs(x) / x); // Normalized Y to X (Positive Y if X is positive, etc.)
    double nd = sqrt(pow(x, 2) + pow(ny, 2)) * sign * -1; // Distance from Origin with sign

if (isnan(nd)) {
  return;
}

    pushd(nd);
    double ad = avgd();
    pushs(ad);

// ROC
double left = 0;
double right = 0;
for (int i = 0; i < slen; i++) {
  if (i < slen / 2) {
    left += s[i];
  } else {
    right += s[i];
  }
}
right /= slen / 2;
left /= slen / 2;
ad = right - left;
if (isnan(ad)) { 
  ad = 0;
}
Serial.print(ad, 4);

    int q = (int)(ad * 1.0);

    int z = 100;

    int padLeft = q + (z/2);

    char str[z + 1];
    str[z] = (char)0;
    for (int i = 0; i < z; i++) {
      str[i] = '-';
    }

    str[z/2] = '|';
    if (padLeft >= 0 && padLeft < z) {
      str[padLeft] = 'X';
    }



    // if (ad == 0.0) {
    if (ad == 0.0) {
     // Nada 
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], LOW);
      digitalWrite(GREEN[2], LOW);
      digitalWrite(GREEN[1], LOW);
      digitalWrite(GREEN[0], LOW);      
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], LOW);
      digitalWrite(RED[3], LOW);
      digitalWrite(RED[4], LOW);
      digitalWrite(RED[5], LOW);
      digitalWrite(YELLOW[0], HIGH);
    
    
    } else if (ad <= -3.0) { 
      // Six reds
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], LOW);
      digitalWrite(GREEN[2], LOW);
      digitalWrite(GREEN[1], LOW);
      digitalWrite(GREEN[0], LOW);
      digitalWrite(RED[0], HIGH);
      digitalWrite(RED[1], HIGH);
      digitalWrite(RED[2], HIGH);
      digitalWrite(RED[3], HIGH);
      digitalWrite(RED[4], HIGH);
      digitalWrite(RED[5], HIGH);
      digitalWrite(YELLOW[0], LOW);

    } else if (ad <= -2.5) {
      // Five reds
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], LOW);
      digitalWrite(GREEN[2], LOW);
      digitalWrite(GREEN[1], LOW);
      digitalWrite(GREEN[0], LOW);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], HIGH);
      digitalWrite(RED[2], HIGH);
      digitalWrite(RED[3], HIGH);
      digitalWrite(RED[4], HIGH);
      digitalWrite(RED[5], HIGH);
      digitalWrite(YELLOW[0], LOW);
          
    } else if (ad <= -2.0) {
      // Four reds
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], LOW);
      digitalWrite(GREEN[2], LOW);
      digitalWrite(GREEN[1], LOW);
      digitalWrite(GREEN[0], LOW);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], HIGH);
      digitalWrite(RED[3], HIGH);
      digitalWrite(RED[4], HIGH);
      digitalWrite(RED[5], HIGH);
      digitalWrite(YELLOW[0], LOW);
    
    } else if (ad <= -1.5) {
      // Three reds
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], LOW);
      digitalWrite(GREEN[2], LOW);
      digitalWrite(GREEN[1], LOW);
      digitalWrite(GREEN[0], LOW);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], LOW);
      digitalWrite(RED[3], HIGH);
      digitalWrite(RED[4], HIGH);
      digitalWrite(RED[5], HIGH);
      digitalWrite(YELLOW[0], LOW);

    } else if (ad <= -1.0) {
      // two reds
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], LOW);
      digitalWrite(GREEN[2], LOW);
      digitalWrite(GREEN[1], LOW);
      digitalWrite(GREEN[0], LOW);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], LOW);
      digitalWrite(RED[3], LOW);
      digitalWrite(RED[4], HIGH);
      digitalWrite(RED[5], HIGH);
      digitalWrite(YELLOW[0], LOW);

     } else if (ad <= -0.50) {
      // one red
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], LOW);
      digitalWrite(GREEN[2], LOW);
      digitalWrite(GREEN[1], LOW);
      digitalWrite(GREEN[0], LOW);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], LOW);
      digitalWrite(RED[3], LOW);
      digitalWrite(RED[4], LOW);
      digitalWrite(RED[5], HIGH);
      digitalWrite(YELLOW[0], LOW);
      
    } else if (ad <= -.25) {
      // turn off reds 
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], LOW);
      digitalWrite(GREEN[2], LOW);
      digitalWrite(GREEN[1], LOW);
      digitalWrite(GREEN[0], LOW);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], LOW);
      digitalWrite(RED[3], LOW);
      digitalWrite(RED[4], LOW);
      digitalWrite(RED[5], LOW);
      digitalWrite(YELLOW[0], HIGH);

//      GREEN, GREEN, GREEN!
   
   
    } else if (ad >= 3.0) {
      // Six greens
      digitalWrite(GREEN[5], HIGH);
      digitalWrite(GREEN[4], HIGH);
      digitalWrite(GREEN[3], HIGH);
      digitalWrite(GREEN[2], HIGH);
      digitalWrite(GREEN[1], HIGH);
      digitalWrite(GREEN[0], HIGH);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], LOW);
      digitalWrite(RED[3], LOW);
      digitalWrite(RED[4], LOW);
      digitalWrite(RED[5], LOW);
      digitalWrite(YELLOW[0], LOW);

} else if (ad >= 2.5) {
      // Five greens
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], HIGH);
      digitalWrite(GREEN[3], HIGH);
      digitalWrite(GREEN[2], HIGH);
      digitalWrite(GREEN[1], HIGH);
      digitalWrite(GREEN[0], HIGH);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], LOW);
      digitalWrite(RED[3], LOW);
      digitalWrite(RED[4], LOW);
      digitalWrite(RED[5], LOW);
      digitalWrite(YELLOW[0], LOW);

} else if (ad >= 2.0) {
      // Four greens
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], HIGH);
      digitalWrite(GREEN[2], HIGH);
      digitalWrite(GREEN[1], HIGH);
      digitalWrite(GREEN[0], HIGH);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], LOW);
      digitalWrite(RED[3], LOW);
      digitalWrite(RED[4], LOW);
      digitalWrite(RED[5], LOW);
      digitalWrite(YELLOW[0], LOW);
      
} else if (ad >= 1.5) {
      // Three greens
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], LOW);
      digitalWrite(GREEN[2], HIGH);
      digitalWrite(GREEN[1], HIGH);
      digitalWrite(GREEN[0], HIGH);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], LOW);
      digitalWrite(RED[3], LOW);
      digitalWrite(RED[4], LOW);
      digitalWrite(RED[5], LOW);
      digitalWrite(YELLOW[0], LOW);
      
   } else if (ad >= 1.0) {
      // Two greens
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], LOW);
      digitalWrite(GREEN[2], LOW);
      digitalWrite(GREEN[1], HIGH);
      digitalWrite(GREEN[0], HIGH);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], LOW);
      digitalWrite(RED[3], LOW);
      digitalWrite(RED[4], LOW);
      digitalWrite(RED[5], LOW);
      digitalWrite(YELLOW[0], LOW);  

} else if (ad >= 0.50) {
      // One greens
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], LOW);
      digitalWrite(GREEN[2], LOW);
      digitalWrite(GREEN[1], LOW);
      digitalWrite(GREEN[0], HIGH);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], LOW);
      digitalWrite(RED[3], LOW);
      digitalWrite(RED[4], LOW);
      digitalWrite(RED[5], LOW);
      digitalWrite(YELLOW[0], LOW);

   } else if (ad >= .25) {
      // Trun off greens
      digitalWrite(GREEN[5], LOW);
      digitalWrite(GREEN[4], LOW);
      digitalWrite(GREEN[3], LOW);
      digitalWrite(GREEN[2], LOW);
      digitalWrite(GREEN[1], LOW);
      digitalWrite(GREEN[0], LOW);
      digitalWrite(RED[0], LOW);
      digitalWrite(RED[1], LOW);
      digitalWrite(RED[2], LOW);
      digitalWrite(RED[3], LOW);
      digitalWrite(RED[4], LOW);
      digitalWrite(RED[5], LOW);
      digitalWrite(YELLOW[0], HIGH);
     
    
    }
//    Serial.print(str);
     
//**********************************************************************************************
    // MY ADDITION

    /*
    Serial.print("\tXAcc:");
    Serial.print(xAcc);
    Serial.print("/tXGyro:");
    Serial.print(xGyro);
*/
//imu::Vector<3> Accel = bno.getAccel();
//  Serial.print("\tqW: ");
//  Serial.print(accel.x());
    
    /* Optional: Display calibration status */
//    displayCalStatus();

    /* Optional: Display sensor status (debug only) */
    //displaySensorStatus();

    /* New line for the next sample */
    Serial.println("");

    /* Wait the specified delay before requesting new data */
    delay(10);
    //delay(BNO055_SAMPLERATE_DELAY_MS);
}

