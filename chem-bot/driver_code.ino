#include "math.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_HMC5883.h"
#define PI 3.1415926535897932384626433832795

//Assign unique ID to the compass module:
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//Motor 1 Pins
int in1 = D4;
int in2 = D5;
//Motor 2 Pins
int in3 = D2;
int in4 = D3;
//Motor 1 - Speed control Pin 
int m1_speed = A5;
//Motor 2 - Speed control Pin
int m2_speed = A4;

//auxiliary pins
int s_1 = A3; //activate sensor power bus for forward sensors
int b_pin = D6; //boundary sensor trigger pin
int o_pin = D7; //Override pin - detects override button press
int comm1 = A0; //comms pin for inter-mcp comms

//Simple counter for debug
int loops;
int d_time;
int d_speed;
int d_dir;

void setup() {
    //set all motor drive pins to output mode
    pinMode(in1,OUTPUT);
    pinMode(in2,OUTPUT);  
    pinMode(in3,OUTPUT);
    pinMode(in4,OUTPUT);
    pinMode(m1_speeded0,OUTPUT);
    pinMode(m2_speed,OUTPUT);
    
    //check for compass online
    if(!mag.begin())
      {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Particle.publish("no HMC5883 detected", PRIVATE);
        while(1);
      }

    //Set auxiliary pins to operating state
    
    //set loops to 0 when firmware reloads
    loops = 0;
}

void drive(d_time, d_speed, d_dir) {
    
        //Driive forward at 'd_speed' speed for 'd_time' seconds:
        Particle.publish("Forward", PRIVATE);
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);
        digitalWrite(in3,HIGH);
        digitalWrite(in4,LOW);
    
        analogWrite(m1_speed,255);
        analogWrite(m2_speed,255);
        Particle.publish("5 second run", PRIVATE);
        delay(5000);
        
        Particle.publish("Now Turn Off", PRIVATE);
        analogWrite(speed0,0);
        analogWrite(speed1,0);
        delay(5000);
        
        Particle.publish("Now backwards", PRIVATE);
        digitalWrite(in1,LOW);
        digitalWrite(in2,HIGH);
        digitalWrite(in3,LOW);
        digitalWrite(in4,HIGH);
        
        analogWrite(speed0,255);
        analogWrite(speed1,255);
        Particle.publish("5 second run", PRIVATE);
        delay(5000);
       
        Particle.publish("Now Turn Off", PRIVATE);
        analogWrite(speed0,0);
        analogWrite(speed1,0);
        delay(5000);
        
        Particle.publish("Now Half Speed", PRIVATE);
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);
        digitalWrite(in3,HIGH);
        digitalWrite(in4,LOW);
    
        analogWrite(speed0,255);
        analogWrite(speed1,128);
        Particle.publish("5 second run", PRIVATE);
        delay(3000);
        
        Particle.publish("Now Turn Off", PRIVATE);
        analogWrite(speed0,0);
        analogWrite(speed1,0);
        delay(5000);
}
void compass_details() {
      sensor_t sensor;
      mag.getSensor(&sensor);
      Serial.println("------------------------------------");
      Serial.print  ("Sensor:       "); Serial.println(sensor.name);
      Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
      Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
      Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
      Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
      Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
      Serial.println("------------------------------------");
      Serial.println("");
      delay(500);
}
    
void get_current_heading() {
    /* Get a new sensor event */
    sensors_event_t event;
    mag.getEvent(&event);

    // Calculate heading when the magnetometer is level, 
    // then correct for signs of axis.
    float heading = atan2(event.magnetic.y, event.magnetic.x);

    // Add your 'Declination Angle' to your heading, 
    // ('Error' of the magnetic field in your location) 
    // Find yours here: http://www.magnetic-declination.com/ 
    float declinationAngle = 0.015;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if(heading < 0)
    heading += 2*PI;

    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
    heading -= 2*PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/PI;

    return headingDegrees
}

void loop() {

}