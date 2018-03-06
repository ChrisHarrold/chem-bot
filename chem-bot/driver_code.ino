#include "math.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_HMC5883.h"
#define PI 3.1415926535897932384626433832795

//Assign unique ID to the compass module:
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//Motor 1 Pins
int M1_1 = D4;
int M1_2 = D5;
//Motor 2 Pins
int M2_1 = D2;
int M1-2 = D3;
//Motor 1 - Speed control Pin 
int m1_s_pin = A5;
//Motor 2 - Speed control Pin
int m2_s_pin = A4;

//other GPIO pins:
int power_bus = A3; //activate sensor power bus for forward sensors
int b_pin = D6; //boundary sensor trigger pin
int o_pin = D7; //Override pin - detects override button press
int comm1 = A0; //comms pin for inter-mcp comms
int err_pin = A1; //not the right pin, need to confirm
int hmc = D2; //pin for reading the digital compass (not sure this is right - just need to placehold it for now)

//program variables
//Simple counters for debug and monitor processes
int loops;
int p_check;
// d_time is how long the motor will run - specified in milisecods and using the delay() function
int d_time;
// d_speed is a value between 1 and 255 - 255 is FULL speed, 128 is half, 1 is an absolute crawl, 0 is stop
int d_speed;
// d_dir is a numeric value aligned to "left" and "right" 
// relative to the current heading- see how this is determined in the which_way() function
int d_dir;
// operate_mode tells the program if it's running in "demo" mode, versus full auto mode with complete functionality
int operate_mode;
// int Err is just for error tracking - 0 for none, 1 for errors
int Err;
// rel_dir is used inside the direction and drive function as a holder variable so I do not lose the original value
int rel_dir;
// int_d_distance is used to tell the robot how far to roll forward only - it is passed as 0 when turning, and in CM for
// straight-line movement - this will eventually be used in a "roll forward AND right" type equation.
int d_distance;
// int spd_bias is a preffered rate of speed based on other factors (like boundary, people detected, etc... this tells
// the robot to prefer a slower speed for operating in tight quarters - specified from 1 (ultra slow) to 10 (full speed))
int spd_bias;
//dict_speeds is a reference table for how fast the robot turns and how far - this is to simplify the heading corrections
// needed later (NO IDEA if DICT is a supported type in C++ - I assume not, but whatever multi variate array type will work)
dict dict_speeds;
//int m1 and m2_speed are the real values to set the rotational velocity of the motors
int m1_speed;
int m2_speed;
//int the_speed is another placeholder variable
int the_speed;
// const ROS is the Rate of Speed in a straight line for the robot
int ROS;
ROS = 50; //expressed as how many centimeters in a second (need to actually measure this)
// motor_bias is for situations where one motor goes a little faster than another!
int m1_motor_bias;
int m2_motor_bias;
// set them to match how much slower one should go than the other (it will subtract this from the raw speed value in
  //order to drop the ROS on the motor to be accurate for straight lines:
m1_motor_bias = 0;
m2_motor_bias = 0;

void setup() {
  // just started up so hopefully no errors yet! - Also reset all counters
  Err = 0;
  p_check = 0;
  loops = 0;    


    //check particle cloud online
    if(Particle.cloud()) 
    {
      Particle.publish("Drive Module is Preparing for operations", PRIVATE);
    }
    else {
      loop until p_check == 5{
        p_check = p_check + 1;
        delay(5000);
      }
      if(p_check == 5) {
        Err = 1;
      }
    }
    
    if(Err == 0) {
      //set all motor drive pins to output mode
      pinMode(in1,OUTPUT);
      pinMode(in2,OUTPUT);  
      pinMode(in3,OUTPUT);
      pinMode(in4,OUTPUT);
      pinMode(m1_s_pin,OUTPUT);
      pinMode(m2_s_pin,OUTPUT);
      pinMode(o_pin, INPUT);
      pinMode(comm1, OUTPUT);
      pinMode(S_1, OUTPUT);
      pinMode(hmc, scl); //do I even need this? Need to find the sample again  
      
      //turn on power bus for sensors and IR LED lights
      digitalWrite(power_bus, HIGH);
    }
    
    //check for override button press
    if(check_op())
      {
        //Checked the button - it is pressed - go to demo mode
        operate_mode = 1;
      }
    if(!check_op())
      {
        //not pressed - full auto mode enabled
        operate_mode = 0;
      }

    //check for compass online
    if(Err == 0) {
      if(!mag.begin())
        {
          /* There was a problem detecting the HMC5883 ... check your connections */
          Particle.publish("no HMC5883 detected", PRIVATE);
          while(1);
          Err = 1;
        }
    }

    //Final check and turn on ready light and notify
    if(Err == 0) 
    {
      digitalWrite(comm1, HIGH);
      delay(10000)
    }
    if(Err > 0)
    {
      digitalWrite(err_pin, HIGH);
      delay(10000);
      halt; // not sure this works, but it will be a hard stop to the rest of the program
    }

}

void check_op() {
  //need to find the right way to do this, but general gist is:
  // if button is pressed > pin is high > override default behavior - also will be used later to cancel
  // normal operations and exit the program loop
  if(o_pin == HIGH) 
  {
    return True;
  }
  else
  {
    return False;
  }
}

int which_way(int headingDegrees) {

  //heading is calculated in nearest cardinal direction with a granularity up to
  //2 levels (SSW for example). This is translated into a direction to turn by the 
  //function - this chart gives the numeric value (0 = North):
  //----------------
  //-  North = 0   -
  //-  NNE = 1     -
  //-  NE = 2      -
  //-  ENE = 3     -
  //-  East = 4
  //-  ESE = 5
  //-  SE = 6
  //-  SSE = 7
  //-  South = 8
  //-  SSW = 9
  //-  SW = 10
  //-  WSW = 11
  //-  West = 12
  //-  WNW = 13
  //-  NW = 14
  //-  NNW = 15
  //-              -
  //---------------- 
  
  // take heading degrees and turn it into a direction we need to turn to go right or left:
  // not sure how best to do this yet - maybe a tree-march from 350 down to 0? I will find
  // a good example

  // math that turns "50 degress heading" into a relative direction to turn:
  
  //math
  //more math
  //and all done

  return d_dir;
}

void f_drive(int d_dir, int d_distance, int spd_bias) {
  // calculate speed and time based on desired direction:
  // use relative speed chart for vlaues:
  // Rate Of Turn (ROT): 90 degrees per second at full speed (not verified yet)
  // Rate of Speed (ROS): 1 M/Second at full speed (not verified yet)
  // everything else is just math

  // from the d_dir, I know what direction I need to put myself in (relative to current heading)
  // if that direction is less than 9, I need to turn right - otherwise left (I could go either way for
  // South of course, but like Derrick Zoolander, I am not an ambi-turner :-) )

  if(d_dir < 9) {
    //turning right
    rel_dir = 1;
  }
  else {
    //turning left
    rel_dir = 2;
  }

  // now how far around that circle? -  I really just need a dict object here that gives me fixed
  // speed to direction values which is based on my ROT - I could calculate it every time but that seems
  // like code for the sake of code, when a reference table will do? This would take the form of:
  // the speed the motors need to be set at, and the time they need to run to turn to heading X.
  // regardless, should be easy enough to find an example to reference - something like:
  
  // if we are not going straight then we need to now how fast the motor should move to turn us to the new heading
  if(d_dir != 0) {
    for(d_dir in speed_list[]){
      m1_speed = speed_list[0];
      m2_speed = speed_list[1];
      d_time = speed_list[2]; 
      //It will have to be a multi-variate array or dict object so this is just a placeholder until I find the example code
    }
    // right turns require us to reverse the right motor (motor 1)
    if(d_dir < 9) {
      digitalWrite(M1_1,LOW);
      digitalWrite(M1_2,HIGH);
      digitalWrite(M2_1,HIGH);
      digitalWrite(M2_2,LOW);
    }
    if(d_dir > 8) {
      //left turns are the opposite - we reverse the left motor
      digitalWrite(M1_1,HIGH);
      digitalWrite(M1_2,LOW);
      digitalWrite(M2_1,LOW);
      digitalWrite(M2_2,HIGH);
    }
  }

  //Set the pin modes for moving the appointed direction
  // for forward motion, the order is High, Low)
  if(d_dir == 0) {
    digitalWrite(M1_1,HIGH);
    digitalWrite(M1_2,LOW);
    digitalWrite(M2_1,HIGH);
    digitalWrite(M2_2,LOW);
    int_thespeed = d_distance/ROS;
    m1_speed = int_thespeed - m1_motor_bias;
    m2_speed = int_thespeed - m2_motor_bias;
  }

  //now that all of that is established, I drive at the calculated speed for the specified time:
  // for turns the time is the time it takes my ROT to get me around to the new direction.
  // for straight lines it is how long it takes me to cover the desired distance in CM
  analogWrite(m1_s_pin, m1_speed);
  analogWrite(m2_s_pin, m2_speed);
  delay(d_time);
  
  //and then stop turning or moving by shutting off the motors:
  analogWrite(m1_s_pin,0);
  analogWrite(m2_s_pin,0);
  return True;
        
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
void f_alertpi() {
  // reports a command complete and driver ready for next instructions
  digitalWrite(comm1, HIGH);
  delay(10000)
}

void loop() {
  // First check override condition:
  if(operate_mode == 1) {
    Particle.publish("Going into Override mode - running demo", PRIVATE);
    //do just a simple turn and point demo
    get_current_heading(headingDegrees);
    f_drive(4, 0, 10);
    f_drive(0, 100, 10);
    
  }
  if(operate_mode == 0) {
    Particle.publish("Going into full operation mode - press and hold override to halt", PRIVATE);
    //full auto operation mode - will loop forever unless broken by the override button

    //step 1 - which way am I facing?
    get_current_heading();
    //step 2 - which way should I go? (this will come from the Visual Recognition code on the Pi and then be calculated
    // into a direction to turn)
    which_way(headingDegrees);
    // check for constraints and get the distance from the VR on the Pi:
    get_distance();
    get_bias();
    // now pass all of that to the drive function so I can move:
    f_drive(d_dir, d_distance, spd_bias);
    // when I get back here I will tell the Pi to sweep again:
    f_alertpi();
    // last step, take a look and see if the override button has been pressed:
    check_op();
    //if it has put us into shutdown mode:
    if(check_op()) {
      operate_mode = 3;
    }
  }
  //override button was pressed - initiate program halt:
  if(operate_mode == 3) {
    // if override is pressed and held during normal operation this will shut everything down
    Particle.publish("Received shutdown - halting program - reset driver module to reactivate", PRIVATE);
    exit(); 
  }
}