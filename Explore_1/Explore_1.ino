#include <ZumoShield.h>
#include <HCSR04.h>
#include <Wire.h>

#define LED_PIN 13

#define SPEED           200 // Maximum motor speed when going straight; variable speed when turning
#define LOW_SPEED       100 // low speed seeting

#define TURN_BASE_SPEED 100 // Base speed when turning (added to variable speed)


#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate


// Allowed deviation (in degrees) relative to target angle that must be achieved before driving straight
#include <HCSR04.h>
#define DEVIATION_THRESHOLD 5


UltraSonicDistanceSensor distanceSensor(4, 5);  //On bot, trig=4, echo=5
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
LSM303 compass;


void setup() {
  // Init Serial
  Serial.begin(9600);
  Serial.println("Opening Serial....");
  Serial.println("Press button to start");
  button.waitForButton();
  Serial.print("Object detected at ");
  Serial.println(distanceSensor.measureDistanceCm());
  Serial.println("Press button to start compass calibration");
  button.waitForButton();
  Serial.println("starting calibration");
  initCompass();
}

// Main loop
void loop() {
  float heading, relative_heading;
  // put your main code here, to run repeatedly:
  double obj_dist = 0;
  blinkWait();
  delay(1000);
  obj_dist = moveForward(5000, 200, 15.0);
  Serial.print("moved and object is ");
  Serial.println(obj_dist);
}


//*******Utilities******
// compass initialization
void initCompass(){
  // The highest possible magnetic value to read in any direction is 2047
  // The lowest possible magnetic value to read in any direction is -2047
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};
  unsigned char index;

  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initiate LSM303
  compass.init();

  // Enables accelerometer and magnetometer
  compass.enableDefault();

  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate

  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setLeftSpeed(SPEED);
  motors.setRightSpeed(-SPEED);

  for(index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

    Serial.println(index);

    delay(50);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  Serial.print("max.x   ");
  Serial.print(running_max.x);
  Serial.println();
  Serial.print("max.y   ");
  Serial.print(running_max.y);
  Serial.println();
  Serial.print("min.x   ");
  Serial.print(running_min.x);
  Serial.println();
  Serial.print("min.y   ");
  Serial.print(running_min.y);
  Serial.println();

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;

  button.waitForButton();
  
}

// led blinks until user button is pressed
void blinkWait(){
  while (1) {
    // blink LED
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      if (button.getSingleDebouncedRelease()){
          break;
      }
    }
}

// Move forward with Ultrasonic sensor
 double moveForward(int move_time, int move_speed, double min_dist)
{
  double dist;
  dist = distanceSensor.measureDistanceCm();
  if  (dist > min_dist) {
    int time_counter = 0;
    int time_step = 100;
    Serial.print("Moving fwd");
    motors.setSpeeds(SPEED, SPEED);
    while ((dist > min_dist) and (time_counter < move_time)) {
      delay(time_step);
      dist = distanceSensor.measureDistanceCm();
      time_counter += time_step;
    }
    motors.setSpeeds(0, 0);
    if (dist <= min_dist) {
      Serial.println("Object found");
    } else {
      Serial.print("Time out ");
      Serial.println(time_counter);
    }
  } else{
    Serial.print("Cannot move, object too close");
  }
  return dist;
}


// Turning same place to new heading
void turnToHeading(int heading_in, int target_heading, int speed) {
  
}


template <typename T> float heading(LSM303::vector<T> v)
{
  float x_scaled =  2.0*(float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}


// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
float averageHeading()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}
