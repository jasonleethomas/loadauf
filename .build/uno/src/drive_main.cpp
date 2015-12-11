#include <Arduino.h>
#include <Servo.h>
void setup();
void loop();
long sensor_read_distance(int echo_pin, int trigger_pin);
#line 1 "src/drive_main.ino"
//#include <Servo.h>

// note: all distances are in m

enum face { left, right, front };
int const sensor_count = 3; 
int const sensor_trigger_pin = 2;
int const sensor_echo_pins[sensor_count] = { 3, 4, 5 };
long sensor_echo_distances[sensor_count];

int const servo_count = 2;
Servo servo_controllers[servo_count];
int const servo_control_pins[sensor_count] = { 9, 10 }; 
int const servo_full_speeds[sensor_count] = { 70, 110 };
int const servo_base_speeds[sensor_count] = { 80, 100 };
int const servo_stop_speed = 90; 

// to-do: tune front (11/27/15)
float const pid_proportional[sensor_count] = { 0, 0, 2 };
float const pid_integral[sensor_count] = { 0, 0, 0 };
float const pid_derivative[sensor_count] = { 0, 0, 0 };

long const distance_boundary[sensor_count] = { 0, 0, 10 }; // centimeters

void setup()
{
  pinMode(sensor_trigger_pin, OUTPUT);  
  servo_controllers[left].attach(servo_control_pins[left], 1000, 2000);
  servo_controllers[right].attach(servo_control_pins[right], 1000, 2000);
  Serial.begin(9600);
}

void loop()
{
  long static time_then = 0;
  long time_now = millis() / 1000; 

  // time since last reading, seconds
  long time_delta = time_now - time_then; 

  time_then = time_now;

  // distance to nearest obstacle, centimeters 
  sensor_echo_distances[front] = sensor_read_distance(sensor_echo_pins[front], sensor_trigger_pin);
      
  long static distance_integral[sensor_count] = { 0, 0, 0 };
  long static distance_delta_then[sensor_count] = { 0, 0, 0 };

  // distance between boundary and obstacle 
  long distance_delta_now[sensor_count] = 
  { 
    0, 
    0,
    (sensor_echo_distances[front] - distance_boundary[front])
  };

  distance_integral[front] += (distance_delta_now[front] * time_delta); 

  // speed approaching obstacle, m/s
  long distance_derivative[sensor_count] = 
  {
    0,
    0,
    ((distance_delta_now[front] - distance_delta_then[front]) / time_delta)
  };

  distance_delta_then[front] = distance_delta_now[front];

  // change in speed to avoid obstacle using pid control 
  int speed_delta[sensor_count] = 
  {
    0,
    0,
    ((distance_delta_now[front] * pid_proportional[front]) +
    (distance_derivative[front] * pid_derivative[front])  +
    (distance_integral[front] * pid_integral[front]))
  };

  // change sign based on servo direction
  int servo_speeds[servo_count] = 
  {
    servo_base_speeds[left] - speed_delta[front],
    servo_base_speeds[right] + speed_delta[front]
  };

  // servos can only go so fast
  if (servo_speeds[left] < servo_full_speeds[left])
    servo_speeds[left] = servo_full_speeds[left];

  if (servo_speeds[right] > servo_full_speeds[right])
    servo_speeds[right] = servo_full_speeds[right];

  // stop if neccessary
  if (servo_speeds[left] >= servo_stop_speed)
    servo_speeds[left] = servo_stop_speed;

  if (servo_speeds[right] <= servo_stop_speed)
    servo_speeds[right] = servo_stop_speed;

  // update servos with speed changes 
  servo_controllers[left].write(servo_speeds[left]);
  servo_controllers[right].write(servo_speeds[right]);

  // print pid paracentimeters
  Serial.print("del_d: "); 
  Serial.println(distance_delta_now[front]);
  Serial.print("der_d: "); 
  Serial.println(distance_derivative[front]);
  Serial.print("int_d: "); 
  Serial.println(distance_integral[front]);
  Serial.print("del_s: "); 
  Serial.println(speed_delta[front]);

  Serial.print("servo_l: "); Serial.println(servo_speeds[left]);
  Serial.print("servo_r: "); Serial.println(servo_speeds[right]);
}

long sensor_read_distance(int echo_pin, int trigger_pin)
{
  // trigger sensors
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigger_pin, LOW);

  long echo_duration = pulseIn(echo_pin, HIGH);

  return (echo_duration / 29 / 2); // centimeters
}
