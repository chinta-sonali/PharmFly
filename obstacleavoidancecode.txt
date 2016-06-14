// ---------------------------------------------------------------------------
// This was our code for controlling obstacle avoidance of our quadcopter. We experimentally determined our proportional constants, and would like to include the PID library of Arduino in future experimentation.
// ---------------------------------------------------------------------------
#include <NewPing.h>
#include <Servo.h>

Servo Roll;
Servo Pitch;
Servo Throttle;
Servo Yaw;
Servo Aux;

#define SONAR_NUM     5 // Number of sensors.
#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
// #define TargetUnder 100 // Altitude to Maintain
#define TargetFront 45
#define TargetLeft 45
#define TargetRight 45
#define TargetBack 45
#define Kr 5
#define Kp 5
#define Kt 5
#define Ky 5
#define Ka 5



unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];
int Rcontrol, Pcontrol, Tcontrol, Ycontrol, Acontrol, Rfreq = 1500, Pfreq = 1500, Tfreq = 1000, Yfreq = 1500, Afreq = 1000;
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(12, 13, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(10, 11, MAX_DISTANCE),
  NewPing(8, 9, MAX_DISTANCE),
  NewPing(10,11, MAX_DISTANCE)
  NewPing(4, 5, MAX_DISTANCE)
};

void setup() {
  Serial.begin(115200);
  Roll.attach(A1);
  Pitch.attach(A2);
  Throttle.attach(A3);
  Yaw.attach(A4);
  Aux.attach(A5);
  Throttle.writeMicroseconds(1000); // This sequence begins the arming procedure of the quad
  Pitch.writeMicroseconds(1500);
  Yaw.writeMicroseconds(1500);
  Roll.writeMicroseconds(1500);
  delay(5000);
  Yaw.writeMicroseconds(2000);
  delay(5500);
  Yaw.writeMicroseconds(1500); // The quad should be armed and motors running here
  Throttle.writeMicroseconds(1100); // This throttle command keeps quad from auto-shutting off
  delay(3000);
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  // Other code that *DOESN'T* analyze ping results can go here.
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.

  if (abs(cm[4] < TargetFront)) {
    Rcontrol = 1500 + (TargetFront - cm[4]) * Kr;
    Serial.println(Rcontrol);
    Serial.println(cm[4]);
    Pitch.writeMicroseconds(Rcontrol);
  }

  if (abs(cm[3] < TargetBack)) {
    Rcontrol = 1500 - (TargetBack - cm[3]) * Kr;
    Serial.println(Rcontrol);
    Serial.println(cm[3]);
    Pitch.writeMicroseconds(Rcontrol);
  }

  if (abs(cm[2] < TargetLeft)) {
    Rcontrol = 1500 + (TargetLeft - cm[2]) * Kr;
    Serial.println(Pcontrol);
    Serial.println(cm[2]);
    Pitch.writeMicroseconds(Pcontrol);
  }

  if (abs(cm[1] < TargetRight)) {
    Rcontrol = 1500 + (TargetRight - cm[1]) * Kr;
    Serial.println(Pcontrol);
    Serial.println(cm[1]);
    Pitch.writeMicroseconds(Pcontrol);
  }
}

