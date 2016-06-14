#include <PID_v1.h>
#include <NewPing.h>
#include <Servo.h>

Servo aleo,eleo,thro;

double Setpoint,Input1,Output1,Input2,Output2,Input3,Output3,Input4,Output4,Input5,Output5;

// NewPing( trigger pin, echo pin, max distance ) 
NewPing s1(A0,A1,200),s2(A2,A3,200),s3(A4,A5,200),s4(2,3,200),s5(4,5,200);

#define Kp 2		// proportional — reduces large part of overall error
#define Ki 5		// integral — reduces final error in system, avoids summing up a small error over multiple times
#define Kd 1		// derivative — counteracts KP and KI when the change happens quickly, no affect on final error

/*
PID calculates an error value, the difference between a measured input
and a desired set point. The controller attempts to minimize the error by 
adjusting the output. 

Values are passed be reference(&) so the PID library can change values within it’s own code 
while changing the values in this code as well. 

PID ( input, output, setpoint, kp, ki, kd, direction )

*/
PID leftPID(&Input1, &Output1, &Setpoint,Kp,Ki,Kd, DIRECT);
PID rightPID(&Input2, &Output2, &Setpoint,Kp,Ki,Kd, DIRECT);
PID frontPID(&Input3, &Output3, &Setpoint,Kp,Ki,Kd, DIRECT);
PID backPID(&Input4, &Output4, &Setpoint,Kp,Ki,Kd, DIRECT);
PID altPID(&Input5, &Output5, &Setpoint,Kp,Ki,Kd, DIRECT);

int left, right, front, back;
int ale, ele, thr; 
int op1, op2, op3, op4;
int aux1, aux2, x, y, z;

// setup happens usually only once, at startup
// initializes values
void setup()
{

// each sensor sends out a ping from each sensor, s# refers to a specific sensor
  Input1=s1.ping_cm();   
  Input2=s2.ping_cm();
  Input3=s3.ping_cm();
  Input4=s4.ping_cm();
  Input5=s5.ping_cm();

  Setpoint=50;

// pid controller changes the output value within set range 
  leftPID.SetOutputLimits(0, 300);
  rightPID.SetOutputLimits(0, 300);
  frontPID.SetOutputLimits(0, 300);
  backPID.SetOutputLimits(0, 300);

// altitude sensor
  altPID.SetOutputLimits(1000, 2000);

// turns the PID controller on for each direction 
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  frontPID.SetMode(AUTOMATIC);
  backPID.SetMode(AUTOMATIC);
  altPID.SetMode(AUTOMATIC);

// attach the specified servo to a pin 
  aleo.attach(11);
  eleo.attach(12);
  thro.attach(13);

// allows for printing to the console
  Serial.begin(115200);
  Serial.println("initializing......");
}

// this code is run continuously 
  void loop()
  {

// pin waits for the value to return HIGH, it then counts the amount of time 
// it is held at a HIGH state. Returns the length of the pulse, times out at 20000 us
    x = pulseIn(6,HIGH,20000);
    y = pulseIn(7,HIGH,20000);
    z = pulseIn(8,HIGH,20000);
    aux1=pulseIn(9,HIGH,20000);
    aux2=pulseIn(10,HIGH,20000);

// drone in takeoff (i think?) 
    if((aux1<=1500)&&(aux2<=1500))
    {
      aleo.writeMicroseconds(x);
      eleo.writeMicroseconds(y);
      thro.writeMicroseconds(z);
    }

// 
    if(aux1>1500)
    {
     
	// send a ping out of every sensor and print the values
	 Input1=s1.ping_cm();
        Serial.print("Sonar1 =");
        Serial.println(Input1);

        Input2=s2.ping_cm();
        Serial.print("Sonar2 =");
        Serial.println(Input2);

        Input3=s3.ping_cm();
        Serial.print("Sonar3 =");
        Serial.println(Input3);

        Input4=s4.ping_cm();
        Serial.print("Sonar4 =");
        Serial.println(Input4);
	
	
	// applies the PID algorithm
	// the compute function doesn’t need any input values bc the library saves all the values for each sensor
	// so when they use the algorithm, the use it on the already saved values. 
        leftPID.Compute();
        rightPID.Compute();
        frontPID.Compute();
        backPID.Compute();

	// calculate the new aileron value based on PID outputs 
        ale=1500+Output1-Output2;

	// calculate the new elevator value based on PID outputs
        ele=1500+Output3-Output4;

	// change servos values based on output
        aleo.writeMicroseconds(ale);
        eleo.writeMicroseconds(ele);

	// print out values
        Serial.print("Aileron =");
        Serial.println(ale);
        Serial.print("Elevator =");
        Serial.println(ele);
    }
        if(aux2>1500)
        {

	 // send out ping for bottom sensor
          Input5=s5.ping_cm();
          Serial.print("Sonar5 =");
          Serial.println(Input5);

	// altitude pid algorithm
          altPID.Compute();
	
	// update throttle 
          thr=Output5;
          thro.writeMicroseconds(thr);
          Serial.print("Throttle =");
        Serial.println(thr);
        }
          
        
      
  }
