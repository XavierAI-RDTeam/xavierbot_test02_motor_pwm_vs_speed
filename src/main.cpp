#include <Arduino.h>
#include <definitions.h>

// encoder pulse count
volatile int frontLeftPulseCount  = 0;
volatile int frontRightPulseCount = 0;
// volatile int rearLeftPulseCount  = 0;
// volatile int rearRightPulseCount = 0;

// protected encoder pulse count
volatile int frontLeftProtectedPulseCount  = 0;
volatile int frontRightProtectedPulseCount = 0;
// volatile int rearLeftProtectedPulseCount  = 0;
// volatile int rearRightProtectedPulseCount = 0;

// distance travelled by each wheel at each pid iteration
float front_left_distance					= 0;
float front_right_distance					= 0;
// float rear_left_distance					= 0;
// float rear_right_distance					= 0;

// target values related to PID received from kinematic model
float front_left_target						= 0;
float front_right_target					= 0;
// float rear_left_target						= 0;
// float rear_right_target						= 0;

// velocity feedback values related to PID loop
float front_left_feedback					= 0;
float front_right_feedback					= 0;
// float rear_left_feedback					= 0;
// float rear_right_feedback					= 0;


// duration between iterations
float timeDuration 							= 300;
long startTime		=	0;

/*******************************************************************************
** Interrupts for Encoder
*******************************************************************************/

void isrFrontLeftA() 
{
	if(digitalRead(FRONT_LEFT_ENCODER_A) != digitalRead(FRONT_LEFT_ENCODER_B))  frontLeftPulseCount ++; 
	else frontLeftPulseCount --;
}

void isrFrontLeftB() 
{
	if (digitalRead(FRONT_LEFT_ENCODER_A) == digitalRead(FRONT_LEFT_ENCODER_B)) frontLeftPulseCount ++; 
	else frontLeftPulseCount --;
}

void isrFrontRightA() 
{
	if(digitalRead(FRONT_RIGHT_ENCODER_A) != digitalRead(FRONT_RIGHT_ENCODER_B))  frontRightPulseCount ++; 
	else frontRightPulseCount --;
}

void isrFrontRightB() 
{
	if (digitalRead(FRONT_RIGHT_ENCODER_A) == digitalRead(FRONT_RIGHT_ENCODER_B)) frontRightPulseCount ++;
  else frontRightPulseCount --;
}

// void isrRearLeftA() 
// {
// 	if(digitalRead(REAR_LEFT_ENCODER_A) != digitalRead(REAR_LEFT_ENCODER_B))  rearLeftPulseCount ++; 
// 	else rearLeftPulseCount --;
// }

// void isrRearLeftB() 
// {
// 	if (digitalRead(REAR_LEFT_ENCODER_A) == digitalRead(REAR_LEFT_ENCODER_B)) rearLeftPulseCount ++; 
// 	else rearLeftPulseCount --;
// }

// void isrRearRightA() 
// {
// 	if(digitalRead(REAR_RIGHT_ENCODER_A) != digitalRead(REAR_RIGHT_ENCODER_B))  rearRightPulseCount ++; 
// 	else rearRightPulseCount --;
// }

// void isrRearRightB() 
// {
// 	if (digitalRead(REAR_RIGHT_ENCODER_A) == digitalRead(REAR_RIGHT_ENCODER_B)) rearRightPulseCount ++;
//   else rearRightPulseCount --;
// }

/*******************************************************************************
** Interrupt safe data transfer
*******************************************************************************/

void transferDataFromInterrupts()
{
	noInterrupts();

	frontLeftProtectedPulseCount  = frontLeftPulseCount;
	frontRightProtectedPulseCount = frontRightPulseCount;
  	// rearLeftProtectedPulseCount   = rearLeftPulseCount;
	// rearRightProtectedPulseCount  = rearRightPulseCount;

	frontLeftPulseCount  = 0;
	frontRightPulseCount = 0;
  	// rearLeftPulseCount   = 0;
	// rearRightPulseCount  = 0;

	interrupts();
}

void setup() {
	Serial.begin(115200);

	startTime = micros();

	pinMode(FRONT_LEFT_ENCODER_A, 	INPUT);
	pinMode(FRONT_LEFT_ENCODER_B, 	INPUT);
	pinMode(FRONT_RIGHT_ENCODER_A, 	INPUT);
	pinMode(FRONT_RIGHT_ENCODER_B, 	INPUT);
  	// pinMode(REAR_LEFT_ENCODER_A, 	INPUT);
	// pinMode(REAR_LEFT_ENCODER_B, 	INPUT);
	// pinMode(REAR_RIGHT_ENCODER_A, 	INPUT);
	// pinMode(REAR_RIGHT_ENCODER_B, 	INPUT);

	attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_ENCODER_A),  isrFrontLeftA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_ENCODER_B),  isrFrontLeftB, CHANGE);
	attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_ENCODER_A), isrFrontRightA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_ENCODER_B), isrFrontRightB, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(REAR_LEFT_ENCODER_A), isrRearLeftA, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(REAR_LEFT_ENCODER_B), isrRearLeftB, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_ENCODER_A), isrRearRightA, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_ENCODER_B), isrRearRightB, CHANGE);


	for (int i = 0; i < 255; i++)
	{

		transferDataFromInterrupts();

		timeDuration = float(micros() - startTime);

		front_left_distance   = REAL_WORLD_FACTOR * 2 * PI * WHEEL_RADIUS * ((((float)frontLeftProtectedPulseCount)	/ ENCODER_PULSES_PER_ROTATION) / ENCODER_REDUCTION);
		front_right_distance  = REAL_WORLD_FACTOR * 2 * PI * WHEEL_RADIUS * ((((float)frontRightProtectedPulseCount)/ ENCODER_PULSES_PER_ROTATION) / ENCODER_REDUCTION);
		// rear_left_distance	  = REAL_WORLD_FACTOR * 2 * PI * WHEEL_RADIUS * ((((float)rearLeftProtectedPulseCount)	/ ENCODER_PULSES_PER_ROTATION) / ENCODER_REDUCTION);
		// rear_right_distance   = REAL_WORLD_FACTOR * 2 * PI * WHEEL_RADIUS * ((((float)rearRightProtectedPulseCount)	/ ENCODER_PULSES_PER_ROTATION) / ENCODER_REDUCTION);

		front_left_feedback   = front_left_distance /0.3;
		front_right_feedback  = front_right_distance/0.3;
		// rear_left_feedback    = rear_left_distance  /timeDuration;
		// rear_right_feedback   = rear_right_distance /timeDuration;

		analogWrite(FRONT_LEFT_PWM_FORWARD, i);
		analogWrite(FRONT_LEFT_PWM_REVERSE, 0);

		analogWrite(FRONT_RIGHT_PWM_FORWARD, 0);
		analogWrite(FRONT_RIGHT_PWM_REVERSE, i);

		// analogWrite(REAR_LEFT_PWM_FORWARD,  i);
		// analogWrite(REAR_LEFT_PWM_REVERSE,  0);

		// analogWrite(REAR_RIGHT_PWM_FORWARD,  0);
		// analogWrite(REAR_RIGHT_PWM_REVERSE,  i);

		Serial.print(i);
		Serial.print(",");
		Serial.print(front_left_feedback);
		Serial.print(",");
		Serial.println(front_right_feedback);
		// Serial.print(",");
		// Serial.print(rearLeftProtectedPulseCount);
		// Serial.print(",");
		// Serial.println(rearRightProtectedPulseCount);

		delay(300);
	}

	for (int i = 255; i >= 0; i--)
	{
		transferDataFromInterrupts();

		timeDuration = float(micros() - startTime);

		front_left_distance   = REAL_WORLD_FACTOR * 2 * PI * WHEEL_RADIUS * ((((float)frontLeftProtectedPulseCount)	/ ENCODER_PULSES_PER_ROTATION) / ENCODER_REDUCTION);
		front_right_distance  = REAL_WORLD_FACTOR * 2 * PI * WHEEL_RADIUS * ((((float)frontRightProtectedPulseCount)/ ENCODER_PULSES_PER_ROTATION) / ENCODER_REDUCTION);
		// rear_left_distance	  = REAL_WORLD_FACTOR * 2 * PI * WHEEL_RADIUS * ((((float)rearLeftProtectedPulseCount)	/ ENCODER_PULSES_PER_ROTATION) / ENCODER_REDUCTION);
		// rear_right_distance   = REAL_WORLD_FACTOR * 2 * PI * WHEEL_RADIUS * ((((float)rearRightProtectedPulseCount)	/ ENCODER_PULSES_PER_ROTATION) / ENCODER_REDUCTION);

		front_left_feedback   = front_left_distance /0.3;
		front_right_feedback  = front_right_distance/0.3;
		// rear_left_feedback    = rear_left_distance  /timeDuration;
		// rear_right_feedback   = rear_right_distance /timeDuration;

		analogWrite(FRONT_LEFT_PWM_FORWARD, i);
		analogWrite(FRONT_LEFT_PWM_REVERSE, 0);

		analogWrite(FRONT_RIGHT_PWM_FORWARD, 0);
		analogWrite(FRONT_RIGHT_PWM_REVERSE, i);

		// analogWrite(REAR_LEFT_PWM_FORWARD,  i);
		// analogWrite(REAR_LEFT_PWM_REVERSE,  0);

		// analogWrite(REAR_RIGHT_PWM_FORWARD,  0);
		// analogWrite(REAR_RIGHT_PWM_REVERSE,  i);

		Serial.print(i);
		Serial.print(",");
		Serial.print(front_left_feedback);
		Serial.print(",");
		Serial.println(front_right_feedback);
		// Serial.print(",");
		// Serial.print(rearLeftProtectedPulseCount);
		// Serial.print(",");
		// Serial.println(rearRightProtectedPulseCount);

		delay(300);
	}
	
}

void loop() {
}