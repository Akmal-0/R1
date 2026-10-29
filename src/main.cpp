#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <config.h>
#include <pid.h>
#include <kinematic.h>
#include <odometry.h>
#include <joystick.h>
#include <scurve.h>
#include <semiauto.h>

JoystickManager joyManager;
ButtonState &button = joyManager.button;
AxisState &joystick = joyManager.axis;

SCurveFilter scurve_x(SCURVE_MAX_ACCEL_LINEAR, SCURVE_MAX_JERK_LINEAR);
SCurveFilter scurve_y(SCURVE_MAX_ACCEL_LINEAR, SCURVE_MAX_JERK_LINEAR);
SCurveFilter scurve_wz(SCURVE_MAX_ACCEL_ANGULAR, SCURVE_MAX_JERK_ANGULAR);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

SemiAutoController semiAuto;

PID wheel1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel3(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel4(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematic kinematic(
	Kinematic::ROBOT_1,
	MOTOR_MAX_RPS,
	MAX_RPS_RATIO,
	MOTOR_OPERATING_VOLTAGE,
	MOTOR_POWER_MAX_VOLTAGE,
	WHEEL_DIAMETER,
	ROBOT_DIAMETER);

Odometry odometry;

unsigned long prev_odom_update = 0;
unsigned long prevT = 0;
volatile long pos[4];

joy joySmoothed;
joy joyCmd;

template <int j>
void readEncoder();
void setMotor(int cwPin, int ccwPin, float pwmVal);
void moveBase();
void Emergencystop();

template <int j>
void readEncoder()
{
	int b = digitalRead(encb[j]);
	if (b > 0)
		pos[j]++;
	else
		pos[j]--;
}

void setup()
{
	Serial.begin(115200);

	for (int i = 0; i < 4; i++)
	{
		pinMode(cw[i], OUTPUT);
		pinMode(ccw[i], OUTPUT);

		analogWriteFrequency(cw[i], PWM_FREQUENCY);
		analogWriteFrequency(ccw[i], PWM_FREQUENCY);
		analogWriteResolution(PWM_BITS);
		analogWrite(cw[i], 0);
		analogWrite(ccw[i], 0);
		pinMode(enca[i], INPUT);
		pinMode(encb[i], INPUT);
	}

	joyManager.begin();
	bno.begin();
	bno.setExtCrystalUse(true);

	semiAuto.begin();

	wheel1.ppr_total(COUNTS_PER_REV1);
	wheel2.ppr_total(COUNTS_PER_REV2);
	wheel3.ppr_total(COUNTS_PER_REV3);
	wheel4.ppr_total(COUNTS_PER_REV4);

	attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
}

void loop()
{
	joyManager.task();

	if (!joyManager.isConnected())
	{
		Emergencystop();
		joyManager.handleDisconnect();
		delay(100);
		return;
	}

	if (joyManager.update())
	{
		if (!joyManager.isReady())
		{
			uint64_t axesSeen = joyManager.getAxesSeen();
			if (!joyManager.qualifyReady())
			{
				Emergencystop();
				joyManager.restoreAxesSeen(axesSeen);
				return;
			}

			prevT = micros();
			prev_odom_update = millis();
		}

		static bool lastBtnY = false;
		static bool lastBtnA = false;
		static bool lastBtnB = false;

		if (button.Y && !lastBtnY)
		{
			sensors_event_t event;
			bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
			semiAuto.triggerForward(-event.orientation.x);
		}
		else if (button.A && !lastBtnA)
		{
			sensors_event_t event;
			bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
			semiAuto.triggerBackward(-event.orientation.x);
		}
		else if (button.B && !lastBtnB)
		{
			semiAuto.cancel();
		}

		lastBtnY = button.Y;
		lastBtnA = button.A;
		lastBtnB = button.B;

		if (!joyManager.isTimeout())
		{
			moveBase();
		}
	}

	if (joyManager.isTimeout())
	{
		Emergencystop();
	}
}

void moveBase()
{
	if (!joyManager.isReady())
	{
		Emergencystop();
		return;
	}

	sensors_event_t event;
	bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

	unsigned long currT = micros();
	float deltaT = ((float)(currT - prevT)) / 1.0e6;

	if (semiAuto.isBusy())
	{
		if (joyManager.isManualInput())
		{
			semiAuto.cancel();
		}
		else
		{
			float autoVx = 0.0f;
			float autoVy = 0.0f;
			float autoWz = 0.0f;
			semiAuto.update(deltaT, -event.orientation.x, autoVx, autoVy, autoWz);

			joySmoothed.axis1_x = autoVx;
			joySmoothed.axis1_y = autoVy;
			joySmoothed.axis0_y = autoWz;
		}
	}

	if (!semiAuto.isBusy())
	{
		float top_speed;
		float rot_speed;
		if (joyManager.isSpeedFast())
		{
			top_speed = SPEED_FAST_TOP;
			rot_speed = SPEED_FAST_ROT;
		}
		else
		{
			top_speed = SPEED_SLOW_TOP;
			rot_speed = SPEED_SLOW_ROT;
		}

		joyCmd.axis1_x = map(-joystick.axis1_x, 0, 128, 0, top_speed);
		joyCmd.axis1_y = map(joystick.axis1_y, 0, 128, 0, top_speed);
		joyCmd.axis0_y = map(joystick.axis0_y, 0, 128, 0, rot_speed);

		joySmoothed.axis1_x = scurve_x.update(joyCmd.axis1_x, deltaT);
		joySmoothed.axis1_y = scurve_y.update(joyCmd.axis1_y, deltaT);
		joySmoothed.axis0_y = scurve_wz.update(joyCmd.axis0_y, deltaT);
	}

	Kinematic::rps req_rps;
	req_rps = kinematic.getRPS(
		joySmoothed.axis1_x,
		-joySmoothed.axis1_y,
		joySmoothed.axis0_y,
		-event.orientation.x);

	float controlled_motor1 = wheel1.control_speed(req_rps.motor1, pos[0], deltaT);
	float controlled_motor2 = wheel2.control_speed(req_rps.motor2, pos[1], deltaT);
	float controlled_motor3 = wheel3.control_speed(req_rps.motor3, pos[2], deltaT);
	float controlled_motor4 = wheel4.control_speed(req_rps.motor4, pos[3], deltaT);

	float current_rps1 = wheel1.get_filt_vel();
	float current_rps2 = wheel2.get_filt_vel();
	float current_rps3 = wheel3.get_filt_vel();
	float current_rps4 = wheel4.get_filt_vel();

	if (fabs(req_rps.motor1) < 0.02)
		controlled_motor1 = 0.0;
	if (fabs(req_rps.motor2) < 0.02)
		controlled_motor2 = 0.0;
	if (fabs(req_rps.motor3) < 0.02)
		controlled_motor3 = 0.0;
	if (fabs(req_rps.motor4) < 0.02)
		controlled_motor4 = 0.0;

	setMotor(cw[0], ccw[0], controlled_motor1);
	setMotor(cw[1], ccw[1], controlled_motor2);
	setMotor(cw[2], ccw[2], controlled_motor3);
	setMotor(cw[3], ccw[3], controlled_motor4);

	Kinematic::velocities vel = kinematic.getVelocities(
		current_rps1,
		current_rps2,
		current_rps3,
		current_rps4);

	unsigned long now = millis();
	float vel_dt = (now - prev_odom_update) / 1000.0;

	prev_odom_update = now;
	odometry.update(vel_dt, vel.linear_x, vel.linear_y, vel.angular_z);
	prevT = currT;
}

void setMotor(int cwPin, int ccwPin, float pwmVal)
{
	if (pwmVal > 0)
	{
		analogWrite(cwPin, fabs(pwmVal));
		analogWrite(ccwPin, 0);
	}
	else if (pwmVal < 0)
	{
		analogWrite(cwPin, 0);
		analogWrite(ccwPin, fabs(pwmVal));
	}
	else
	{
		analogWrite(cwPin, 0);
		analogWrite(ccwPin, 0);
	}
}

void Emergencystop()
{
	joyManager.reset();
	semiAuto.cancel();

	joyCmd = {};
	joySmoothed = {};

	scurve_x.reset();
	scurve_y.reset();
	scurve_wz.reset();

	for (int i = 0; i < 4; i++)
	{
		analogWrite(cw[i], 0);
		analogWrite(ccw[i], 0);
	}
}
