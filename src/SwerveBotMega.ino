#include <Encoder.h>
#include <RoboLib.h>
#include <Servo.h>
#include <PID.h>

double inputFront;
double inputBack;
double outputFront;
double outputBack;

const int dataSizeConst = 3;
float* processedData;

RoboLib myBot(dataSizeConst,dataSizeConst*5);

PID frontPID(&inputFront, &outputFront, 0, 100,0.01,0.001,0);
PID backPID(&inputBack, &outputBack, 0,100,0.01,0.001,0);

Servo frontSteeringMotor;
Servo backSteeringMotor;
Servo frontLeftMotor;
Servo frontRightMotor;
Servo backLeftMotor;
Servo backRightMotor;

Encoder frontEncoder(2,3);
Encoder backEncoder(18,19);

void serialEvent()	{
	myBot.saveData();
}
void setup()	{
	myBot.begin(9600);
	frontSteeringMotor.attach(A0);
	backSteeringMotor.attach(A1);
	frontRightMotor.attach(A2);
	frontLeftMotor.attach(A3);
	backRightMotor.attach(A4);
	backLeftMotor.attach(A5);
}
void loop() {
	processedData = myBot.getData();

	inputFront = frontEncoder.read();
	inputBack = backEncoder.read();

	if(processedData[2]==1) {
		frontPID.setSetpoint(90*processedData[0]);
		backPID.setSetpoint(-90*processedData[0]);
	} if(processedData[2]==0) {
		frontPID.setSetpoint(90*processedData[0]);
		backPID.setSetpoint(90*processedData[0]);
	}

	int power = (int)mapf(processedData[1],1,-1,670,2330);

	frontSteeringMotor.writeMicroseconds(outputFront);
	backSteeringMotor.writeMicroseconds(outputFront);
	frontLeftMotor.writeMicroseconds(power);
	frontRightMotor.writeMicroseconds(power);
	backLeftMotor.writeMicroseconds(power);
	backRightMotor.writeMicroseconds(power);

	myBot.printInfo();

	while(myBot.fatalError())	{
		frontSteeringMotor.writeMicroseconds(1500);
		frontLeftMotor.writeMicroseconds(1500);
		frontRightMotor.writeMicroseconds(1500);
		backSteeringMotor.writeMicroseconds(1500);
		backLeftMotor.writeMicroseconds(1500);
		backRightMotor.writeMicroseconds(1500);
	}
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
