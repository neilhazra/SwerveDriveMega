/*
  RoboLib.h - Library for Arduino to control FRC Robots.
  Created by Neil Hazra, July 4, 2016.
*/

#ifndef RoboLib_h
#define RoboLib_h

#include "Arduino.h"

class RoboLib
{
  public:
   RoboLib(int dataSize, int maxSize);
   void begin(int baudRate);
   float* getData();
   void saveData();
   void printInfo();
   bool fatalError();
 private:
	float *processedData;
	char *incommingData;
	char *_incommingData;
	int _dataSizeConst;
	int _maxSize;
	float mapf(float x, float in_min, float in_max, float out_min,float out_max);
	unsigned long prevTime;
};
RoboLib::RoboLib(int dataSizeConst, int maxSize)	{
	_dataSizeConst = dataSizeConst;
	_maxSize = maxSize;
	incommingData = new char[maxSize];
	_incommingData = new char[maxSize];
	processedData = new float[dataSizeConst];
}
void RoboLib::begin(int baudRate)	{
	Serial.begin(baudRate);
	prevTime = millis();
}

float* RoboLib::getData()	{
   return processedData;
}
void RoboLib::saveData()  {
	Serial.readStringUntil('\n').toCharArray(incommingData,_maxSize);
    for(int i = 0; i<_maxSize; i++)  {
      _incommingData[i] = incommingData[i];
    }
   char *p = _incommingData;
   char *str;
   char *data[_dataSizeConst];
   int i = 0;
   while ((str = strtok_r(p, ";", &p)) != NULL)   {// delimiter is the semicolon
     data[i] = str;
     i++;
   }

   for(int _i = 0; _i<i; _i++)  {
     processedData[_i] = atof(data[_i]);
   }
   prevTime = millis();
}

bool RoboLib::fatalError()	{
	if(millis() - prevTime> 300 && !Serial.available())  {
		return true;
	}	else {
		return false;
	}
}

void RoboLib::printInfo() {
  int x = _dataSizeConst;
  for(int i = 0; i < x; i++)  {
    Serial.print(processedData[i]);
    Serial.print(" ");
  }
  Serial.println();
}

#endif
