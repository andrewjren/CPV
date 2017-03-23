#ifndef ZABERX_H
#define ZABERX_H

#include "Arduino.h"
#define mmResolution 0.000047625
#define umResolution 0.047625
#define resolutionDeg  0.000234375
#define resolutionRad  0.000004091

#define quad 4294967296
#define cubed 16777216
#define squared 65536


	int readAnalog(int analogPin, int iterations);

	long mm(float mmValue);

	long um(float umValue);

	long stepsD(float degr);

	long stepsR(float radis);

	double interp(double input);

	double interp1(double input);

	double interp2(double input);


#endif