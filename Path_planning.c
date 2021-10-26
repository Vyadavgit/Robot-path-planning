/*
 \file		Pathplanning.c
 \author	${user}
 \date		${date}
 \brief		Simple Hello World! for the Ev3
*/

#include <ev3.h>

int main(void)
{
	// INFO This code template works only with recent versions of the API. If TermPrintln is not found,
	//      please update the API or use the "Hello World EV3 Project Old API" as template.

	InitEV3();
	//TODO Place here your variables

	//TODO Place here your code
	TermPrintf("Press ENTER to exit");
	ResetRotationCount(OUT_B);
	ResetRotationCount(OUT_C);
	while (MotorRotationCount(OUT_B)<500 && MotorRotationCount(OUT_C)<500) {
	OnFwdSync(OUT_BC, 5);
	TermPrintf("B: %d, C: %d\n", MotorRotationCount(OUT_B), MotorRotationCount(OUT_C));
	}
	Off(OUT_BC);
	Wait(500);
	RotateMotor(OUT_B, 5, 120);

	FreeEV3();
	return 0;
}
