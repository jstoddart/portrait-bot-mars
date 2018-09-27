//Firmware for testing motor functions

//DEPENDENCIES
#include <AccelStepper.h>

//AXIS Setup

//un-comment this line only for X-axis
//AccelStepper stepper( AccelStepper::DRIVER, 2, 3);

//uncomment this line only for Y-axis
AccelStepper stepper( AccelStepper::DRIVER, 5, 6);

void setup()
{
	stepper.setMaxSpeed(250);
	stepper.setAcceleration(50);
	stepper.moveTo(1000);
}

void loop()
{
	if( stepper.distanceToGo() == 0 )
	{
		stepper.moveTo( -stepper.currentPosition() );
	}

	stepper.run();
	
}
