/*
 Name:		Multimaterial_Claw_GUI.ino
 Created:	9/21/2018
 Author:	Jehan
*/
#include <stdlib.h>
#include <stdio.h>

int solenoidPin = 4;    //This is the output pin on the Arduino we are using
int flexSensPin = A0;
int extSensPin = A1;

int pressure = 0;
double refResist = 0; // TODO: choose a reference resistor for our required force range
double sensInputVolt = -1; // TODO: figure out whether it would be annoying to output -1V
double desPressure = 0;

void setup() {
  Serial.begin(9600);
  pinMode(solenoidPin, OUTPUT);     //Sets the pin as an output
  pinMode(flexSensPin, INPUT);
  pinMode(extSensPin, INPUT);
}

void loop() { 
  //Note that analogRead takes ~100 microseconds and maps 0 to 5V between 0 to 1023
  //TODO: READ FROM UI WHAT THE DESIRED PRESSURE IS IN NEXT LINE

  if (Serial.available() > 0) {
	// read the incoming bytes:
	desPressure = Serial.parseFloat();

	// say what you got:
	Serial.print("I received: ");
	Serial.println(desPressure);
  }
  //Currently bang bang control (which may cause constant switching)
  //TODO: implement PID control
  pressure = volt2Pressure(refResist,
						   sensInputVolt,
						   analogRead2Voltage(analogRead(presSensPin))); 
  if (pressure > desPressure) {
	digitalWrite(solenoidPin, LOW);
  } else {
	digitalWrite(solenoidPin, HIGH);
  }

  delay(5);
}

// TODO: I haven't implemented this
double readPresFromUI() {
	// reads some input from the UI
	return desPressure;
}

// TODO: this function is also not finished
double volt2Pressure(double resistor, double volt_supp, double volt_sens) {
	// Volt to pressure in Pa
	// Used to find the actual pressure using the Tekscan #A401 with the recommended circuit in the datasheet
	// Datasheet: https://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Pressure/A401-force-sensor.pdf
	// WARNING: if volt supp is not negative, then this is going to supply a negative voltage
	double area = pi * (.0254 / 2)^2;	// Area in m^2
	double res_sens = (-volt_supp / volt_sens) * resistor;
	// TODO: the force vs voltage must be characterized, meaning we need to get a load cell from somewhere to test this
	double pressure = force / area;
}

double analogRead2Voltage(int reading) {
	//	the reading is some value from 0 to 1024
	double voltage = ((double) reading) / 1024 * 5;
	return voltage;
}