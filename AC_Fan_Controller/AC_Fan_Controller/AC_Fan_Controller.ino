/*
Name:		AC_controller_V01.ino
Created:	5/2/2016 1:16:58 PM
Author:	Tom


#Known issues:
-Use optibootloader else watchdog timer, does not work! (change boards.txt accordingly) https://andreasrohner.at/posts/Electronics/How-to-make-the-Watchdog-Timer-work-on-an-Arduino-Pro-Mini-by-replacing-the-bootloader/


#Info
Default values nu sensors attached: Motor Temp C: -127.00	Air Temp C: -47.54	Humidity %: -6.49

#Sources
SI7021: http://www.dfrobot.com/wiki/index.php/SI7021_Temperature_and_humidity_sensor_SKU:TOY0054
watchdog: http://folk.uio.no/jeanra/Microelectronics/ArduinoWatchdog.html



#TO DO:
-If no motor temp sensor is detected, use safe minimum delay value
-Add program state where motor temp is to high and speed is austed to cool extra, else shutdown
-Temp used for determining speed. if threshold 26 is reached speed will increase
-Disable interrupts. in full on and off state. instead of off flag
*/



///Libraries
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <util/delay.h>  //for _delay_ms() that can be used inside the ISR

///Pin definements
#define CAN_INT 2
#define ZERO_CROSS 3
#define GREEN_LED 4
#define RED_LED 5
#define TRIAC_DRIVER 6
#define MOTOR_TEMP 7
#define BLUE_LED 8
#define RELAY 9
#define CAN_CS 10
#define MOSI 11
#define MISO 12
#define SCK 13
#define MANUAL_POT A0
#define LOAD_CURRENT A1
#define G_LED_CAN A2
#define Y_LED_CAN A3
#define SDA A4 
#define SCL A5 

///Other definements
#define BAUD 9600 //Serial UART baud rate
#define ADDRI2C 0x40 //Temp/RH sensor SI7021 adress

///Library objects
OneWire oneWire(MOTOR_TEMP); //dallas temp sensor. create onewire bus on this pin
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature. 						
DeviceAddress insideThermometer; // arrays to hold device address

///Globals: main variables
int triacDelay = 0;

///Globals: support variables
boolean offFlag = true;



/// the setup function runs once when you press reset or power the board
void setup() {
	//wdt_enable(WDTO_4S); //enable watchdog timer 4sec
	configure_wdt();

	///initialize pinmodes
	//pinMode(CAN_INT, INPUT); //Not necessary, beacause all pins are defined as input as default
	//pinMode(ZERO_CROSS, INPUT); //Not necessary, beacause all pins are defined as input as default
	pinMode(GREEN_LED, OUTPUT);
	pinMode(RED_LED, OUTPUT);
	pinMode(TRIAC_DRIVER, OUTPUT);
	//pinMode(MOTOR_TEMP, INPUT); //Not necessary, library for sensor initializes this pin
	pinMode(BLUE_LED, OUTPUT);
	pinMode(RELAY, OUTPUT);
	digitalWrite(RELAY, HIGH); //relay is active low. when program starts relay must be high to be safe
	pinMode(CAN_CS, OUTPUT);
	//pinMode(MOSI, OUTPUT); //Not necessary
	//pinMode(MISO, OUTPUT); //Not necessary
	//pinMode(SCK, OUTPUT);  //Not necessary
	//pinMode(MANUAL_POT, INPUT); //Not necessary, beacause all pins are defined as input as default
	//pinMode(LOAD_CURRENT, INPUT); //Not necessary, beacause all pins are defined as input as default
	pinMode(G_LED_CAN, OUTPUT);
	pinMode(Y_LED_CAN, OUTPUT);

	///Setup UART serial
	Serial.begin(BAUD);

	///Initialize temp/RH sensor SI7021
	Wire.begin();
	delay(100);
	Wire.beginTransmission(ADDRI2C);
	Wire.endTransmission();

	///Initializes motor temp sensor
	sensors.begin(); //Search for one wire sensors on the bus
	Serial.print("Found ");
	Serial.print(sensors.getDeviceCount(), DEC);
	Serial.println(" oneWire devices.");

	// Method 1: Save found onewire adresses
	// Search for devices on the bus and assign based on an index. Ideally,
	// you would do this to initially discover addresses on the bus and then 
	// use those addresses and manually assign them (see above) once you know 
	// the devices on your bus (and assuming they don't change).
	if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");

	sensors.setResolution(insideThermometer, 9);// set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)


	attachInterrupt(digitalPinToInterrupt(ZERO_CROSS), zerocross, RISING);

	///Setup completed, blink all RGB LED for a sec
	digitalWrite(RED_LED, HIGH);
	digitalWrite(GREEN_LED, HIGH);
	digitalWrite(BLUE_LED, HIGH);
	digitalWrite(G_LED_CAN, HIGH);
	digitalWrite(Y_LED_CAN, HIGH);

	delay(1000);
	digitalWrite(RED_LED, LOW);
	digitalWrite(GREEN_LED, LOW);
	digitalWrite(BLUE_LED, LOW);
	digitalWrite(G_LED_CAN, LOW);
	digitalWrite(Y_LED_CAN, LOW);
}

// the loop function runs over and over again until power down or reset
void loop() {
	wdt_reset(); //reset watchdog timer



	int potentiometer = analogRead(MANUAL_POT); //variable that stores potentiometer value (0-1023)// read the input on analog pin 0:
	int potPercent = map(potentiometer, 0, 1023, 0, 100);
	int potState = 0;


	if (potPercent <= 10) potState = 0; //off
	if (potPercent > 10 && potPercent < 90) potState = 1; //manual control
	if (potPercent >= 90 && potPercent < 95) potState = 2; //auto control based on air temperature
	if (potPercent >= 95) potState = 3; //full on

	switch (potState)
	{
	case 0: //off state. Relay open (triac closed, safety feature if relays fails)
		offFlag = true;
		digitalWrite(RELAY, !LOW);
		digitalWrite(TRIAC_DRIVER, HIGH); //safety to prevent motor temp rising due tho triac leackage current (motor is aircooled) if relay fails.

		digitalWrite(RED_LED, LOW);
		digitalWrite(GREEN_LED, LOW);
		digitalWrite(BLUE_LED, HIGH);
		break;
	case 1: //Manual control
		digitalWrite(RELAY, !HIGH);
		offFlag = false;

		triacDelay = map(potPercent, 10, 90, 7000, 3000); //values 7000 and 3000 work best

		digitalWrite(RED_LED, LOW);
		digitalWrite(GREEN_LED, HIGH);
		digitalWrite(BLUE_LED, LOW);
		break;
	case 2:
		
		break;
	case 3: //full on state
		digitalWrite(RELAY, !HIGH);
		offFlag = true;
		digitalWrite(TRIAC_DRIVER, HIGH);
		digitalWrite(RED_LED, HIGH);
		digitalWrite(GREEN_LED, LOW);
		digitalWrite(BLUE_LED, LOW);
		break;
	default:
		break;
	}


	Serial.print("Triacdelay:");
	Serial.println(triacDelay);

	Serial.print("Motor Temp C: ");
	Serial.print(getMotorTemp());
	Serial.print("\t");

	Serial.print("Air Temp C: ");
	Serial.print(getAirTemp());
	Serial.print("\t");

	Serial.print("Humidity %: ");
	Serial.println(getHumidity());

	// print out the value you read:
	//Serial.print("pot:");
	//Serial.println(potentiometer);
}


/*
Set watchdog to 8S before causing an ISR
*/
void configure_wdt(void)
{

	cli();                           // disable interrupts for changing the registers

	MCUSR = 0;                       // reset status register flags

									 // Put timer in interrupt-only mode:                                       
	WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
									 // using bitwise OR assignment (leaves other bits unchanged).
	WDTCSR = 0b01000000 | 0b100001; // set WDIE: interrupt enabled
									// clr WDE: reset disabled
									// and set delay interval (right side of bar) to 8 seconds

	sei();                           // re-enable interrupts 
}


/*
50HZ equals 20ms
+---------------+
|   XX          |
|  X  X         |
| X    X    X   |
|       X  X    |
|        XX     |
+---------------+
0    10   20ms

By detection of a zero cross. the detector gives a high puls (of 500uS) [check onenote for scope screenshots
A triac turns can be turned on afther the voltage has reached the turn on threshold (actually the current provided at the gate) check datasheet.

Because we are switching an inductive load there is an phase shift between voltage and current. controller based on time isn't good enough.


*/
void zerocross() {
	if (offFlag == false) {
		delayMicroseconds(triacDelay);
		digitalWrite(TRIAC_DRIVER, HIGH);
		delay(4); //turn on puls, check datasheet for minimum turn on puls width 
		digitalWrite(TRIAC_DRIVER, LOW);
	}
}
/*returns the temperature of the motor in celcius (DS1820 onewire)*/
float getMotorTemp()
{
	sensors.requestTemperatures(); // Send the command to get temperatures
	return sensors.getTempC(insideThermometer);
}

/*
return temp form the SI7021 sensor
*/
double getAirTemp() {
	/**Send command of initiating temperature measurement**/
	Wire.beginTransmission(ADDRI2C);
	Wire.write(0xE3);
	Wire.endTransmission();

	Wire.requestFrom(ADDRI2C, 2);

	int X0 = 0;
	int X1 = 0;
	double X = 0;
	double X_out = 0;
	if (Wire.available() <= 2);
	{
		X0 = Wire.read();
		X1 = Wire.read();
		X0 = X0 << 8;
		X_out = X0 + X1;
	}

	//Calculate temperature
	X = (175.72*X_out) / 65536;
	X = X - 46.85;
	return X;
}

double getHumidity() {
	/**Send command of initiating relative humidity measurement**/
	Wire.beginTransmission(ADDRI2C);
	Wire.write(0xE5);
	Wire.endTransmission();

	/**Read data of relative humidity**/
	Wire.requestFrom(ADDRI2C, 2);

	int Y0 = 0;
	int Y1 = 0;
	int Y2 = 0;
	double Y_out1 = 0;
	double Y_out2 = 0;
	double Y = 0;

	if (Wire.available() <= 2);
	{
		Y0 = Wire.read();
		Y2 = Y0 / 100;
		Y0 = Y0 % 100;
		Y1 = Wire.read();
		Y_out1 = Y2 * 25600;
		Y_out2 = Y0 * 256 + Y1;
	}

	/**Calculate and display relative humidity**/
	Y_out1 = (125 * Y_out1) / 65536;
	Y_out2 = (125 * Y_out2) / 65536;
	Y = Y_out1 + Y_out2;
	Y = Y - 6;
	return Y;
}


ISR(WDT_vect)
{
	//do special ISR stuff here, when MCU gets stuck
	//temp:
	digitalWrite(RELAY, !HIGH);
	digitalWrite(TRIAC_DRIVER, HIGH);

	digitalWrite(RED_LED, HIGH);
	digitalWrite(GREEN_LED, HIGH);
	digitalWrite(BLUE_LED, HIGH);
	digitalWrite(G_LED_CAN, LOW);
	digitalWrite(Y_LED_CAN, LOW);

	long i = 0;
	for (i = 0; i < 300000; i++) //delay for 5minutes
	{
		_delay_ms(1);
	}



	// Reboot MCU
	// configure
	MCUSR = 0;                          // reset flags

										// Put timer in reset-only mode:
	WDTCSR |= 0b00011000;               // Enter config mode.
	WDTCSR = 0b00001000 | 0b000000;    // clr WDIE (interrupt enable...7th from left)
									   // set WDE (reset enable...4th from left), and set delay interval
									   // reset system in 16 ms...
									   // unless wdt_disable() in loop() is reached first

									   // reboot
	while (1);
}