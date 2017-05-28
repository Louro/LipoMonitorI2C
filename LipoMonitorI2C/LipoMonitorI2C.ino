/*
 Name:		LipoMonitorI2C.ino
 Created:	5/28/2017 9:51:23 AM
 Author:	pjc
*/

#include "LowPower.h"
#include <avr/sleep.h>
#include <avr/power.h>


//ADuM1251

// the setup function runs once when you press reset or power the board
void setup() {
	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(0, INPUT_PULLUP);

	digitalWrite(LED_BUILTIN, LOW);

	Serial.begin(9600);
	Serial.println("Starting...");
	blink_battery_voltage(readVcc());

}

ISR(PCINT2_vect)
{
	//Serial.println("interrupt..."); 
}  // end of PCINT2_vect

unsigned long lastSleep;
const unsigned long WAIT_TIME = 5000;
unsigned int led = 0;
unsigned char incomingByte = 0;
unsigned char recState = 1;
unsigned char flag = 0;

#define myAddress '1'

#define rec_lock 1
#define rec_addr 2
#define rec_send 3

// the loop function runs over and over again until power down or reset
void loop() {

	noInterrupts();
	// pin change interrupt (example for D0)
	PCMSK2 |= bit(PCINT16); // want pin 0
	PCIFR |= bit(PCIF2);   // clear any outstanding interrupts
	PCICR |= bit(PCIE2);   // enable pin change interrupts for D0 to D7
	interrupts();

	//Serial.println("Sleeping"); 
	while (!(UCSR0A & _BV(TXC0)));
	digitalWrite(LED_BUILTIN, LOW);

	// Enter power down state forever with ADC and BOD module disabled
	LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

	//Serial.println("Wakeup"); 

	digitalWrite(LED_BUILTIN, HIGH);

	//sendBatteryLevel();

	lastSleep = millis();

	while (millis() - lastSleep < WAIT_TIME)
	{
		if (Serial.available() > 0) {
			// read the incoming byte:
			incomingByte = Serial.read();
			//Serial.write(incomingByte);
			switch (recState) {
			case rec_lock: if (incomingByte == 0x55)
				recState = rec_addr;
				break;
			case rec_addr: if (incomingByte == myAddress)
				recState = rec_send;
						   else {
							   recState = rec_lock;
							   break;
						   }
			case rec_send: sendBatteryLevel();
				recState = rec_lock; flag = 1;
				break;
			}
		}
		if (flag) {
			flag = 0;
			break;
		}
	}

}

void sendBatteryLevel() {
	Serial.println(readVcc(), DEC);
}

long readVcc() {
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	ADMUX = _BV(MUX5) | _BV(MUX0);
#else
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA, ADSC)); // measuring

	uint8_t low = ADCL; // must read ADCL first - it then locks ADCH  
	uint8_t high = ADCH; // unlocks both

	long result = (high << 8) | low;

	result = 1093000L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	return result; // Vcc in millivolts
}

#define blink() digitalWrite(LED_BUILTIN,HIGH); delay(400); digitalWrite(LED_BUILTIN,LOW); delay(400);

void blink_battery_voltage(long volt) {
	unsigned int decimal;
	unsigned int fractional;
	unsigned int i;
	if (volt < 1000)
		return;

	fractional = volt / 1000;
	decimal = (volt - (fractional * 1000)) / 100;

	for (i = 0; i<fractional; i++) {
		blink();
	}

	delay(800);

	for (i = 0; i<decimal; i++) {
		blink();
	}
}



