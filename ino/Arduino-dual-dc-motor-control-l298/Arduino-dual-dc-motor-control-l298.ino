/*Title: DC Motor Control with Encoder (Single)*/
/*Author: Khairul Izwan 10-03-2020*/
/*Description: Controlling DC Motor with Encoder using */

/*Parts*/
/*1. DC Motor:: 12V 38RPM 5kgfcm Brushed DC Geared Motor with Encoder :: https://my.cytron.io/p-12v-38rpm-5kgfcm-brushed-dc-geared-motor-with-encoder?search=12v%2038RPM&description=1&src=search.list */
/*2. Driver:: Shield L298P Motor Driver with GPIO :: https://my.cytron.io/p-shield-l298p-motor-driver-with-gpio?src=search.list*/

/*Notes:*/
/*D10, D11, D12 and D13 for motor control*/
/*D10 controls speed of motor A and D11 controls speed of motor B.*/
/*Pin D12 controls direction of motor A and Pin D13 controls direction of motor B.*/

/*Pin Assignment*/
int PWMA = 10; // Left
int DIRA = 12;
int PWMB = 11; // Right
int DIRB = 13;

/*Speed*/
int SPD = 0;

void setup()
{
	pinMode(DIRA, OUTPUT);
	pinMode(DIRB, OUTPUT);
	pinMode(PWMA, OUTPUT);
	pinMode(PWMB, OUTPUT);
}

void loop()
{
	{
/*		Direction Control*/
		digitalWrite(DIRA, LOW);
		digitalWrite(DIRB, HIGH);
/*		PWM Speed Control*/
		analogWrite(PWMA, SPD);
		analogWrite(PWMB, SPD);
	}
}
