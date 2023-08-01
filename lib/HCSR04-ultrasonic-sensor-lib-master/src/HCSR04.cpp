#include "HCSR04.h"
#define other_trig_pin 32
float temp=19.307,maxTimeoutMicroSec=23529.4,maxDistanceCm=400,humidity=94;
////////////////////////////////////consttruct/destruct
void HCSR04::init(int out[], int echo[], int n)
{
	this->out = out;
	this->echo = echo;
	this->n = n;
	for(int i = 0; i < 2; i++)
	{
	pinMode(this->out[i], OUTPUT);
	}
	for (int i = 0; i < n; i++)
		pinMode(this->echo[i], INPUT);
}
HCSR04::HCSR04(int out, int echo) { this->init(new int[1]{out}, new int[1]{echo}, 1); }
HCSR04::HCSR04(int out[], int echo[], int n) { this->init(out, echo, n); }
HCSR04::~HCSR04()
{
	~*this->out;
	delete[] this->echo;
	~this->n;
}

///////////////////////////////////////////////////dist
float HCSR04::dist(int n) const
{	
	unsigned long maxDistanceDurationMicroSec;
	// for(int i = 0; i < 2; i++)
	// {
	digitalWrite(this->out[1], LOW);digitalWrite(this->out[0], LOW);
	// }
	digitalWrite(other_trig_pin, LOW);
	delayMicroseconds(2);
	// for(int i = 0; i < 2; i++)
	// {
	digitalWrite(this->out[1], HIGH);digitalWrite(this->out[0], HIGH);
	// }
	digitalWrite(other_trig_pin, HIGH);
	delayMicroseconds(10);
	// for(int i = 0; i < 2; i++)
	// {
	digitalWrite(this->out[1], LOW);digitalWrite(this->out[0], LOW);
	digitalWrite(other_trig_pin, LOW);
	// }
	                          // noInterrupts();
	//old code ends here      // float d = pulseIn(this->echo[n], HIGH, 23529.4); // max sensor dist ~4m
	                          // interrupts();
	                          // return d / 58.8235;
    float speedOfSoundInCmPerMicroSec = 0.03313 + 0.0000606 * temp + 0.000001243*humidity; // Cair ≈ (331.3 + 0.606 ⋅ ϑ) m/s

    // Compute max delay based on max distance with 25% margin in microseconds
    maxDistanceDurationMicroSec = 2.5 * maxDistanceCm / speedOfSoundInCmPerMicroSec;
    if (maxTimeoutMicroSec > 0) {
    	maxDistanceDurationMicroSec = maxDistanceDurationMicroSec>maxTimeoutMicroSec?maxTimeoutMicroSec:maxDistanceDurationMicroSec;
    }

    // Measure the length of echo signal, which is equal to the time needed for sound to go there and back.
    unsigned long durationMicroSec = pulseIn(this->echo[n], HIGH, maxDistanceDurationMicroSec); // can't measure beyond max distance

    float distanceCm = durationMicroSec / 2.0 * speedOfSoundInCmPerMicroSec;
    if (distanceCm == 0 || distanceCm > maxDistanceCm) {
        return -1.0 ;
    } else {
        return distanceCm;
    }

}
float HCSR04::dist() const { return this->dist(0); }
