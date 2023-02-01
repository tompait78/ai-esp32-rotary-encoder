// AiEsp32RotaryEncoder.h
// https://github.com/igorantolic/ai-esp32-rotary-encoder/blob/master/examples/Esp32RotaryEncoderBasics/Esp32RotaryEncoderBasics.ino

#ifndef _AIESP32ROTARYENCODER_h
#define _AIESP32ROTARYENCODER_h

#include "Arduino.h"

// Rotary Encocer
#define AIESP32ROTARYENCODER_DEFAULT_A_PIN 1
#define AIESP32ROTARYENCODER_DEFAULT_B_PIN 2
#define AIESP32ROTARYENCODER_DEFAULT_BUT_PIN 0
#define AIESP32ROTARYENCODER_DEFAULT_STEPS 2

typedef enum
{
	BUT_DOWN = 0,
	BUT_PUSHED = 1,
	BUT_UP = 2,
	BUT_RELEASED = 3,
	BUT_DISABLED = 99,
} ButtonState;

class AiEsp32RotaryEncoder
{

private:
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;
	volatile long encoder0Pos = 0;

	volatile int8_t lastMovementDirection = 0; //1 right; -1 left
	volatile unsigned long lastMovementAt = 0;

	int buttonLastState = LOW;  // the previous state from the input pin
	unsigned long buttonPressedTime  = 0;
	unsigned long buttonReleasedTime = 0;
	unsigned long noClickUntil = 0;

	unsigned long buttonTime = 0;

	bool isEnabled = true;

	uint8_t encoderAPin = AIESP32ROTARYENCODER_DEFAULT_A_PIN;
	uint8_t encoderBPin = AIESP32ROTARYENCODER_DEFAULT_B_PIN;
	uint8_t encoderButtonPin = AIESP32ROTARYENCODER_DEFAULT_BUT_PIN;
	long encoderSteps = AIESP32ROTARYENCODER_DEFAULT_STEPS;

	long _minEncoderValue = -1 << 15;
	long _maxEncoderValue = 1 << 15;

	uint8_t old_AB;
	long lastReadEncoder0Pos;
	bool previous_butt_state;

	ButtonState buttonState;

	int8_t enc_states[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
	void (*ISR_callback)();
	void (*ISR_button)();

public:
	AiEsp32RotaryEncoder(
		uint8_t encoderAPin = AIESP32ROTARYENCODER_DEFAULT_A_PIN,
		uint8_t encoderBPin = AIESP32ROTARYENCODER_DEFAULT_B_PIN,
		uint8_t encoderButtonPin = AIESP32ROTARYENCODER_DEFAULT_BUT_PIN);
	void setSteps(long steps = 1);
	void setBoundaries(long minValue = -100, long maxValue = 100, bool circleValues = false);
	void IRAM_ATTR readEncoder_ISR();
	void IRAM_ATTR readButton_ISR();

	void setup(void (*ISR_callback)(void), void (*ISR_button)(void));
	void reset(long newValue = 0);
	long readEncoder();
	void setEncoderValue(long newValue);
	long encoderChanged();
	ButtonState currentButtonState();
	ButtonState readButtonState();

	long checkEncoderButtonClicked(unsigned long minWaitMilliseconds = 500);
	int isEncoderButtonDown();
};
#endif
