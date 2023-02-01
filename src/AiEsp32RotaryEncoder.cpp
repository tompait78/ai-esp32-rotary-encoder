// based on https://github.com/marcmerlin/IoTuz code - extracted and modified Encoder code
//
//

#include "esp_log.h"
#define LOG_TAG "AiEsp32RotaryEncoder"

#include "AiEsp32RotaryEncoder.h"

void IRAM_ATTR AiEsp32RotaryEncoder::readEncoder_ISR() {

	unsigned long now = millis();
	portENTER_CRITICAL_ISR(&(this->mux));
	if (this->isEnabled) {
		this->old_AB <<= 2; //remember previous state
		int8_t ENC_PORT = (digitalRead(this->encoderBPin) ? (1 << 1) : 0) | (digitalRead(this->encoderAPin) ? (1 << 0) : 0);
		this->old_AB |= (ENC_PORT & 0x03); //add current state
		int8_t currentDirection = (this->enc_states[(this->old_AB & 0x0f)]); //-1,0 or 1
		if (currentDirection != 0) {
			long prevRotaryPosition = this->encoder0Pos / this->encoderSteps;
			this->encoder0Pos += currentDirection;
			long newRotaryPosition = this->encoder0Pos / this->encoderSteps;

			//respect limits
			if (this->encoder0Pos > this->_maxEncoderValue)
				this->encoder0Pos = this->_maxEncoderValue;
			if (this->encoder0Pos < this->_minEncoderValue)
				this->encoder0Pos = this->_minEncoderValue;
		}
	}
	portEXIT_CRITICAL_ISR(&(this->mux));
}

void IRAM_ATTR AiEsp32RotaryEncoder::readButton_ISR() {
	portENTER_CRITICAL_ISR(&(this->buttonMux));

	uint8_t butt_state = !digitalRead(this->encoderButtonPin);
	if (!this->isEnabled) {
		buttonState = BUT_DISABLED;
	} else if (butt_state && !this->previous_butt_state) {
		this->previous_butt_state = true;
		Serial.println("Button Pushed");
		buttonState = BUT_PUSHED;
	} else if (!butt_state && this->previous_butt_state) {
		this->previous_butt_state = false;
		Serial.println("Button Released");
		buttonState = BUT_RELEASED;
	} else {
		buttonState = (butt_state ? BUT_DOWN : BUT_UP);
		Serial.println(butt_state ? "BUT_DOWN" : "BUT_UP");
	}

	portEXIT_CRITICAL_ISR(&(this->buttonMux));
}

AiEsp32RotaryEncoder::AiEsp32RotaryEncoder(uint8_t encoder_APin, uint8_t encoder_BPin, uint8_t encoder_ButtonPin) {
	this->old_AB = 0;

	this->encoderAPin = encoder_APin;
	this->encoderBPin = encoder_BPin;
	this->encoderButtonPin = encoder_ButtonPin;

	pinMode(this->encoderAPin, INPUT_PULLDOWN);
	pinMode(this->encoderBPin, INPUT_PULLDOWN);
}

void AiEsp32RotaryEncoder::setBoundaries(long minEncoderValue, long maxEncoderValue, bool circleValues) {
	this->_minEncoderValue = minEncoderValue * this->encoderSteps;
	this->_maxEncoderValue = maxEncoderValue * this->encoderSteps;
}

void AiEsp32RotaryEncoder::setSteps(long steps) {
	this->encoderSteps = steps;
}

long AiEsp32RotaryEncoder::readEncoder() {
	return (this->encoder0Pos / this->encoderSteps);
}

void AiEsp32RotaryEncoder::setEncoderValue(long newValue) {
	reset(newValue);
}

long AiEsp32RotaryEncoder::encoderChanged() {
	long _encoder0Pos = readEncoder();
	long encoder0Diff = _encoder0Pos - this->lastReadEncoder0Pos;

	this->lastReadEncoder0Pos = _encoder0Pos;

	return encoder0Diff;
}

void AiEsp32RotaryEncoder::setup(void (*ISR_callback)(void), void (*ISR_button)(void)) {
	this->lastReadEncoder0Pos = 0;
	// Initialize rotary encoder reading and decoding
	this->previous_butt_state = 0;
	if (this->encoderButtonPin >= 0) {
		pinMode(this->encoderButtonPin, INPUT_PULLUP);
	}
	attachInterrupt(digitalPinToInterrupt(this->encoderAPin), ISR_callback, CHANGE);
	attachInterrupt(digitalPinToInterrupt(this->encoderBPin), ISR_callback, CHANGE);
	attachInterrupt(digitalPinToInterrupt(this->encoderButtonPin), ISR_button, RISING);
}

ButtonState AiEsp32RotaryEncoder::currentButtonState()
{
	return buttonState;
}

ButtonState AiEsp32RotaryEncoder::readButtonState()
{
	ButtonState _buttonState = buttonState;
	return buttonState;
}

void AiEsp32RotaryEncoder::reset(long newValue_) {
	newValue_ = newValue_ * this->encoderSteps;
	this->encoder0Pos = newValue_;
	this->lastReadEncoder0Pos = this->encoder0Pos;
	if (this->encoder0Pos > this->_maxEncoderValue)
		this->encoder0Pos = this->_maxEncoderValue;
	if (this->encoder0Pos < this->_minEncoderValue)
		this->encoder0Pos = this->_minEncoderValue;
}

long AiEsp32RotaryEncoder::checkEncoderButtonClicked(unsigned long minWaitMilliseconds) {
	if (millis() < noClickUntil) return 0; //untracked click

	int buttonCurrentState = digitalRead(encoderButtonPin);
	//ESP_LOGD(TAG, "Button state: %d -> %d", buttonLastState, buttonCurrentState);
  	if (buttonCurrentState == HIGH) // not pressed
    	return 0; //no click
  	buttonLastState = buttonCurrentState; // save the the last state
	noClickUntil = millis() + minWaitMilliseconds;
	int pressCounter = 0;
	while (millis() < noClickUntil) {
		int buttonCurrentState = digitalRead(encoderButtonPin);
		if (buttonLastState == HIGH && buttonCurrentState == LOW) { // button is pressed
			//ESP_LOGD(TAG, "Button pressed: %d", pressCounter);
		} else if (buttonLastState == LOW && buttonCurrentState == HIGH) { // button is released
			noClickUntil = millis() + minWaitMilliseconds;
			pressCounter++;
			//ESP_LOGD(TAG, "Button released: %d", pressCounter);
		}
		buttonLastState = buttonCurrentState; // save the the last state
	}
	//ESP_LOGD(TAG, "Button timeout: %d", pressCounter);
	noClickUntil = millis() + 2 * minWaitMilliseconds;
	return pressCounter
		? pressCounter //short clicks
		: -1; //long press
}

