#include "Arduino.h"

#include "AiEsp32RotaryEncoder.h"
// https://github.com/igorantolic/ai-esp32-rotary-encoder/blob/master/examples/Esp32RotaryEncoderBasics/Esp32RotaryEncoderBasics.ino
#define PIN_A				1
#define PIN_B				2
#define PIN_BUTTON			0

//instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(PIN_A, PIN_B, PIN_BUTTON);

void rotary_onButtonClick() {
	static unsigned long lastTimePressed = 0;
	//ignore multiple press in that time milliseconds
	if (millis() - lastTimePressed < 500) return;
	lastTimePressed = millis();
	Serial.print("button pressed ");
	Serial.print(millis());
	Serial.println(" milliseconds after restart");
}

void rotary_loop() {
	//dont print anything unless value changed
	if (rotaryEncoder.encoderChanged()) {
		Serial.print("Value: ");
		Serial.println(rotaryEncoder.readEncoder());
	}
	int pressCounter = rotaryEncoder.checkEncoderButtonClicked();
	//if (pressCounter > 2) pressCounter = 2;
	if (pressCounter != 0) {
		Serial.printf("ButtonClick: %d\n", pressCounter);
	}
}

void IRAM_ATTR readEncoderISR() {
	rotaryEncoder.readEncoder_ISR();
}
void IRAM_ATTR readButtonISR() {
	rotaryEncoder.readEncoder_ISR();
}

void setup() {
	Serial.begin(115200);
	//we must initialize rotary encoder
	rotaryEncoder.setup(readEncoderISR, readButtonISR);
  	rotaryEncoder.setBoundaries(0, 100, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
}

void loop() {
	rotary_loop();
	delay(50); //or do whatever you need to do...
}