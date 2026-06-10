#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#define NRF_CE    3
#define NRF_CSN   2
#define SERVO_PIN 6

#define JOY_CENTER    2048
#define JOY_DEAD      200
#define POT_MIN_MOVE  50

#define SIGNAL_TIMEOUT_MS 500

RF24  radio(NRF_CE, NRF_CSN);
Servo servo;

const byte address[6] = "00001";

unsigned long lastReceived = 0;

struct Payload {
  uint16_t joy1x, joy1y;
  uint16_t joy2x, joy2y;
  uint16_t pot;
  uint8_t  buttons;
};

int computeAngle(Payload& d) {

  if ((d.buttons >> 5) & 1) return 180;
  if ((d.buttons >> 4) & 1) return 120;
  if ((d.buttons >> 3) & 1) return 60;
  if ((d.buttons >> 2) & 1) return 0;
  if ((d.buttons >> 7) & 1) return 135;
  if ((d.buttons >> 6) & 1) return 45;

  if (d.pot > POT_MIN_MOVE) {
    return map(d.pot, 0, 4095, 0, 180);
  }

  int axes[4] = { (int)d.joy1x, (int)d.joy1y,
                  (int)d.joy2x, (int)d.joy2y };

  int bestDeflection = JOY_DEAD;
  int bestAxis       = -1;

  for (int i = 0; i < 4; i++) {
    int deflection = abs(axes[i] - JOY_CENTER);
    if (deflection > bestDeflection) {
      bestDeflection = deflection;
      bestAxis       = i;
    }
  }

  if (bestAxis >= 0) {
    return map(axes[bestAxis], 0, 4095, 0, 180);
  }

  return 90;
}

void setup() {
  Serial.begin(115200);

  servo.attach(SERVO_PIN, 544, 2400);
  servo.write(90);

  if (!radio.begin()) {
    Serial.println("NRF24 not detected! Check CE=3, CSN=2 wiring.");
    while (1) delay(500);
  }

  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  lastReceived = millis();
  Serial.println("RX ready.");
}

void loop() {
  if (radio.available()) {
    Payload data;
    radio.read(&data, sizeof(data));
    lastReceived = millis();

    int angle = computeAngle(data);
    servo.write(angle);

    Serial.print("joy1x="); Serial.print(data.joy1x);
    Serial.print(" joy1y="); Serial.print(data.joy1y);
    Serial.print(" joy2x="); Serial.print(data.joy2x);
    Serial.print(" joy2y="); Serial.print(data.joy2y);
    Serial.print(" pot=");   Serial.print(data.pot);
    Serial.print(" btns=");  Serial.print(data.buttons, BIN);
    Serial.print(" → angle="); Serial.println(angle);
  }

  // Failsafe
  if (millis() - lastReceived > SIGNAL_TIMEOUT_MS) {
    servo.write(90);
    Serial.println("SIGNAL LOST — centering servo");
    lastReceived = millis();  // prevent flood of messages
  }
}