#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define ST7735_DARKGREY  0x7BEF

#define SPI_SCK   40
#define SPI_MOSI  41
#define TFT_CS    17
#define TFT_DC    16
#define TFT_RST   18

#define NRF_CE    47
#define NRF_CSN   48
#define NRF_SCK   40
#define NRF_MOSI  41
#define NRF_MISO  42

#define JOY1_X_PIN   1
#define JOY1_Y_PIN   2
#define JOY2_X_PIN   3
#define JOY2_Y_PIN   4
#define POT_PIN      6

#define JOY1_SW_PIN  7
#define JOY2_SW_PIN  8
#define BTN1_PIN     12
#define BTN2_PIN     13
#define BTN3_PIN     14
#define BTN4_PIN     15
#define TOGGLE1_PIN  38
#define TOGGLE2_PIN  21

// ── Tuning ───────────────────────────────────────────
// Joystick center on a 12-bit ADC is ~2048.
// Deadzone: ignore inputs within ±200 of center.
// Pot center: pots usually rest at 0 or 4095, not center.
// Set POT_MIN_MOVE to 50 — any pot reading above 50 is "active".
#define JOY_CENTER    2048
#define JOY_DEAD      200     // joystick deadzone radius
#define POT_MIN_MOVE  50      // pot must be above this to count

Adafruit_ST7735 tft(TFT_CS, TFT_DC, SPI_MOSI, SPI_SCK, TFT_RST);
SPIClass nrfSPI(HSPI);
RF24 radio(NRF_CE, NRF_CSN);

const byte address[6] = "00001";

struct Payload {
  uint16_t joy1x, joy1y;
  uint16_t joy2x, joy2y;
  uint16_t pot;
  uint8_t  buttons;
};

// ── NEW: "most deflected input wins" ─────────────────
// Each joystick axis reports how far it is from center (0–2048).
// The axis with the largest deflection controls the angle.
// Pot overrides everything except buttons.
// Buttons are absolute top priority.
int computeAngle(Payload& d) {

  // --- Step 1: buttons (absolute priority) ---
  if ((d.buttons >> 5) & 1) return 180;
  if ((d.buttons >> 4) & 1) return 120;
  if ((d.buttons >> 3) & 1) return 60;
  if ((d.buttons >> 2) & 1) return 0;
  if ((d.buttons >> 7) & 1) return 135;
  if ((d.buttons >> 6) & 1) return 45;

  // --- Step 2: pot (second priority, only when not centered) ---
  // Pots sweep 0→4095. If yours rests near 2048, lower POT_MIN_MOVE.
  // If yours rests near 0, this is correct as-is.
  if (d.pot > POT_MIN_MOVE) {
    return map(d.pot, 0, 4095, 0, 180);
  }

  // --- Step 3: joysticks — winner = most deflected axis ---
  int axes[4] = { (int)d.joy1x, (int)d.joy1y,
                  (int)d.joy2x, (int)d.joy2y };

  int bestDeflection = JOY_DEAD;  // must beat deadzone to count
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

  // --- Step 4: nothing active → hold last angle or center ---
  return 90;
}

// ── Display ──────────────────────────────────────────
#define PIVOT_X  80   // tft.width()/2 for 160px landscape
#define PIVOT_Y  88   // tft.height()-40 gives room for buttons
#define ARM_LEN  45

int  lastAngle   = -999;  // force first draw
bool lastBtns[8] = {};

void drawServoArm(int angleDeg, uint16_t color) {
  float rad = (angleDeg * PI) / 180.0;
  int x2 = PIVOT_X + (int)(ARM_LEN * cos(PI - rad));
  int y2 = PIVOT_Y - (int)(ARM_LEN * sin(rad));
  tft.drawLine(PIVOT_X, PIVOT_Y, x2, y2, color);
  tft.fillCircle(x2, y2, 3, color);
}

void drawStaticUI() {
  tft.fillScreen(ST7735_BLACK);

  tft.setTextColor(ST7735_CYAN);
  tft.setTextSize(1);
  tft.setCursor(10, 4);
  tft.print("SERVO CONTROLLER");
  tft.drawFastHLine(0, 14, 160, ST7735_CYAN);

  // Arc
  for (int a = 0; a <= 180; a += 5) {
    float r = (a * PI) / 180.0;
    int x = PIVOT_X + (int)((ARM_LEN + 8) * cos(PI - r));
    int y = PIVOT_Y - (int)((ARM_LEN + 8) * sin(r));
    tft.drawPixel(x, y, ST7735_DARKGREY);
  }

  tft.fillCircle(PIVOT_X, PIVOT_Y, 4, ST7735_WHITE);

  tft.setTextColor(ST7735_YELLOW);
  tft.setCursor(2,         PIVOT_Y - 6);            tft.print("0");
  tft.setCursor(PIVOT_X-6, PIVOT_Y - ARM_LEN - 10); tft.print("90");
  tft.setCursor(146,       PIVOT_Y - 6);            tft.print("180");

  tft.setTextColor(ST7735_DARKGREY);
  tft.setCursor(2, PIVOT_Y + 10);
  tft.print("J1  J2  B1 B2 B3 B4 T1 T2");
}

void updateAngle(int angle) {
  // Always erase old arm and draw new one — no skip
  if (lastAngle != -999) {
    drawServoArm(lastAngle, ST7735_BLACK);
    // Re-draw the pivot dot in case arm erased it
    tft.fillCircle(PIVOT_X, PIVOT_Y, 4, ST7735_WHITE);
  }
  drawServoArm(angle, ST7735_GREEN);

  // Angle number — clear box first then print
  tft.fillRect(55, PIVOT_Y - 16, 55, 10, ST7735_BLACK);
  tft.setTextColor(ST7735_GREEN);
  tft.setTextSize(1);
  tft.setCursor(55, PIVOT_Y - 16);
  tft.print(angle);
  tft.print(" deg");

  lastAngle = angle;
}

void updateButtons(uint8_t btnByte) {
  const char* labels[] = {"J1","J2","B1","B2","B3","B4","T1","T2"};
  int xPos[] = {2,16,30,42,54,66,78,90};
  int yPos = PIVOT_Y + 20;

  for (int i = 0; i < 8; i++) {
    bool pressed = (btnByte >> i) & 1;
    if (pressed != lastBtns[i]) {
      tft.fillRect(xPos[i], yPos, 12, 8,
                   pressed ? ST7735_GREEN : ST7735_BLACK);
      tft.setTextColor(pressed ? ST7735_BLACK : ST7735_DARKGREY);
      tft.setCursor(xPos[i], yPos);
      tft.print(labels[i]);
      lastBtns[i] = pressed;
    }
  }
}

void setup() {
  Serial.begin(115200);

  int btnPins[] = {JOY1_SW_PIN, JOY2_SW_PIN,
                   BTN1_PIN, BTN2_PIN, BTN3_PIN, BTN4_PIN,
                   TOGGLE1_PIN, TOGGLE2_PIN};
  for (int p : btnPins) pinMode(p, INPUT_PULLUP);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  drawStaticUI();

  nrfSPI.begin(NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN);
  if (!radio.begin(&nrfSPI)) {
    tft.setTextColor(ST7735_RED);
    tft.setCursor(20, 55);
    tft.print("NRF24 FAIL!");
    Serial.println("NRF24 not detected!");
    while (1) delay(500);
  }

  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();

  tft.setTextColor(ST7735_GREEN);
  tft.setCursor(45, 55);
  tft.print("TX READY");
  delay(1000);
  drawStaticUI();
  updateAngle(90);  // draw arm at center immediately
}

void loop() {
  Payload data;
  data.joy1x = analogRead(JOY1_X_PIN);
  data.joy1y = analogRead(JOY1_Y_PIN);
  data.joy2x = analogRead(JOY2_X_PIN);
  data.joy2y = analogRead(JOY2_Y_PIN);
  data.pot   = analogRead(POT_PIN);

  data.buttons = 0;
  data.buttons |= (!digitalRead(JOY1_SW_PIN)) << 0;
  data.buttons |= (!digitalRead(JOY2_SW_PIN)) << 1;
  data.buttons |= (!digitalRead(BTN1_PIN))    << 2;
  data.buttons |= (!digitalRead(BTN2_PIN))    << 3;
  data.buttons |= (!digitalRead(BTN3_PIN))    << 4;
  data.buttons |= (!digitalRead(BTN4_PIN))    << 5;
  data.buttons |= (!digitalRead(TOGGLE1_PIN)) << 6;
  data.buttons |= (!digitalRead(TOGGLE2_PIN)) << 7;

  int angle = computeAngle(data);

  // Debug — open Serial Monitor to verify values are changing
  Serial.print("joy1x="); Serial.print(data.joy1x);
  Serial.print(" joy1y="); Serial.print(data.joy1y);
  Serial.print(" joy2x="); Serial.print(data.joy2x);
  Serial.print(" joy2y="); Serial.print(data.joy2y);
  Serial.print(" pot=");   Serial.print(data.pot);
  Serial.print(" btns=");  Serial.print(data.buttons, BIN);
  Serial.print(" → angle="); Serial.println(angle);

  updateAngle(angle);
  updateButtons(data.buttons);
  radio.write(&data, sizeof(data));

  delay(30);
}