#include <cy_serdebug.h>
#include <cy_serial.h>

#include <Adafruit_NeoPixel.h>
const char *gc_hostname = "NghtLghtNeo";
#include "cy_wifi.h"
#include "cy_ota.h"
#include <Ticker.h>

#define btnpin 4
#define ledpinbl 13
#define ledpinrt 15
#define ledpingn 12
#define LDRPin (A0)

#define pirpin 14

//#define lightpin1 16
//#define lightpin2 5
#define NeoPIN 5 // Neopixel data pin
#define NeoCnt 6

// Parameter for Timing, ...
#define LDRThres 5
// Light On Time in s
#define OnTimeLight 25

int LDRValue;
volatile boolean gv_PIR_Int;
boolean gv_PIR_on = false;
boolean gv_light_on = false;

const int CMD_WAIT = 0;
const int CMD_BUTTON_CHANGE = 1;
int cmd = CMD_WAIT;
int buttonState = HIGH;
static long startPress = 0;




Ticker ticker_piroff;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NeoCnt, NeoPIN, NEO_GRB + NEO_KHZ800);

void set_rgb(int iv_red, int iv_green, int iv_blue) {

  int lv_green = map(iv_green, 0, 255, 0, PWMRANGE) * 0.8;
  int lv_red = map(iv_red, 0, 255, 0, PWMRANGE);
  int lv_blue = map(iv_blue, 0, 255, 0, PWMRANGE);

  analogWrite(ledpinrt, lv_red);
  analogWrite(ledpingn, lv_green);
  analogWrite(ledpinbl, lv_blue);
}

void toggle() {
  //  led_stat++;
  //  if (led_stat > 5) {
  //    led_stat = 0;
  //  }

}
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    if (wait > 0) {
      strip.show();
      delay(wait);
    }
  }
  if (wait == 0) {
    strip.show();
  }
}

void piroff()
{
  gv_PIR_on = false;
  ticker_piroff.detach();
}

void restart() {
  ESP.reset();
  delay(1000);
}

void reset() {
  //reset wifi credentials
  WiFi.disconnect();
  delay(1000);
  ESP.reset();
  delay(1000);
}


void IntBtn() {
  cmd = CMD_BUTTON_CHANGE;
}

void IntPIR() {
  gv_PIR_Int = true;

}

void setup() {
  // put your setup code here, to run once:

  cy_serial::start(__FILE__);


  pinMode(ledpinrt, OUTPUT);
  pinMode(ledpingn, OUTPUT);
  pinMode(ledpinbl, OUTPUT);

  //  pinMode(lightpin1, OUTPUT);
  //  pinMode(lightpin2, OUTPUT);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  colorWipe(strip.Color(255, 0, 0), 10); // Red
  delay(500);
  colorWipe(strip.Color(0, 255, 0), 10); //
  delay(500);
  colorWipe(strip.Color(0, 0, 255), 10); //
  delay(500);
  colorWipe(strip.Color(0, 0, 0), 10); //

  set_rgb(255, 255, 255);

  wifi_init(gv_clientname);

  init_ota(gv_clientname);

  set_rgb(0, 0, 0);

  strip.show(); // Initialize all pixels to 'off'

  //setup button
  pinMode(btnpin, INPUT);
  attachInterrupt(btnpin, IntBtn, CHANGE);

  //
  pinMode(pirpin, INPUT);
  attachInterrupt(pirpin, IntPIR, RISING);

  delay(500);

}

void loop() {

  check_ota();

  // Interrupt on PIR occured?
  if ( gv_PIR_Int == true ) {
    ticker_piroff.detach();
    gv_PIR_on = true;
    ticker_piroff.attach(OnTimeLight, piroff);
    gv_PIR_Int = false;
  }

  switch (cmd) {
    case CMD_WAIT:
      break;
    case CMD_BUTTON_CHANGE:
      int currentState = digitalRead(btnpin);
      if (currentState != buttonState) {
        if (buttonState == LOW && currentState == HIGH) {
          long duration = millis() - startPress;
          if (duration < 1000) {
            DebugPrintln("short press - toggle LED");
            toggle();
          } else if (duration < 5000) {
            DebugPrintln("medium press - reset");
            restart();
          } else if (duration < 60000) {
            DebugPrintln("long press - reset settings");
            reset();
          }
        } else if (buttonState == HIGH && currentState == LOW) {
          startPress = millis();
        }
        buttonState = currentState;
      }
      break;
  }

  if ( !gv_light_on ) {

    // Lights off: measure LDR
    // so turn off all LEDs
    analogWrite(ledpingn, 0);
    analogWrite(ledpinrt, 0);
    analogWrite(ledpinbl, 0);
    delay(100);

    // now measure LDR
    LDRValue = analogRead(LDRPin);

    // show result of measurement
    if ( LDRValue < LDRThres ) {
      analogWrite(ledpingn, 10);
    }
  }

  int pirState = digitalRead(pirpin);

  if (pirState == 1) {
    analogWrite(ledpinrt, 50);
  } else {
    analogWrite(ledpinrt, 0);
  }

  if (gv_PIR_on) {
    // PIR trigged, is LDR OK?
    if ( LDRValue < LDRThres ) {
      // --> Turn On Light
      analogWrite(ledpinbl, 255);
      colorWipe(strip.Color(255, 0, 0), 0); // Red
      //digitalWrite(lightpin1, HIGH);
      //digitalWrite(lightpin2, HIGH);
      gv_light_on = true;
    }

  } else {
    analogWrite(ledpinbl, 0);
    colorWipe(strip.Color(0, 0, 0), 0); // Off
    //digitalWrite(lightpin1, LOW);
    //digitalWrite(lightpin2, LOW);
    gv_light_on = false;
  }

  delay(100);

}