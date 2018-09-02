#include <cy_serdebug.h>
#include <cy_serial.h>

#include <Adafruit_NeoPixel.h>

#include "cy_wifi.h"
#include "cy_ota.h"
#include <Ticker.h>
//#include "cy_mqtt.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Pins
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Parameter for Timing, ...
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define LDRThres 20
// Light On Time in s
#define OnTimeLight 15

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// global Variables
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int LDRValue;
volatile boolean gv_PIR_Int;
boolean gv_PIR_on = false;
boolean gv_light_on = false;
boolean gv_neo_on = false;

const int CMD_WAIT = 0;
const int CMD_BUTTON_CHANGE = 1;
int cmd = CMD_WAIT;
int buttonState = HIGH;
static long startPress = 0;

Ticker ticker_piroff;
Ticker ticker_LDRmeas;
boolean gv_tickPIRmeas = true;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NeoCnt, NeoPIN, NEO_GRB + NEO_KHZ800);

const char *gc_hostname = "NghtLghtNeo";
//const char* mqtt_pubtopic_ldr   = "NghtLghtNeo/LDR/val";

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

void tickPIRmeas() {
  gv_tickPIRmeas = true;
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
  set_rgb(255, 0, 0);
  delay(500);
  colorWipe(strip.Color(0, 255, 0), 10); //
  set_rgb(0, 255, 0);
  delay(500);
  colorWipe(strip.Color(0, 0, 255), 10); //
  set_rgb(0, 0, 255);
  delay(500);
  colorWipe(strip.Color(0, 0, 0), 10); //

  set_rgb(255, 255, 255);

  wifi_init(gc_hostname);

  init_ota(gv_clientname);

  //init_mqtt(gv_clientname);

  set_rgb(0, 0, 0);

  strip.show(); // Initialize all pixels to 'off'

  //setup button
  pinMode(btnpin, INPUT);
  attachInterrupt(btnpin, IntBtn, CHANGE);

  //
  pinMode(pirpin, INPUT);
  attachInterrupt(pirpin, IntPIR, RISING);

  delay(500);

  ticker_LDRmeas.attach(1, tickPIRmeas);

}

void loop() {

  check_ota();

  //check_mqtt();

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

  if ( !gv_light_on && gv_tickPIRmeas) {

    // Lights off: measure LDR
    // so turn off all LEDs
    set_rgb(0, 0, 0);
    delay(200);

    // now measure LDR
    LDRValue = analogRead(LDRPin);


    //    char buffer[10];
    //    dtostrf(LDRValue, 0, 1, buffer);
    //    client.publish(mqtt_pubtopic_ldr, buffer, false);

    // show result of measurement
    if ( LDRValue < LDRThres ) {
      analogWrite(ledpingn, 200);
    }
    gv_tickPIRmeas = false;
  }

//  int pirState = digitalRead(pirpin);
//
//  if (pirState == 1) {
//    analogWrite(ledpinrt, 255);
//  } else {
//    analogWrite(ledpinrt, 0);
//  }

  if (gv_PIR_on) {
    analogWrite(ledpinrt, 50);
    // PIR trigged, is LDR OK?
    if ( LDRValue < LDRThres ) {
      // --> Turn On Light
      analogWrite(ledpinbl, 255);
      if (gv_neo_on == false ) {
        colorWipe(strip.Color(255, 0, 0), 10); // Red
        //digitalWrite(lightpin1, HIGH);
        //digitalWrite(lightpin2, HIGH);
        gv_neo_on = true;
      }
      gv_light_on = true;
    }

  } else {
    analogWrite(ledpinrt, 0);
    analogWrite(ledpinbl, 0);
    if (gv_neo_on == true ) {
      colorWipe(strip.Color(0, 0, 0), 10); // Off
      //digitalWrite(lightpin1, LOW);
      //digitalWrite(lightpin2, LOW);
      gv_neo_on = false;
    }
    gv_light_on = false;
  }

  delay(100);

}
