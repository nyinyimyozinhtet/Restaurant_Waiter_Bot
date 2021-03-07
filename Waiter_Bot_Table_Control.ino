/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************

  Blynk using a LED widget on your phone!

  App project setup:
    LED widget on V1
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>


// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "R5gHPoTrl4mELc2Mtn0nQxgIXWaPh2xG";
char ssid[] = "NyiNyi";
char pass[] = "aaaaaaaa";
WidgetLED led1(V0);
WidgetLED led2(V1);
WidgetLED led3(V6);
WidgetLED led4(V7);
SoftwareSerial ESPserial(3, 1);
//BlynkTimer timer;


BLYNK_CONNECTED() {
  Blynk.syncVirtual(V2);
  Blynk.syncVirtual(V3);
  Blynk.syncVirtual(V4);
  Blynk.syncVirtual(V5);
}

BLYNK_WRITE(V2) {
  int buttonState = param.asInt();
  
  
  if (buttonState == 1) {
    led1.on();
       ESPserial.write("1");
       Serial.println("");
    
  } else {
    led1.off();
    Serial.println("LED off");
  }
}
BLYNK_WRITE(V3) {
  int buttonState = param.asInt();
  
  
  if (buttonState == 1) {
    led2.on();
       ESPserial.write("2");
       Serial.println("");
    
  } else {
    led2.off();
    Serial.println("LED off");
  }
}

BLYNK_WRITE(V4) {
  int buttonState = param.asInt();
  
  
  if (buttonState == 1) {
    led3.on();
       ESPserial.write("3");
       Serial.println("");
    
  } else {
    led3.off();
    Serial.println("LED off");
  }
}

BLYNK_WRITE(V5) {
  int buttonState = param.asInt();
  
  
  if (buttonState == 1) {
    led4.on();
       ESPserial.write("4");
       Serial.println("");
    
  } else {
    led4.off();
    Serial.println("LED off");
  }
}

// V1 LED Widget is blinking
//e
void setup()
{
  // Debug console
  Serial.begin(9600);
  ESPserial.begin(9600); 

  Blynk.begin(auth, ssid, pass);

  //  timer.setInterval(1000L, blinkLedWidget);
}

void loop()
{
  Blynk.run();
  
  //  timer.run();
}
