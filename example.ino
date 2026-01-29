#include <M5Stack.h>

int i = 0;

void setup(){
  M5.begin();
  M5.Power.begin();
}

void loop() {
  i++;
  M5.Lcd.drawCentreString(String(i), 160, 80, 8);
  delay(1000);
}
