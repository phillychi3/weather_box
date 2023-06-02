#include <Arduino.h>

int ledpins[] = {13,12,14,27};
int rledpins[] = {27,14,12,13};



void setup() {
  for (auto &&i : ledpins)
  {
    pinMode(i, OUTPUT);
  }
}

void loop() {
  //流星燈

  for (int i = 0; i < 4; i++)
  {
    digitalWrite(rledpins[i], HIGH);
    delay(100);
  }
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(rledpins[i], LOW);
  }
}
