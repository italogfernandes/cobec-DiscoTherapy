#include "Timer.h"

#define PIN_EMG A0
#define baud 115200
#define samplePeriod 1

Timer t;
int valor_lido;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(baud);
  pinMode(PIN_EMG, INPUT);
  t.every(samplePeriod, takeReading);
}

void loop() {
  // put your main code here, to run repeatedly:]
   t.update();
}

void takeReading(){
  valor_lido = analogRead(PIN_EMG);
  Serial.println(valor_lido);
}

