#include <Encoder.h>
#include "TimerOne.h"

#define pwm_moteur_1 11
#define fwd_moteur_1 8
#define rev_moteur_1 7

#define pwm_moteur_2 3
#define fwd_moteur_2 4
#define rev_moteur_2 5

#define PI 3.1415926535897932384626433832795
#define LONG_MAX 2147483647

#define encodeur_A_moteur_1 9
#define encodeur_B_moteur_1 12

#define encodeur_A_moteur_2 2
#define encodeur_B_moteur_2 6

int pwmOutput = 200;
int pwmOutputreverse = 255;
int sens = 1;

long count_moteur_1;
double position_moteur_1;

long count_moteur_2;
double position_moteur_2;

Encoder encodeur_moteur_1(encodeur_A_moteur_1,encodeur_B_moteur_1);
Encoder encodeur_moteur_2(encodeur_A_moteur_2,encodeur_B_moteur_2);

void setup() {
  pinMode(pwm_moteur_1, OUTPUT);
  pinMode(fwd_moteur_1, OUTPUT);
  pinMode(rev_moteur_1, OUTPUT);

  pinMode(pwm_moteur_2, OUTPUT);
  pinMode(fwd_moteur_2, OUTPUT);
  pinMode(rev_moteur_2, OUTPUT);

  pinMode(10, OUTPUT);
  Timer1.initialize(2.5e5);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  
  Serial.begin(115200);
  cli();
  //set timer0 interrupt at 1kHz
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 249;// = (16*10^6) / (1000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // pwm_moteur_1ble timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei();  
}

ISR(TIMER0_COMPA_vect){//Interrupt 1kHz
 count_moteur_1 = encodeur_moteur_1.read();
 if (count_moteur_1 < LONG_MAX && count_moteur_1 > -LONG_MAX){
  position_moteur_1 = 360*(double)count_moteur_1/(65*2*2*12);
 }
 else{
  encodeur_moteur_1.write(0);
 }

 count_moteur_2 = encodeur_moteur_2.read();
 if (count_moteur_2 < LONG_MAX && count_moteur_2 > -LONG_MAX){
  position_moteur_2 = 360*(double)count_moteur_2/(65*2*2*12);
 }
 else{
  encodeur_moteur_2.write(0);
 }
 
 analogWrite(pwm_moteur_1,pwmOutput);
 analogWrite(pwm_moteur_2,pwmOutput);
}

void callback()
{
  Serial.print("position_moteur_1: ");
  Serial.println(position_moteur_1);
  Serial.print("position moteur 2: ");
  Serial.println(position_moteur_2);
  if(sens ==1){
    digitalWrite(fwd_moteur_1, HIGH);
    digitalWrite(rev_moteur_1, LOW);

    digitalWrite(fwd_moteur_2, LOW);
    digitalWrite(rev_moteur_2, HIGH);
    sens = 0;
  }
  else{
    digitalWrite(fwd_moteur_1, LOW);
    digitalWrite(rev_moteur_1, HIGH);

    digitalWrite(fwd_moteur_2, HIGH);
    digitalWrite(rev_moteur_2, LOW);
    sens = 1;
  }
  digitalWrite(10, digitalRead(10) ^ 1);  
}

void loop() {
  
}
