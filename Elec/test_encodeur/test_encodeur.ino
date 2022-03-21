/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01

    by Dejan Nedelkovski, www.HowToMechatronics.com
*/
#include <Encoder.h>

#define pwm_moteur_1 9
#define fwd_moteur_1 8
#define rev_moteur_1 7

#define encodeur_A_moteur_1 10
#define encodeur_B_moteur_1 11

int pwmOutput = 255;
int pwmOutputrev_moteur_1erse = 255;
int sens = 1;

int count_moteur_1;
int position_moteur_1;

Encoder encodeur_moteur_1(encodeur_A_moteur_1,encodeur_B_moteur_1);

void setup() {
  pinMode(pwm_moteur_1, OUTPUT);
  pinMode(fwd_moteur_1, OUTPUT);
  pinMode(rev_moteur_1, OUTPUT);
  
  Serial.begin(9600);
  cli();
  //set timer0 interrupt at 2kHz
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // pwm_moteur_1ble timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei();  
}

ISR(TIMER0_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
//generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave- toggle high then toggle low)
 count_moteur_1 = encodeur_moteur_1.read();
 //position_moteur_1 = ((count_moteur_1 * 6) % 360)/(49.6) ;
 position_moteur_1 = 2.*PI*((float)count_moteur_1)/(48.6*6);
 position_moteur_1 = modulo(position_moteur_1,2*PI);
 Serial.println(position_moteur_1);
}
void loop() {
  if(sens == 1){
    digitalWrite(fwd_moteur_1, HIGH);
    digitalWrite(rev_moteur_1, LOW);
    analogWrite(pwm_moteur_1, pwmOutput); // Send PWM signal to L298N pwm_moteur_1ble pin
    sens = 0;
    Serial.println("b");
     Serial.println(position_moteur_1);
    delay(500);
    }
  else{
    digitalWrite(fwd_moteur_1, LOW);
    digitalWrite(rev_moteur_1, HIGH);
    analogWrite(pwm_moteur_1, pwmOutput);
    sens = 1;
    Serial.println("a");
    delay(500);
  }
}

double modulo(double a,double b){
  int c = (int)a/b;
  int res = a - c*b;
  if (res < 0) {
    res = res + b;
  }
  return res;
}
