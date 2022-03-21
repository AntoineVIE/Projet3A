/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01

    by Dejan Nedelkovski, www.HowToMechatronics.com
*/

#define enA 9
#define fwd 8
#define rev 7

int pwmOutput = 0;
int pwmOutputReverse = 255;
int sens = 1;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(fwd, OUTPUT);
  pinMode(rev, OUTPUT);
  Serial.begin(9600);
  // Set initial rotation direction
  digitalWrite(fwd, HIGH);
  digitalWrite(rev, LOW);
}

void loop() {
  /*if(pwmOutput < 255){
    analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
    digitalWrite(fwd, HIGH);
    digitalWrite(rev, LOW);
    pwmOutput += 1;
    delay(20);
  }
  else{
    analogWrite(enA,pwmOutputReverse);
    digitalWrite(fwd,LOW);
    digitalWrite(rev,HIGH);
    pwmOutputReverse -= 1;
    delay(20);   
  }*/
  if(sens == 1){
    digitalWrite(fwd, HIGH);
    digitalWrite(rev, LOW);
    analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
    sens = 0;
    delay(500);
    Serial.println("help");
    }
  else{
    digitalWrite(fwd, LOW);
    digitalWrite(rev, HIGH);
    analogWrite(enA, pwmOutput);
    sens = 1;
    delay(500);
  }
}
