#include <Encoder.h>
#include "TimerOne.h"

#define NUMBERS_OF_MOTORS 1 //2 Moteurs

#define FIRST_MOTOR 0
#define SECOND_MOTOR 1

#define pwm_moteur_1 11   //Pin Arduino pour le pwm du premier moteur
#define fwd_moteur_1 8    //Pin Arduino pour le sens moteur avant
#define rev_moteur_1 7    //Pin Arduino pour le sens moteur arrière

#define pwm_moteur_2 3    //Pin Arduino pour le pwm du deuxième moteur
#define fwd_moteur_2 4    //Pin Arduino pour le sens moteur avant
#define rev_moteur_2 5    //Pin Arduino pour le sens moteur arrière

#define PI 3.1415926535897932384626433832795
#define LONG_MAX 2147483647

#define encodeur_A_moteur_1 9       //Pin Arduino pour la voie A de l'encodeur du premier moteur
#define encodeur_B_moteur_1 12      //Pin Arduino pour la voie B de l'encodeur du premier moteur

#define encodeur_A_moteur_2 2       //Pin Arduino pour la voie A de l'encodeur du deuxième moteur
#define encodeur_B_moteur_2 6       //Pin Arduino pour la voie B de l'encodeur du deuxième moteur

#define INDEX_MAX_POSITION  5       //Index maximum dans le tableau position (pas plus de 5 positions enregistrées)
#define INDEX_MAX_COMMAND   5       //Idem pour le tableau commande
#define INDEX_MAX_TARGET    5       //Idem pour le tableau target

#define PWM_MAX             255.    //Valeur maximale pouvant être donnée en pwm (correspond à la vitesse max)
#define COMMAND_MAX         12.     //Valeur maximale pouvant être donnée en tension (correspond à la vitesse max)

#define TRANSFER_TARGET     4.5129  //Coefficient précompensateur sur la consigne

long count_moteur_1;                //Compte les ticks des voies de l'encodeur du premier moteur                                       
long count_moteur_2;                //Compte les ticks des voies de l'encodeur du deuxième moteur    

double position_motors[NUMBERS_OF_MOTORS][INDEX_MAX_POSITION];          //Tableau référençant les différentes positions enregistrées des deux moteurs (0 = FIRST_MOTOR, 1 = SECOND_MOTOR)
double command_motors[NUMBERS_OF_MOTORS][INDEX_MAX_COMMAND];            //Tableau référençant les différentes commandes enregistrées des deux moteurs (0 = FIRST_MOTOR, 1 = SECOND_MOTOR)
double target_motors[NUMBERS_OF_MOTORS];                                //Tableau référençant les consignes des deux moteurs (0 = FIRST_MOTOR, 1 = SECOND_MOTOR)

double transfer_on_saturation[NUMBERS_OF_MOTORS][INDEX_MAX_COMMAND];    //Tableau référençant les différentes commandes après traitement par le polynome d'anti wind-up (0 = FIRST_MOTOR, 1 = SECOND_MOTOR)
double transfer_on_feedback[NUMBERS_OF_MOTORS][INDEX_MAX_COMMAND];      //Tableau référençant les différentes commandes après traitement par le polynome de feedback (0 = FIRST_MOTOR, 1 = SECOND_MOTOR)

Encoder encodeur_moteur_1(encodeur_A_moteur_1,encodeur_B_moteur_1);     //Encodeur du premier moteur (cf Encoder.h)
Encoder encodeur_moteur_2(encodeur_A_moteur_2,encodeur_B_moteur_2);     //Encodeur du deuxième moteur (cf Encoder.h)

//Définition des différentes valeurs du régulateurs
double sat_pol_num[INDEX_MAX_COMMAND] = {0,-0.0201,0.0515,-0.0440,0.0141};                      //Numérateur du polynome anti wind-up
double sat_pol_den[INDEX_MAX_COMMAND] = {1.0000,   -3.2493,    3.9729,   -2.1675,    0.4454};   //Dénominateur du polynome anti wind-up

double ret_pol_num[INDEX_MAX_POSITION] = {0,    1.6020,   -4.0425,    3.4071,   -0.9631};       //Numérateur du polynome feedback
double ret_pol_den[INDEX_MAX_POSITION] = {1.0000,   -3.2493,    3.9729,   -2.1675,    0.4454};  //Dénominateur du polynome feedback

//Prototype du controleur
void controller(double target[NUMBERS_OF_MOTORS],
                double command[NUMBERS_OF_MOTORS][INDEX_MAX_COMMAND],
                double position[NUMBERS_OF_MOTORS][INDEX_MAX_POSITION],
                int num_of_motor);
                
void setup() {
  //Définition des pins
  pinMode(pwm_moteur_1, OUTPUT);
  pinMode(fwd_moteur_1, OUTPUT);
  pinMode(rev_moteur_1, OUTPUT);

  pinMode(pwm_moteur_2, OUTPUT);
  pinMode(fwd_moteur_2, OUTPUT);
  pinMode(rev_moteur_2, OUTPUT);

  pinMode(10, OUTPUT);
  Timer1.initialize(5e5);             // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);   // attaches callback() as a timer overflow interrupt
  
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

  target_motors[0] = 1; //rad             //Consigne pour le premier moteur
  target_motors[1] = -target_motors[0];   //Consigne pour le deuxième moteur (déphasage de pi/2)
}

ISR(TIMER0_COMPA_vect){//Interrupt 1kHz
 count_moteur_1 = encodeur_moteur_1.read(); //Enregistre le nombre de ticks totaux
 if (count_moteur_1 < LONG_MAX && count_moteur_1 > -LONG_MAX){
  position_motors[FIRST_MOTOR][0] = 2*PI*(double)count_moteur_1/(65*2*2*12);
  //65 : Rapport de réduction
  //2 : 2 voies
  //2 : Front montant et descendant
  //12 : Pulse Per Revolution
  //Transforme les counts moteurs en angle utilisable
 }
 else{
  encodeur_moteur_1.write(0); //Si les counts sont au delà du max du type long on remet à zéro
 }
 //Idem pour le deuxième moteur
 count_moteur_2 = encodeur_moteur_2.read();
 if (count_moteur_2 < LONG_MAX && count_moteur_2 > -LONG_MAX){
  position_motors[SECOND_MOTOR][0] = 2*PI*(double)count_moteur_2/(65*2*2*12);
 }
 else{
  encodeur_moteur_2.write(0);
 }
 //Appel du régulateur pour les 2 moteurs
 controller(target_motors,command_motors,position_motors,0);
 controller(target_motors,command_motors,position_motors,1);
}

void callback()
{
  Serial.print("position_moteur_1: ");
  Serial.println(position_motors[FIRST_MOTOR][0]);
  Serial.print("position moteur 2: ");
  Serial.println(position_motors[SECOND_MOTOR][0]);
  target_motors[0] = -target_motors[0]; //Inversion de la commande toutes les secondes
  target_motors[1] = -target_motors[1]; //Inversion de la commande toutes les secondes
}

int commandToPwm(double command){
  return command/COMMAND_MAX*PWM_MAX; //Transforme la valeur de la commande en tension en pwm
}

void transferOnSaturation(double command[INDEX_MAX_COMMAND],double output[INDEX_MAX_COMMAND]){
  offsetCommand(command,output);  //Mise à jour des deux paramètres
  for (int index = 0; index<INDEX_MAX_COMMAND; index++){
    output[0] += command[index]*sat_pol_num[index] + output[index]*sat_pol_den[index]; //Utilisation de la transformée en z anti wind-up
  }
}

void transferOnFeedback(double y[INDEX_MAX_POSITION],double output[INDEX_MAX_POSITION]){
  offsetFeedBack(y,output); //Mise à jour des deux paramètres
  for (int index = 0; index<INDEX_MAX_POSITION; index++){
    output[0] += y[index]*ret_pol_num[index] + output[index]*ret_pol_den[index];  //Utilisation de la transformée en z feedback
  }
}

void saturation(double u){
  // Sature la commande si celle-ci dépasse le maximum possible
  if(u >= COMMAND_MAX){
      u = COMMAND_MAX;
    }
    if(u <= -COMMAND_MAX){
      u = -COMMAND_MAX;
    }
}

void writeOnPins(double u, int num_of_motor){
  if(num_of_motor == 0){
  //Permet d'écrire sur les pins du moteur 1 et de gérer le sens en fonction du signe de u
    if(u < 0){
      digitalWrite(fwd_moteur_1, LOW);
      digitalWrite(rev_moteur_1, HIGH);
      int pwmOutput = commandToPwm(abs(u));
      analogWrite(pwm_moteur_1,pwmOutput);
    }
    else{
      digitalWrite(fwd_moteur_1, HIGH);
      digitalWrite(rev_moteur_1, LOW);
      int pwmOutput = commandToPwm(abs(u));
      analogWrite(pwm_moteur_1,pwmOutput);
    }
  }
  else{
    //Permet d'écrire sur les pins du moteur 2 et de gérer le sens en fonction du signe de u
    if(u < 0){
      digitalWrite(fwd_moteur_2, LOW);
      digitalWrite(rev_moteur_2, HIGH);
      int pwmOutput = commandToPwm(abs(u));
      analogWrite(pwm_moteur_2,pwmOutput);
    }
    else{
      digitalWrite(fwd_moteur_2, HIGH);
      digitalWrite(rev_moteur_2, LOW);
      int pwmOutput = commandToPwm(abs(u));
      analogWrite(pwm_moteur_2,pwmOutput);
    }
  }
  
}

void controller(double target[NUMBERS_OF_MOTORS],
                double command[NUMBERS_OF_MOTORS][INDEX_MAX_COMMAND],
                double position[NUMBERS_OF_MOTORS][INDEX_MAX_POSITION],
                int num_of_motor)
{
  if(num_of_motor == 0){
    transferOnSaturation(command[num_of_motor],transfer_on_saturation[num_of_motor]);     //Met à jour la dernière valeur de transfer_on_saturation
    transferOnFeedback(position_motors[num_of_motor],transfer_on_feedback[num_of_motor]); //Met à jour la dernière valeur de transfer_on_feedback
    command[num_of_motor][0]=  -transfer_on_feedback[num_of_motor][0] + 
                  transfer_on_saturation[num_of_motor][0] +
                  TRANSFER_TARGET*target_motors[num_of_motor];                            //La dernière valeur de la commande est mise à jour

    saturation(command[num_of_motor][0]);                                                 //Saturation si nécessaire
    writeOnPinsFirstMotor(command[num_of_motor][0],num_of_motor);                         //Envoi au moteur
  }
  else{
    transferOnSaturation(command[num_of_motor],transfer_on_saturation[num_of_motor]);     //Met à jour la dernière valeur de transfer_on_saturation
    transferOnFeedback(position_motors[num_of_motor],transfer_on_feedback[num_of_motor]); //Met à jour la dernière valeur de transfer_on_feedback
    command[num_of_motor][0] =  -transfer_on_feedback[num_of_motor][0] + 
                  transfer_on_saturation[num_of_motor][0] +
                  TRANSFER_TARGET*target_motors[num_of_motor];                            //La dernière valeur de la commande est mise à jour
    saturation(command[num_of_motor][0]);                                                 //Saturation si nécessaire
    writeOnPins(command[num_of_motor][0],num_of_motor);                                   //Envoi au moteur           
  }
}

void offsetCommand(double command[INDEX_MAX_COMMAND],double output[INDEX_MAX_COMMAND]){
  for(int i=1; i<INDEX_MAX_COMMAND; i++){
        //Approximation retardée
        //Décalage des valeurs dans le tableau afin de mettre à jour
        command[i]               = command[i-1];
        output[i]                = output[i-1];  
    }
    command[0]  = 0;
    output[0]   = 0;
    //On met les deux dernières valeurs à zéros
    //Command sera mis à jour par le controleur
    //Output sera mis à jour par transferOnSaturation
}

void offsetFeedBack(double y[INDEX_MAX_COMMAND],double output[INDEX_MAX_COMMAND]){
  for(int i=1; i<INDEX_MAX_COMMAND; i++){
        //Approximation retardée
        //Décalage des valeurs dans le tableau afin de mettre à jour
        y[i]                      = y[i-1];
        output[i]                = output[i-1];  
    }
    y[0]  = 0;
    output[0]   = 0;
    //On met les deux dernières valeurs à zéros
    //y sera mis à jour par interruption
    //output sera mis à jour par transferOnFeedback 
}



void loop(){
};
