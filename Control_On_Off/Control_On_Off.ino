#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

// Parametros de configuracion
double theta0 = 25; // Posición angular inicial (grados), 25° --> Elevacion, 73° --> Acimutal
double Ts = 1; // Tiempo de muestreo en segundos 
double h0 = 8.0; // Hora de inicio
double FC = 5.0; // Factor de contraccion para la trayectoria solar
int axis = 0; // Eje del SS, 0 --> Elevacion, 1 --> Acimutal 

// Parametros de configuracion del controlador On-Off
double umbral = 0.5; // Umbral de activacion en grados
double U = 0.3; // Magnitud de control en estado "On"

#define pi 3.1415926535

/////////////////////////////////////////

// Variables del controlador

int t0 = 0; // Instante de inicio de los ciclos de operación
double t; // Tiempo actual
double theta; // Posicion angular del eje en grados
double ref; // Referencia de posicion angular del eje en grados
double u; // Variable de control
double error; // Error de seguimiento de trayectoria
///////////////////////////////////////

// Configuracion de hardware

#define PPR 20000.0
#define in1_motor 4
#define in2_motor 6
#define pwm_motor 5
#define EN_DIAG_motor 3
#define b_encoder 7
#define a_encoder 8

///////////////////////////////////////

void lectura_encoder()
{
    int16_t posicion = (int16_t)TIM1->CNT;
    theta = (posicion/PPR)*360 + theta0;
}

double trayectoria_solar(double t, int axis){
double N,lambda,L,delta,tau,sn,sinh1,h,cosz1,z;
N = 93; // Numero de día;
lambda = 40.48*(pi/180.0); // Latitud
L = -3.68*(pi/180.0); // Longitud
sn = 12; // Medio día solar
delta=(23.45*(pi/180))*sin(2*pi*((284+N)/365)); // Declinación solar
tau=(sn-t)*(15*(pi/180)); // Angulo horario 

sinh1 =(cos(lambda)*cos(delta)*cos(tau))+(sin(lambda)*sin(delta));
h=asin(sinh1);

cosz1=(sin(h)*sin(lambda)-sin(delta))/(cos(h)*cos(lambda));
z=acos(cosz1);

if (t > sn){
z=-z;
}

if(axis == 0){
  return h; // Eje de elevación
}
if(axis == 1){
  return z; // Eje acimutal
}

}

void control(double u){
  u = u*255.0;
  if (u > 255){
        u = 255;
      }
      if (u < -255){
        u = -255;
      }
  if(u < 0){
    digitalWrite(in1_motor,HIGH);
    digitalWrite(in2_motor,LOW); 
    }
    else{
    digitalWrite(in1_motor,LOW);
    digitalWrite(in2_motor,HIGH); 
    }
    analogWrite(pwm_motor,abs((int)u));
}

/////////////////////////////////////////////

void setup() {
  Serial.begin(115200);

    // Configurar D7 (A8) y D8 (A9) como entradas
    pinMode(a_encoder, INPUT);
    pinMode(b_encoder, INPUT);
    pinMode(in1_motor, OUTPUT);
    pinMode(pwm_motor, OUTPUT);
    pinMode(in2_motor, OUTPUT);
    pinMode(EN_DIAG_motor, OUTPUT);
    pinMode(A5, INPUT);
    digitalWrite(EN_DIAG_motor, HIGH);

    // Habilitar el reloj para TIM1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  

    // Configurar los pines D7 y D8 (A8 y A9) en AF1 (TIM1)
    GPIOA->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);  // Limpiar bits
    GPIOA->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);  // AF mode
    GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL8_Pos) | (1 << GPIO_AFRH_AFSEL9_Pos);  // AF1 para TIM1

    // Configurar TIM1 en modo encoder
    TIM1->CR1 = 0;  // Resetear configuración
    TIM1->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;  // Modo encoder (ambos flancos)
    TIM1->CCMR1 = (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);  // CH1 y CH2 como entradas
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;  // Habilitar captura en ambos canales
    TIM1->PSC = 0;  // Sin preescalador (conteo en cada pulso)
    TIM1->ARR = 0xFFFF;  // Máximo conteo
    TIM1->CR1 |= TIM_CR1_CEN;  // Habilitar contador

}

void loop(){
    double voltaje1 = 0;
    double voltaje2 = 0;
    if(millis()-t0 >= Ts*1000){
        t0 = millis(); // Se registra el tiempo actual y se usa como referencia de inicio para el ciclo de operacion
        t = h0 + ((t0/1000.0)/3600.0)*FC; // Se calcula el tiempo actual en horas
        lectura_encoder(); // La funcion actualiza el valor de la variable theta en grados
        ref = (180.0/pi)*trayectoria_solar(t, axis); // Se obtiene la componente del vector solar en el eje correspondiente en grados
        error = ref-theta;
        
        // Calculo de accion de control
        
        u=0;
        if(error < -umbral){
            u = -U;
        }
        if(error > umbral){
            u = U;
        }
        
        Serial.print(ref,4); Serial.print(",");
        Serial.print(theta,4); Serial.print(",");
        Serial.print(u,4); Serial.print(",");
        Serial.print(error,4); Serial.print(",");
        Serial.println(voltaje2,4);
        
    }
    /*
    if(Serial.available()){
        u = Serial.parseFloat();
    }
    */
    control(u);
}
                
