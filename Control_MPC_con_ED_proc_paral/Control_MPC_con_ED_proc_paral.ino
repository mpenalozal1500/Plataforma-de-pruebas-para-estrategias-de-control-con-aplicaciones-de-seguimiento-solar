#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

// Parametros de configuracion
#define Nh 10                                            // Horizonte de predicción
#define Nc 5                                             // Horizonte de control
double xmin[Nc] = {-1,-1,-1,-1,-1};        // Cotas inferiores
double xmax[Nc] = {1,1,1,1,1};                // Cotas superiores
double x0 = 73;                                           // Posición angular inicial (grados)
double delta = 0.1;                                        // Coeficiente de ponderacion del error de seguimiento
double lambda = 1;                                       // Coeficiente de ponderacion del consumo energético

int Ts = 1; // Tiempo de muestreo en segundos (enteros)
double h0 = 8.0; // Hora de inicio
double FC = 5.0; // Factor de contraccion para la trayectoria

#define NP 50    // Numero de vectores en cada generacion
int iter_max = 500; // Numero maximo de iteraciones para la optimizacion
double de_min = 0.00000001; // Cota de desviacion estandar relativa para criterio de paro
double F = 0.7;    // Factor de ponderacion [0, 2]
double CR = 0.3; // Coeficiente de cruza [0, 1]

///////////////////////////////////////

// Configuracion de hardware

#define PPR 20000.0

#define in1_motor 4
#define in2_motor 6
#define pwm_motor 5

#define EN_DIAG_motor 3

#define b_encoder 7
#define a_encoder 8

/////////////////////////////////////////

// Variables del controlador

#define x_s 2 // Dimensión del vector de estado

int t0 = 0; // Instante de inicio de las ventanas de control
int k; // Contador del instante de muestreo en las ventanas de control

#define pi 3.1415926535

double Y[Nh] = {0.0};  // Vector de predicción de salida
double r[Nh] = {0.0};  // Vector de trayectoria de referencia
double U[Nc] = {0.0};    // Vector de secuencia de control
double x[x_s] = {0.0}; // Vector de estados

int c = 0; // Bandera para calculo de secuencia de control
int arranque = 0; // Bandera para ciclo de arranque

///////////////////////////////////////

// Variables del optimizador

#define n Nc      // Elementos por vector (Igual al horizonte de control)
double X[NP][n];  // Matriz de generacion
double V[NP][n];  // Matriz de vectores mutados
double Ut[NP][n]; // Matriz de cruza
double Fss[NP]; // Vector de Fitness para criterio de paro
double prom; // Promedio
double de; // Desviación estándar
double der; // Desviación estándar relativa
///////////////////////////////////////

void lectura_vector_x()
{
    int16_t posicion = (int16_t)TIM1->CNT;
    x[0] = ((posicion/PPR)*2.0*pi + x0*(pi/180.0))*6900;
    x[1] = 0;
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

void gen_tray()
{
    double tray[Nh];
    double t;
    for(int i =0;i<Nh;i++){
      t = h0 + (((t0/1000.0) + (i+1)*Ts)/3600.0)*FC;
      tray[i] = trayectoria_solar(t,1);
    }
    memcpy(r, tray, sizeof(tray));
}

void predictor()
{
    double xk[x_s];
    double xk1[x_s];
    xk[0] = x[0];
    xk[1] = x[1];
    double a11 = 1;
    double a12 = 0.132545499810172;
    double a21 = 0;
    double a22 = 5.310936621518021e-04;
    double b1 = 1.593376854928286e2;
    double b2 = 1.835866460120635e2;
    double c1 = 1.0/6900.0;
    double c2 = 0;

    for (int j = 0; j < Nh; j++)
    {
        if (j < Nc)
        {
            if(fabs(U[j])>0.3){
            xk1[0] = xk[0]*a11 + a12*xk[1] + b1*U[j]; // Dinámica del primer estado
            xk1[1] = xk[0]*a21 + xk[1]*a22 + b2*U[j]; // Dinámica del segundo estado
            }
            else{
                xk1[0] = xk[0]*a11 + a12*xk[1]; // Dinámica del primer estado
                xk1[1] = xk[0]*a21 + xk[1]*a22; // Dinámica del segundo estado
            }

        }
        else
        {
            xk1[0] = xk[0]*a11 + a12*xk[1]; // Dinámica del primer estado
            xk1[1] = xk[0]*a21 + xk[1]*a22; // Dinámica del segundo estado
        }
        Y[j] = xk1[0]*c1 + xk1[1]*c2;  // Cálculo de la salida
        xk[0] = xk1[0]; // Actualización del primer estado
        xk[1] = xk1[1]; // Actualización del segundo estado
    }
}

void inicializacion()
{
    // Inicialización aleatoria de la matriz de generacion

    for (int i = 0; i < NP; i++)
    {
        for (int j = 0; j < n; j++)
        {
            double al = (rand() % (10000 + 1) / 10000.0); // Numero aleatorio entre 0 y 1
            X[i][j] = xmin[j] + (xmax[j] - xmin[j]) * al;
        }
    }
}

void mutacion()
{
    for (int i = 0; i < NP; i++)
    {
        int r1, r2, r3;

        // Primer indice aleatorio
        do
        {
            r1 = rand() % NP;
        } while (r1 == i);

        // Segundo indice aleatorio
        do
        {
            r2 = rand() % NP;
        } while (r2 == i || r2 == r1);

        // tercer indice aleatorio
        do
        {
            r3 = rand() % NP;
        } while (r3 == i || r3 == r1 || r3 == r2);

        double v[n];

        for (int s = 0; s < n; s++)
        {
            v[s] = X[r1][s] + F * (X[r2][s] - X[r3][s]);
        }

        memcpy(V[i], v, n * sizeof(double));
    }
}

void cruza()
{
    for (int i = 0; i < NP; i++)
    {
        int random_j = rand() % n; //

        for (int j = 0; j < n; j++)
        {
            double random_CR = (rand() % (10000 + 1) / 10000.0); // Número aleatorio entre 0 y 1
            if (random_CR <= CR || j == random_j)
            {
                Ut[i][j] = V[i][j];
            }
            else
            {
                Ut[i][j] = X[i][j];
            }
        }
    }
}

double eval_restricciones(double xt[n])
{
    memcpy(U, xt, n * sizeof(double));
    predictor();
#define g 2 // Numero de restricciones de desigualdad
    double g1 = 0;
    double g2 = 0;
    for (int i = 0; i < Nc; i++)
    {
        if ((xt[i] - xmax[i]) > 0)
        {
            g1 = g1 + xt[i] - xmax[i];
        }
        if ((xmin[i] - xt[i]) > 0)
        {
            g1 = g1 + xmin[i] - xt[i];
        }
        if (fabs(xt[i]) > 0.1 && fabs(xt[i]) < 0.3){
            g1 = g1 + fabs(xt[i]);
        }
    }
   for (int i = 0; i < Nh; i++)
    {
        if (fabs(Y[i] - r[i]) > (0.5*(pi/180.0)))
        {
            g2 = g2 + fabs(Y[i]-r[i]);
        }
    }
    double c[g] = {g1,g2};

#define h 2 // Numero de restricciones de igualdad
    double h1 = 0;
    double h2 = 0;

    double ceq[h] = {h1, h2};

    double viol_g = 0.0;
    for (int i = 0; i < g; i++)
    {
        if (c[i] > 0)
        {
            viol_g = viol_g + c[i];
        }
    }

    double epsilon = 0.0001; // Tolerancia para restricciones de igualdad
    double viol_h = 0.0;
    for (int i = 0; i < h; i++)
    {
        if (fabs(ceq[i]) - epsilon > 0)
        {
            viol_h = viol_h + fabs(ceq[i]) - epsilon;
        }
    }
    return viol_g + viol_h;
}

double funcion_costo(double xt[n])
{
    memcpy(U, xt, n * sizeof(double));
    predictor();

    double et = 0; // Error de seguimiento (tracking)
    for (int i = 0; i < Nh; i++)
    {
        et = et + (r[i] - Y[i]) * (r[i] - Y[i]);
    }

    double ce = 0; // Consumo de energia
    for (int i = 0; i < Nc; i++)
    {
        ce = ce + U[i] * U[i];
    }

    double f = delta * et + lambda * ce;
    return f;
}

void seleccion()
{
    for (int i = 0; i < NP; i++)
    {
        int ganador;

        double x1[n], x2[n], x_ganador[n];
        memcpy(x1, X[i], n * sizeof(double));
        memcpy(x2, Ut[i], n * sizeof(double));
        double V1 = eval_restricciones(x1);
        double V2 = eval_restricciones(x2);
        double f1 = 1e9;
        double f2 = 1e9;

        if (V1 == 0)
        {
            f1 = funcion_costo(x1);
        }
        if (V2 == 0)
        {
            f2 = funcion_costo(x2);
        }

        if (V1 == 0 && V2 == 0)
        {
            if (f1 < f2)
            {
                ganador = 1;
            }
            else
            {
                ganador = 2;
            }
        }
        else
        {
            if (V1 == 0)
            {
                ganador = 1;
            }
            else
            {
                if (V2 == 0)
                {
                    ganador = 2;
                }
                else
                {
                    if (V1 < V2)
                    {
                        ganador = 1;
                    }
                    else
                    {
                        ganador = 2;
                    }
                }
            }
        }
        if (ganador == 2)
        {
            memcpy(X[i], Ut[i], n * sizeof(double));
        }
        memcpy(x_ganador, X[i], n * sizeof(double));
        Fss[n] = funcion_costo(x_ganador); 
    }
}

double promedio(){
    double prom = 0;
    for(int i = 0;i<NP;i++){
        prom = prom + Fss[i];
    }
    prom = prom/NP;
    return prom;
}

double desviacion_estandar(){
    double de = 0;
    for(int i = 0;i<NP;i++){
        de = de + (Fss[i]-prom)*(Fss[i]-prom);
    }
    de = sqrt(de/NP);
    return de;
}

void optimizador()
{
    inicializacion();
    for (int i = 0; i < iter_max; i++)
    {
        mutacion();
        cruza();
        seleccion();
        prom = promedio();
        //Serial.print("Promedio = "); Serial.println(prom,10);
        de = desviacion_estandar();
        //Serial.print("Desviacion estandar = "); Serial.println(de,10);
        der = de/(fabs(prom)+(1e-12));
        //Serial.print(de,10); Serial.print(" | "); Serial.println(i);
        if(de<de_min){
            break;
        }
    }
    memcpy(U, X[0], n * sizeof(double));
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

void preparacion_arranque(){
  lectura_vector_x();
  t0 = 0;
  gen_tray();
  optimizador();
  
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

    c = 0;
    arranque = 0;

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

    // Primer cálculo de trayectoria

}

void loop() {
    double voltaje1;
    double voltaje2;
    if(arranque == 0){
    preparacion_arranque();
    arranque = 1;
    }
    k = ((millis()-t0) / (Ts*1000));
    if(k >= Nh){
        t0 = millis();
        k = 0;
        c = 0;
    }
    double u;
    if(k < Nc){
        u = U[k];
    }
    else{
    u = 0;
    }
    if(fabs(u) < 0.3){
        u = 0;
    }
    control(u);
    if(k == Nc && c == 0){
        lectura_vector_x();
        gen_tray();
        Serial.println("0,0,0,0,0");
        optimizador();
        c = 1;
    }
       /* predictor();
        Serial.print("Y = [");
        for(int i = 0; i<Nh;i++){
            Serial.print(Y[i]);
            Serial.print(" ");
        }
        Serial.println("]");
        Serial.print("r = [");
        for(int i = 0; i<Nh;i++){
            Serial.print(r[i]);
            Serial.print(" ");
        }
        Serial.println("]");
        Serial.print("U = [");
        for(int i = 0; i<Nc;i++){
            Serial.print(U[i],6);
            Serial.print(" ");
        }
        Serial.println("]");
         */
         
        int16_t posicion = (int16_t)TIM1->CNT;
        double theta = (posicion/PPR)*2.0*pi + x0*(pi/180);
        /*
        Serial.print("Referencia: "); Serial.print(r[k],4); Serial.print(" rad | ");
        Serial.print("Posicion: "); Serial.print(theta,4); Serial.print(" rad | ");
        Serial.print("Control: "); Serial.print(u,4); Serial.print(" | ");
        Serial.print("Error: "); Serial.println((r[k]-theta)*(180/pi));
        delay(Ts*1000);
        */
        
        
        if(k<5){
            voltaje1 = analogRead(A1);
            voltaje2 = analogRead(A2);
        Serial.print(r[k],4); Serial.print(",");
        Serial.print(theta,4); Serial.print(",");
        Serial.print(u,4); Serial.print(",");
        Serial.print(voltaje1,4); Serial.print(",");
        Serial.println(voltaje2,4);
        delay(500);
        }
                
}
