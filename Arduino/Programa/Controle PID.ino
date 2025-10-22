#include <Wire.h> // Library I2C - Biblioteca I2C
#include <SSD1306Ascii.h> //I2C OLED
#include <SSD1306AsciiWire.h> //I2C OLED

#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiWire oled;

#define ENCA 2 // Verde
#define ENCB 5 // Amarelo
#define IN2 6 // Entrada drive L298N
#define IN1 7 // Entrada drive L298N

int pot_proportional = A0; // Entrada potênciometro Proporcional
int pot_derivative = A2; // Entrada potênciometro Derivativo
int pot_integral = A1; // Entrada potênciometro Integral
int valuepot_P = 0; // Valor do potênciometro P
int valuepot_I = 0; // Valor do potênciometro I
int valuepot_D = 0; // Valor do potênciometro D
float pot_P; 
float pot_I; 
float pot_D;


float cv1,error1,error2;
float Kp;
float Ki;
float Kd;
float Tm = 0.1;




float controlSignal = 0; // Sinal de saída PV

const int PWMPin = 10;
int PWMValue = 0;
int motorDirection = 0; 

//Setpoint - Posição Alvo
float targetPosition = 0;

// Valores medidos
volatile float motorPosition = 0; // Posição encoder do motor
float previousMotorPosition = -1; 

long prevT = 0;
float eprev = 0;
float eintegral = 0;
float OLEDTimer = 0; // Intervalo de atualização do display

//Rotary encoder - Encoder rotativo
const int RotaryCLK = 3; //CLK pin (S2) - Pino CLK (S2)
const int RotaryDT = 9; //DT pin (S1) - Pino DT (S1)
const int RotarySW = 8; //SW pin (Key) - Pino SW (Key)
int RotaryButtonValue = 0; // Botão acionado ou não
float RotaryTime; // Tempo de debounce
volatile int rotaryValue = 0; // Valor manipulado pelo encoder
int previousRotaryValue = -1; // Valor prévio

// Status CLK e DT
int CLKNow;
int CLKPrevious;
int DTNow;
int DTPrevious;

// Variáveis PID
float previousTime = 0; //Cálculo delta t
float previousError = 0; // Cálculo do derivativo (edot)
float errorIntegral = 0; // Erro integral 
unsigned long currentTime = 0; // Tempo no momento calculado
float deltaTime = 0; // Diferença tempo
float error = 0; // Erro
float edot = 0; // Derivativo (de/dt)

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Inicializa I2C
  Wire.setClock(800000L); // Clock mais rápido

  pinMode(pot_proportional, INPUT);// Configura pot_proportional como entrada
  pinMode(pot_integral, INPUT);// Configura pot_integral como entrada
  pinMode(pot_derivative, INPUT);// Configura pot_derivative como entrada 

// Define os sinais de encoder como entrada
  pinMode(ENCA,INPUT); 
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), checkEncoder, RISING); // Faz a leitura toda a vez que o sinal do encoder A sobe chamando a função readEncoder
  pinMode(PWMPin,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

// Definição dos pinos do encoder rotativo
  pinMode(RotaryCLK, INPUT_PULLUP); //CLK
  pinMode(RotaryDT, INPUT_PULLUP); //DT
  pinMode(RotarySW, INPUT_PULLUP); //SW
  attachInterrupt(digitalPinToInterrupt(RotaryCLK), RotaryEncoder, CHANGE);
  // Guarda valores
  CLKPrevious = digitalRead(RotaryCLK);
  DTPrevious = digitalRead(RotaryDT);

#if RST_PIN >= 0
  oled.begin(&Adafruit128x32, I2C_ADDRESS, RST_PIN);
#else
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
#endif 

  oled.setFont(Adafruit5x7);
  oled.clear(); // Limpa display
  oled.set2X(); 
  oled.println("   PID    "); // Mensagem de entrada
  oled.println("Controller");
  oled.set1X();
  delay(1000);
  OLEDTimer = millis(); // Inicia o timer
  oled.clear();
  displayPermanentItems();
  refreshDisplay();

}

void loop() {

  CheckVariablesPID();

  calculatePID();

  driveMotor();

  CheckRotaryButton();

  refreshDisplay();

  printValues();

}

void checkEncoder(){
// Lê o encoder do motor
  int b = digitalRead(ENCB);
  if(b>0){
    motorPosition--;
  }
  else{
    motorPosition++;
  }
}

void driveMotor(){
// Drive motor
// Determina a velocidade e a direção baseada no valor do sinal de controle

  if (controlSignal < 0) // Valor negativo: CCW
  {
    motorDirection = -1;
  }
  else if (controlSignal > 0) // Positivo: CW
  {
    motorDirection = 1;
  }
  else // Para o motor
  {
    motorDirection = 0;
  }

 PWMValue = (int)fabs(controlSignal); 
  if (PWMValue > 255)
  {
    PWMValue = 255;
  }

  if (PWMValue < 30 && error != 0)
  {
    PWMValue = 30;
  }

// Define direção do motor
  if(motorDirection == 1){
    digitalWrite(IN2,HIGH);
    digitalWrite(IN1,LOW);
  }
  else if(motorDirection == -1){
    digitalWrite(IN2,LOW);
    digitalWrite(IN1,HIGH);
  }
  else if(error = 0){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    PWMValue = 0;
    digitalWrite(PWMPin,LOW);
    
  }
  else {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    PWMValue = 0;
    digitalWrite(PWMPin,LOW);
  } 
  
// Leitura Analógica do pino PWM
  analogWrite(PWMPin, PWMValue);
}

void calculatePID(){
// PID
  currentTime = millis(); //current time

  previousTime = currentTime; // Salva o tempo atual para comparação da próxima iteração

  error = targetPosition - motorPosition; // Posição atual - setpoint

  controlSignal = cv1 + (Kp + (Kd/Tm))*error + (-Kp + Ki*Tm - 2*Kd/Tm)*error1 + (Kd/Tm)*error2;
  cv1 = controlSignal;
  error2 = error1;
  error1 = error;
  
}

void printValues(){
  Serial.println(error);
  Serial.println(PWMValue);

 
}

void displayPermanentItems(){
// Printa os termos no display
  oled.setCursor(0, 0); //(x [pixels], y[lines])
  oled.print("Target");

  oled.setCursor(0, 2);
  oled.print("Position");
}

void refreshDisplay(){
// Display
  if (millis() - OLEDTimer > 100) 
    {
    if (previousRotaryValue != rotaryValue)
    {
      oled.setCursor(0, 1);
      oled.print("      ");
      oled.setCursor(0, 1);
      oled.print(rotaryValue); // printa o setpoint pelo o encoder rotativo

      previousRotaryValue = rotaryValue;
      OLEDTimer = millis(); //reset timer
    }

    if (motorPosition != previousMotorPosition)
    {
      oled.setCursor(0, 3);
      oled.print("      ");
      oled.setCursor(0, 3);
      oled.print(motorPosition, 0); //printa o novo valor absoluto de posição

      previousMotorPosition = motorPosition;
      OLEDTimer = millis(); //reset timer
    }
  }
  else
  {
    //skip
  }
}
void RotaryEncoder(){
// Target position
  CLKNow = digitalRead(RotaryCLK); // Estado do pino CLK
    
  if (CLKNow != CLKPrevious  && CLKNow == 1)
  {
    if (digitalRead(RotaryDT) != CLKNow) 
    {
      rotaryValue = rotaryValue - 56 ; 
    }
    else
    {
      rotaryValue = rotaryValue + 56 ; 
          }
  }
  CLKPrevious = CLKNow;  // Guarda o último estado CLK
}
void CheckRotaryButton(){
// Checa o posição alvo
  RotaryButtonValue = digitalRead(RotarySW); 

  if (RotaryButtonValue == 0) 
  {
    if (millis() - RotaryTime > 1000)
    {
      targetPosition = rotaryValue; 
      digitalWrite(PWMPin,HIGH);
      RotaryTime = millis(); 
    }
  }
}
void CheckVariablesPID(){
// Leitura dos potenciômetros
  valuepot_P = analogRead(pot_proportional);
  pot_P = map(valuepot_P, 0, 1023, 0, 20);//Função map() para converter a escala de 0 a 1023 para a escala de 0 a 1 com incremento de 0.05
  
  Kp = pot_P;

  valuepot_I = analogRead(pot_integral);
  pot_I = map(valuepot_I, 0, 1023, 0, 20)/20.0;//Função map() para converter a escala de 0 a 1023 para a escala de 0 a 1 com incremento de 0.05
  
  Ki = pot_I;

  valuepot_D = analogRead(pot_derivative);
  pot_D = map(valuepot_D, 0, 1023, 0, 20)/20.0;//Função map() para converter a escala de 0 a 1023 para a escala de 0 a 1 com incremento de 0.05
  
  Kd = pot_D;
}
