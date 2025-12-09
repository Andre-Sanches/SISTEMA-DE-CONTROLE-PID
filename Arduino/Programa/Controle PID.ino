#include <Wire.h> // Biblioteca I2C
#include <SSD1306Ascii.h> //I2C OLED
#include <SSD1306AsciiWire.h> //I2C OLED

#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiWire oled;

#define ENCA 2 // Verde
#define ENCB 5 // Amarelo
#define IN2 6 // Entrada drive L298N
#define IN1 7 // Entrada drive L298N

int pot_proporcional = A0; // Entrada potênciometro Proporcional
int pot_derivativo = A2; // Entrada potênciometro Derivativo
int pot_integral = A1; // Entrada potênciometro Integral
int valorpot_P = 0; // Valor do potênciometro P
int valorpot_I = 0; // Valor do potênciometro I
int valorpot_D = 0; // Valor do potênciometro D
float pot_P; 
float pot_I; 
float pot_D;

//PID tuning - Parametrização PID
float proporcional; // Proporcional (Kp)
float derivativo ; // Derivativo (Kd)
float integral; // Integral (Ki)
float SinalControle = 0; // Sinal de saída PV

const int PWMPino = 10;
int PWMValor = 0;
int DirecaoMotor = 0; 

//Setpoint - Posição Alvo
float PosicaoAlvo = 0;

// Valores medidos
volatile float PosicaoMotor = 0; // Posição encoder do motor
float PosicaoAnteriorMotor = -1; 

long prevT = 0;
float eprev = 0;
float eintegral = 0;
float OLEDTimer = 0; // Intervalo de atualização do display

//Rotary encoder - Encoder rotativo
const int RotaryCLK = 3; //CLK pin (S2) - Pino CLK (S2)
const int RotaryDT = 9; //DT pin (S1) - Pino DT (S1)
const int RotarySW = 8; //SW pin (Key) - Pino SW (Key)
int BotaoRotativoValor = 0; // Botão acionado ou não
float RotaryTime; // Tempo de debounce
volatile int ValorRotativo = 0; // Valor manipulado pelo encoder
int ValorAnteriorRotativo = -1; // Valor prévio

// Status CLK e DT
int CLKAgora;
int CLKAnterior;
int DTNow;
int DTPrevious;

// Variáveis PID
float previousTime = 0; //Cálculo delta t
float ErroAnterior = 0; // Cálculo do derivativo (edot)
float errorIntegral = 0; // Erro integral 
float currentTime = 0; // Tempo no momento calculado
float deltaTime = 0; // Diferença tempo
float ValorErro = 0; // Erro
float edot = 0; // Derivativo (de/dt)

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Inicializa I2C
  Wire.setClock(800000L); // Clock mais rápido

  pinMode(pot_proporcional, INPUT);// Configura pot_proportional como entrada
  pinMode(pot_integral, INPUT);// Configura pot_integral como entrada
  pinMode(pot_derivativo, INPUT);// Configura pot_derivative como entrada 

// Define os sinais de encoder como entrada
  pinMode(ENCA,INPUT); 
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), LeituraEncoder, RISING); // Faz a leitura toda a vez que o sinal do encoder A sobe chamando a função readEncoder
  pinMode(PWMPino,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

// Definição dos pinos do encoder rotativo
  pinMode(RotaryCLK, INPUT_PULLUP); //CLK
  pinMode(RotaryDT, INPUT_PULLUP); //DT
  pinMode(RotarySW, INPUT_PULLUP); //SW
  attachInterrupt(digitalPinToInterrupt(RotaryCLK), EncoderRotativo, CHANGE);
  // Guarda valores
  CLKAnterior = digitalRead(RotaryCLK);
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
  Valoresfixosdisplay();
  AtualizaDisplay();

}

void loop() {

  VerificaValoresPID();

  CalculoPID();

  DriveMotor();

  EncoderRotativoBotao();

  AtualizaDisplay();

  Valores();

}

void LeituraEncoder(){
// Lê o encoder do motor
  int b = digitalRead(ENCB);
  if(b>0){
    PosicaoMotor++;
  }
  else{
    PosicaoMotor--;
  }
}

void DriveMotor(){
// Drive motor
// Determina a velocidade e a direção baseada no valor do sinal de controle

  if (SinalControle < 0) // Valor negativo: CCW
  {
    DirecaoMotor = -1;
  }
  else if (SinalControle > 0) // Positivo: CW
  {
    DirecaoMotor = 1;
  }
  else // Para o motor
  {
    DirecaoMotor = 0;
  }

 PWMValor = (int)fabs(SinalControle); 
  if (PWMValor > 255)
  {
    PWMValor = 255;
  }

  if (PWMValor < 30 && ValorErro != 0)
  {
    PWMValor = 30;
  }

// Define direção do motor
  if(DirecaoMotor == 1){
    digitalWrite(IN2,HIGH);
    digitalWrite(IN1,LOW);
  }
  else if(DirecaoMotor == -1){
    digitalWrite(IN2,LOW);
    digitalWrite(IN1,HIGH);
  }
  else{
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    PWMValor = 0;
  }  
// Leitura Analógica do pino PWM
  analogWrite(PWMPino, PWMValor);
}

void CalculoPID(){
// PID
  currentTime = micros(); //current time
  deltaTime = (currentTime - previousTime) / 1000000.0; // Diferença em segundos
  previousTime = currentTime; // Salva o tempo atual para comparação da próxima iteração

  ValorErro = PosicaoMotor - PosicaoAlvo; // Posição atual - setpoint

  edot = (ValorErro - ErroAnterior) / deltaTime; //edot = de/dt - termo derivativo

  errorIntegral = errorIntegral + (ValorErro * deltaTime); //termo integral

  SinalControle = (proporcional * ValorErro) + (derivativo * edot) + (integral * errorIntegral); // Soma final das três variaveis

  ErroAnterior = ValorErro; // Salva o valor para a próxima iteração para obter a diferença
}

void Valores(){
  Serial.print(ValorErro);
  Serial.print(" ");
  Serial.print(PosicaoAlvo);
  Serial.print(" ");
  Serial.print(PosicaoMotor);
  Serial.println(" ");
}

void Valoresfixosdisplay(){
// Printa os termos no display
  oled.setCursor(0, 0); //(x [pixels], y[lines])
  oled.print("Setpoint:");

  oled.setCursor(0, 1);
  oled.print("Posicao:");

  oled.setCursor(0, 2);
  oled.print("Kp:");

  oled.setCursor(55, 2);
  oled.print("Ki:");

  oled.setCursor(105, 2);
  oled.print("Kd:");
}

void AtualizaDisplay(){
// Display
  if (millis() - OLEDTimer > 100) 
    {
    if (ValorAnteriorRotativo != ValorRotativo)
    {
      oled.setCursor(60, 0);
      oled.print("      ");
      oled.setCursor(60, 0);
      oled.print(ValorRotativo); // printa o setpoint pelo o encoder rotativo

      ValorAnteriorRotativo = ValorRotativo;
      OLEDTimer = millis(); //Reset o timer do display
    }

    if (PosicaoMotor != PosicaoAnteriorMotor)
    {
      oled.setCursor(60, 1);
      oled.print("      ");
      oled.setCursor(60, 1);
      oled.print(PosicaoMotor, 0); //printa valor de posição atual do motor

      PosicaoAnteriorMotor = PosicaoMotor;
      OLEDTimer = millis(); //Reset o timer do display
    }
    if (millis() - OLEDTimer > 100) // Printar valores do controlador no display
    {
      oled.setCursor(0, 3);
      oled.print("      ");
      oled.setCursor(0, 3);
      oled.print(proporcional, 2); //printa o valor de Kp

      oled.setCursor(50, 3);
      oled.print("      ");
      oled.setCursor(50, 3);
      oled.print(integral, 2); //printa o valor de Ki

      oled.setCursor(100, 3);
      oled.print("      ");
      oled.setCursor(100, 3);
      oled.print(derivativo, 2); //printa o valor de Kd

      OLEDTimer = millis(); //Reset o timer do display
    }
    
  }
  
}
void EncoderRotativo(){
// Target position
  CLKAgora = digitalRead(RotaryCLK); // Estado do pino CLK
    
  if (CLKAgora != CLKAnterior  && CLKAgora == 1)
  {
    if (digitalRead(RotaryDT) != CLKAgora) 
    {
      ValorRotativo = ValorRotativo - 50 ; 
    }
    else
    {
      ValorRotativo = ValorRotativo + 50 ; 
          }
  }
  CLKAnterior = CLKAgora;  // Guarda o último estado CLK
}
void EncoderRotativoBotao(){
// Checa o posição alvo
  BotaoRotativoValor = digitalRead(RotarySW); 

  if (BotaoRotativoValor == 0) 
  {
    if (millis() - RotaryTime > 1000)
    {
      PosicaoAlvo = ValorRotativo; 
      digitalWrite(PWMPino,HIGH);
      RotaryTime = millis(); 
    }
  }
}
void VerificaValoresPID(){
// Leitura dos potenciômetros
  valorpot_P = analogRead(pot_proporcional);
  pot_P = map(valorpot_P, 0, 1023, 0, 20)/4.0;//Função map() para converter a escala de 0 a 1023 para a escala de 0 a 5 com incremento de 0.25
  
  proporcional = pot_P;

  valorpot_I = analogRead(pot_integral); 
  pot_I = map(valorpot_I, 0, 1023, 0, 20)/10.0;//Função map() para converter a escala de 0 a 1023 para a escala de 0 a 2 com incremento de 0.10
  
  integral = pot_I;

  valorpot_D = analogRead(pot_derivativo);
  pot_D = map(valorpot_D, 0, 1023, 0, 20)/20.0;//Função map() para converter a escala de 0 a 1023 para a escala de 0 a 1 com incremento de 0.05
  
  derivativo = pot_D;
}

