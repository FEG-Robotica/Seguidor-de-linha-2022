# teste_de_posicao

int AIN1=7;  // Esquerdo    // AIN ligado no motor esquerdo -> controla o sentido de rotação do motor
int AIN2=6;                 // Segundo pino do motor, um para frente e o outro para trás
int PWMA=5;                 // PWM define o tempo que o circuito irá receber tensão e o tempo em que não irá receber tensão
int STBY=8;                 // Alimentação geral do robô
int BIN1=9;//Direito        // BIN ligado no motor direito
int BIN2=10;                
int PWMB=11;                // Verificar todos os pinos na hora de testar
int leitura[8];
double sensor[8];
double bspeed=10;     // Testar no dia da competição
double VelocR=0.0;     
double PID = 0;
double setpoint = 4.0; 
double erro=0.0;
double P=0.0;
double I=0.0;
double D=0.0;
double kp=15;         // Testar os valores de kp e kd, não utilizar ki
double ki=0;          // Começar com kp = 25 e ir aumentando o máximo que der (kp = 50, por exemplo)
double kd=0;          // Ao alcançar o valor máximo, deixar kp e kd = 1/2 kp máximo. Ir diminuindo kd se o robo ficar instável
double pos= 0.0;
double posin=0.0;
double dt=0.0;
double de=0.0;
double tf=0.0; 
int i;
double erroi=0;
int modulo_start = 23;










void setup() {
  
 Serial.begin(9600);
  pinMode(3, INPUT); //sensor 7
  pinMode(2, INPUT); //sensor 1
  pinMode(A1, INPUT); //sensor 5 - meio +1
  pinMode(A2, INPUT); //sensor 6- meio +2
  pinMode(A3, INPUT); //sensor 2- meio -2
  pinMode(A4, INPUT); //sensor 3- meio -1
  pinMode(A5, INPUT);//sensor 4 - meio
  pinMode(AIN1, OUTPUT);//detectar a linha e mexer os motores.
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

}

void sensores()
{
  for(i = 2; i<7 ; i++)
  {
    if(sensor[i] < 800)
    {
      leitura[i] = 1;
    }
    else
    leitura[i] = 0;
  }
}


double posicao()                //posicao de acordo com os 5 sensores do centro
{
  int i;
  double pos = 0.0, total=0.0, local;
  for(i = 2; i < 7; i++)
  {
    if(leitura[i] == 1)
    {
       pos = pos + i;
       
       total = total + 1;
    }
  }
  if(total != 0)
  {
    local = pos/total;
    return local;
  }
  else
  return posin;
}







void loop() {

 sensor[0] = 0;
 sensor[7] = digitalRead(3);
 sensor[1] = digitalRead(2);
 sensor[2] = analogRead(A3);
 sensor[3] = analogRead(A4);
 sensor[4] = analogRead(A5);
 sensor[5] = analogRead(A1);
 sensor[6] = analogRead(A2);

  leitura[1] = sensor[1];
  leitura[7] = sensor[7];

  sensores();
  
  pos = posicao();
  
  posin = pos;//posicao do robo

  erro=setpoint-pos;  //Cálculo do erro // Infinitésimo do erro, pequena variação
  
  de=erro-erroi;  // Infinitésimo do erro, pequena variação 
  
  erroi=erro; //erroi recebe o erro atual para fazer o "de" depois

  dt=(millis()-tf)/1000.0; //Função millis corresponde ao tempo que oprograma ta rodando, dt é o inifitésimo do tempo

  tf=millis();  //Mesmo conceito do erroi
  
  P = kp*erro; //Controle Proporcional

  I = I + ki*erro*dt; //Controle Integral

  D = kd*(de/dt); //Controle Derivativo

  PID = P+I-D; //PID COMPLETO

  delay(1000);
  Serial.print(" posicao =  ");
  Serial.println(posicao());
  delay(1000);
  Serial.print(" PID =  ");
  Serial.println(PID);
  

 

 }
