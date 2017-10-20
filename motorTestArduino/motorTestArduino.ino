const int IA1 = 10;
const int IA2 = 11;
const int IB1 = 12;
const int IB2 = 13;

void setup() {
  pinMode(IA1, OUTPUT);
  pinMode(IA2, OUTPUT);
  pinMode(IB1, OUTPUT);
  pinMode(IB2, OUTPUT);
}

void loop() {
  int i;
  int td = 5000;
  int ns = 126;
  for(i = 0; i < 255; i++){
    MA1_Forward(255 - i);//Motor MA1 forward; PWM speed control
    printf(255-i);
    delay(5000/126);
  }
  
  for(i = 0; i < 255; i++){
    MA2_Backward(255 - i);//Motor MA1 forward; PWM speed control
    printf(255-i);
    delay(10000/126);
  }
  MA1_Forward(0);
  delay(1000);
  //MA2_Backward(200);//Motor MA1 backward; PWM speed control
  //delay(1000);
}
void MA1_F_Fast(int Speed1) //fast decay; Speed = High duty-cycle
{
  analogWrite(IA1, Speed1);
  digitalWrite(IA2, LOW);
}

void MA1_F_Slow(int Speed1) //slow decay; Speed = Low duty-cycle
{
  int Speed2 = 255 - Speed1;
  analogWrite(IA1, HIGH);
  digitalWrite(IA2, Speed2 );
}

void MB1_F_Fast(int Speed1) //fast decay; Speed = High duty-cycle
{
  analogWrite(IB1, Speed1);
  digitalWrite(IB2, LOW);
}

void MB1_F_Slow(int Speed1) //slow decay; Speed = Low duty-cycle
{
  int Speed2 = 255 - Speed1;
  analogWrite(IB1, HIGH);
  digitalWrite(IB2, Speed2 );
}

void MA1_R_Fast(int Speed1) //fast decay; Speed = High duty-cycle
{
  analogWrite(IA2, Speed1);
  digitalWrite(IA1, LOW);
}

void MA1_R_Slow(int Speed1) //slow decay; Speed = Low duty-cycle
{
  int Speed2 = 255 - Speed1;
  analogWrite(IA2, HIGH);
  digitalWrite(IA1, Speed2 );
}

void MB1_R_Fast(int Speed1) //fast decay; Speed = High duty-cycle
{
  analogWrite(IB2, Speed1);
  digitalWrite(IB1, LOW);
}

void MB1_R_Slow(int Speed1) //slow decay; Speed = Low duty-cycle
{
  int Speed2 = 255 - Speed1;
  analogWrite(IB2, HIGH);
  digitalWrite(IB1, Speed2);
}
