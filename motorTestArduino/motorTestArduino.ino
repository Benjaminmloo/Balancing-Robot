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
  fullTestLoop();
}


void fullTestLoop()
{
  int i;
  int td = 10000;
  int ns = 255;

  for (i = 0; i < ns; i++) {
    MA_F_Fast(ns - i);//Motor MA1 forward; PWM speed control
    delay(td / ns);
  }
  digitalWrite(IA1, LOW);
  digitalWrite(IA2, LOW);
  delay(td / 5);

  for (i = 0; i < ns; i++) {
    MA_F_Slow(ns - 1);//Motor MA1 forward; PWM speed control
    delay(td / ns);
  }

  digitalWrite(IA1, LOW);
  digitalWrite(IA2, LOW);
  delay(td / 5);



  for (i = 0; i < ns; i++) {
    MA_R_Fast(ns - i);//Motor MA1 forward; PWM speed control
    delay(td / ns);
  }

  digitalWrite(IA1, LOW);
  digitalWrite(IA2, LOW);
  delay(td / 5);

  for (i = 0; i < ns; i++) {
    MA_R_Slow(ns - i);//Motor MA1 forward; PWM speed control
    delay(td / ns);
  }

  digitalWrite(IA1, LOW);
  digitalWrite(IA2, LOW);
  delay(td / 5);

  for (i = 0; i < ns; i++) {
    MB_F_Fast(ns - i);//Motor MA1 forward; PWM speed control
    delay(td / ns);
  }
  digitalWrite(IA1, LOW);
  digitalWrite(IA2, LOW);
  delay(td / 5);

  for (i = 0; i < ns; i++) {
    MB_F_Slow(ns - i);//Motor MA1 forward; PWM speed control
    delay(td / ns);
  }
  digitalWrite(IB1, LOW);
  digitalWrite(IB2, LOW);
  delay(td / 5);

  for (i = 0; i < ns; i++) {
    MB_R_Fast(ns - i);//Motor MA1 forward; PWM speed control
    delay(td / ns);
  }
  digitalWrite(IB1, LOW);
  digitalWrite(IB2, LOW);
  delay(td / 5);

  for (i = 0; i < ns; i++) {
    MB_R_Slow(ns - i);//Motor MA1 forward; PWM speed control
    delay(td / ns);
  }
  digitalWrite(IB1, LOW);
  digitalWrite(IB2, LOW);
  delay(td / 5);
}
void MA_F_Fast(int Speed1) //fast decay; Speed = High duty-cycle
{
  analogWrite(IA1, Speed1);
  digitalWrite(IA2, LOW);
}

void MA_F_Slow(int Speed1) //slow decay; Speed = Low duty-cycle
{
  int Speed2 = 255 - Speed1;
  digitalWrite(IA1, HIGH);
  analogWrite(IA2, Speed2 );
}

void MB_F_Fast(int Speed1) //fast decay; Speed = High duty-cycle
{
  analogWrite(IB1, Speed1);
  digitalWrite(IB2, LOW);
}

void MB_F_Slow(int Speed1) //slow decay; Speed = Low duty-cycle
{
  int Speed2 = 255 - Speed1;
  digitalWrite(IB1, HIGH);
  analogWrite(IB2, Speed2 );
}

void MA_R_Fast(int Speed1) //fast decay; Speed = High duty-cycle
{
  analogWrite(IA2, Speed1);
  digitalWrite(IA1, LOW);
}

void MA_R_Slow(int Speed1) //slow decay; Speed = Low duty-cycle
{
  int Speed2 = 255 - Speed1;
  digitalWrite(IA2, HIGH);
  analogWrite(IA1, Speed2 );
}

void MB_R_Fast(int Speed1) //fast decay; Speed = High duty-cycle
{
  analogWrite(IB2, Speed1);
  digitalWrite(IB1, LOW);
}

void MB_R_Slow(int Speed1) //slow decay; Speed = Low duty-cycle
{
  int Speed2 = 255 - Speed1;
  digitalWrite(IB2, HIGH);
  analogWrite(IB1, Speed2);
}
