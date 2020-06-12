//Multiplex + Rotary encorder
const int selectPins[3] = {22, 1, 0}; // S0~22, S1~1, S2~0
const int sigPin = A7; // SIG~21

#define clck 0
#define outputA 1
#define outputB 2

int counter = 0;
int currStateA;
int prevStateA;

void setup() {
  Serial.begin (9600);

  for (int i=0; i<3; i++)
  {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], HIGH);
  }
  pinMode(sigPin, INPUT); // Set up Z as an input

    prevStateA = getValuePin(outputA);
}

void loop() {

  if(getValuePin(clck) != 1){
    counter = 0;
    Serial.print("CLK");
    Serial.println();
  }

  currStateA = getValuePin(outputA);
  if ((currStateA != prevStateA)&&(prevStateA == 1)) {
      if (getValuePin(outputB) != currStateA) {
        counter ++;
        Serial.print("ROTATION Clockwise");
        Serial.println();
      } else {
        counter --;
        Serial.print("ROTATION Anti-Clockwise");
        Serial.println();
        }
    }
   prevStateA = currStateA;
}

void selectMuxPin(byte pin)
{
  for (int i=0; i<3; i++)
  {
    if (pin & (1<<i))
      digitalWrite(selectPins[i], HIGH);
    else
      digitalWrite(selectPins[i], LOW);
  }
}

int convertToBinary(int value){
  if(value >= 500)
    return 1;
  else
    return 0;
}

int getValuePin(byte pinNumber){
  selectMuxPin(pinNumber);
  
  return convertToBinary(analogRead(A7));
}
