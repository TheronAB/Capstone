
const int voltPinA = 36;
const int voltPinB = 39;
const int voltPinC = 34;
const int currentPinA = 35;
const int currentPinB = 32;
const int currentPinC = 33;
//const int tempPin = 15;

int voltAadc = 0;
int voltBadc = 0;
int voltCadc = 0;
int currentAadc = 0;
int currentBadc = 0;
int currentCadc = 0;
//int tempadc = 0;

int voltValA = 0;
int voltValB = 0;
int voltValC = 0;
int currentValA = 0;
int currentValB = 0;
int currentValC = 0;
//int tempVal = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  i = 0;
  Va = 0;
  Vb = 0;
  Vc = 0;
  Aa = 0;
  Ab = 0;
  Ac = 0;
  //T = 0;
  while (i < 100):
    voltAadc = analogRead(voltPinA);
    voltBadc = analogRead(voltPinB);
    voltCadc = analogRead(voltPinC);
    currentAadc = analogRead(currentPinA);
    currentBadc = analogRead(currentPinB);
    currentCadc = analogRead(currentPinC);
    //tempadc = analogRead(tempPin);
    voltValA = floor(0.00080586*voltAadc);
    voltValB = floor(0.00080586*voltBadc);
    voltValC = floor(0.00080586*voltCadc);
    currentValA = floor(0.00080586*currentAadc);
    currentValB = floor(0.00080586*currentBadc);
    currentValC = floor(0.00080586*currentCadc);
    //tempVal = floor(0.00080586*tempadc);
    if (voltValA > Va):
      Va = voltValA;
    if (voltValB > Vb):
      Vb = voltValB;
    if (voltValC > Vc):
      Vc = voltValC;
    if (currentValA > Aa):
      Aa = currentValA;
    if (currentValB > Ab):
      Ab = currentValB;
    if (currentValC > Ac):
      Ac = currentValC;
    //if (tempVal > T):
    //  T = tempVal  
    i += 1;    
  delay(100);
  Serial.print("Phase A: voltage = ");
  Serial.print(Va);
  Serial.print(" current = ");
  Serial.print(Aa);
  Serial.print(" Phase B: voltage = ");
  Serial.print(Vb);
  Serial.print(" current = ");
  Serial.print(Ab);
  Serial.print(" Phase C: voltage = ");
  Serial.print(Vc);
  Serial.print(" current = ");
  Serial.print(Ac);
  //Serial.print("Temperature = ");
  //Serial.print(tempVal);
}
