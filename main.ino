
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
  delay(1000);
  Serial.print("Phase A: voltage = ");
  Serial.print(voltValA);
  Serial.print(" current = ");
  Serial.print(currentValA);
  Serial.print(" Phase B: voltage = ");
  Serial.print(voltValB);
  Serial.print(" current = ");
  Serial.print(currentValB);
  Serial.print(" Phase C: voltage = ");
  Serial.print(voltValC);
  Serial.print(" current = ");
  Serial.print(currentValC);
  //Serial.print("Temperature = ");
  //Serial.print(tempVal);
}
