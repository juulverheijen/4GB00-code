byte statusLed = 13;
byte sensorInterrupt = 0; // 0 = digital pin 2
byte sensorPin = 2;
float calibrationFactor = 30.5;
volatile byte pulseCount;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long oldTime;
const int pressureSensorPin = A0;
void pulseCounter()
{
  pulseCount++;
}
void setup()
{
  Serial.begin(9600);
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, HIGH);
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  oldTime = 0;
  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
}
void loop()
{
  if ((millis() - oldTime) > 1000)
  {
    detachInterrupt(sensorInterrupt);
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
    oldTime = millis();
    flowMilliLitres = (flowRate / 60) * 1000;
    totalMilliLitres += flowMilliLitres;
    unsigned int frac;
    Serial.print("Flow rate: ");
    Serial.print(int(flowRate));
    Serial.print("L/min");
    Serial.print("\t");
    Serial.print("Output Liquid Quantity: ");
    Serial.print(totalMilliLitres);
    Serial.println("mL");
    Serial.print("\t"); // Print tab space
    Serial.print(totalMilliLitres / 1000);
    Serial.print("L");
    int sensorValue = analogRead(pressureSensorPin);
  // Convert the analog value to pressure (adjust the conversion as needed)
  float pressurePsi = map(sensorValue, 0, 1023, 0, 100);
    Serial.print(" Pressure (Bar)");
    Serial.print( pressurePsi /19 );
    // Reset the pulse counter so we can start incrementing again
    pulseCount = 0;
    // Enable the interrupt again now that we've finished sending output
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
  }
}

