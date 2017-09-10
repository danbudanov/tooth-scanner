int buttonPin = 8;

int reading, old_reading;
void setup() {
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);

  old_reading = digitalRead(buttonPin);
}

void loop() {
  reading = digitalRead(buttonPin);
  if ((old_reading==0) && (reading==1)) {
    Serial.print("Button");
  }
  old_reading = reading;
}
