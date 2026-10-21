void setup() {
  pinMode(13, OUTPUT);
  pinMode(2, INPUT_PULLUP);
}
void loop() {
  digitalWrite(13, digitalRead(2) == LOW);
}
