// SpotMicro (упрощённая модель): 12 серв, 4 ноги x 3 сустава
// Кнопки: 22 = идти, 23 = сесть, 24 = встать
#include <Servo.h>

// порядок ног: FL, FR, RL, RR. Сустав: 0 плечо, 1 бедро, 2 колено
const int PIN[4][3] = {{2,3,4}, {5,6,7}, {8,9,10}, {11,12,13}};
const bool MIRROR[4] = {false, true, false, true};   // правые ноги зеркальны
Servo srv[4][3];

const int STAND[3] = {90, 60, 120};   // стойка
const int SIT[3]   = {90, 100, 60};   // сидит

void setLeg(int leg, int a, int b, int c) {
  int v[3] = {a, b, c};
  for (int j = 0; j < 3; j++) {
    int ang = MIRROR[leg] ? 180 - v[j] : v[j];
    srv[leg][j].write(constrain(ang, 0, 180));
  }
}

void pose(const int p[3]) {
  for (int l = 0; l < 4; l++) setLeg(l, p[0], p[1], p[2]);
}

// рысь: диагональные пары ног двигаются в противофазе
void walkStep(float t) {
  for (int l = 0; l < 4; l++) {
    bool pairA = (l == 0 || l == 3);          // FL+RR
    float ph = t + (pairA ? 0 : PI);
    float lift = max(0.0f, sin(ph));          // фаза подъёма ноги
    int hip  = STAND[0] + 15 * cos(ph);       // вперёд-назад
    int fem  = STAND[1] - 25 * lift;          // поднять бедро
    int knee = STAND[2] + 25 * lift;          // согнуть колено
    setLeg(l, hip, fem, knee);
  }
}

void setup() {
  for (int l = 0; l < 4; l++)
    for (int j = 0; j < 3; j++) srv[l][j].attach(PIN[l][j]);
  for (int p = 22; p <= 24; p++) pinMode(p, INPUT_PULLUP);
  pose(STAND);
}

float t = 0;
void loop() {
  if (digitalRead(22) == LOW) { walkStep(t); t += 0.12; }
  else if (digitalRead(23) == LOW) pose(SIT);
  else if (digitalRead(24) == LOW) pose(STAND);
  delay(20);
}
