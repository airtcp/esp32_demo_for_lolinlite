#include <WiFi.h>

// settings
#define EIGHT_DIGIT false     // true if your machine has 8 digits
#define ORIGIN_SENSOR false   // true if you installed origin sensor
#define TOUCH_CONTROL false   // true if you want to compensate origin by touch
#define ORIGIN_COMPENSATION 50 // compensation of origin mark position
#define ORIGIN_THRES 3500      // photo reflector sensor threshold
#define DEBUG true

const char* ssid     = "hometb"; // your WiFi's SSID
const char* password = "a527879262"; // your WiFi's password
#define TIMEZONE 8 // timezone (GMT = 0, Japan = 9)
// settings end

#if EIGHT_DIGIT
#define DIGIT 8
#else
#define DIGIT 4
#endif
typedef struct {
  int v[DIGIT];
} Digit;


// Motor and clock parameters
#define STEPS_PER_ROTATION 4096 // steps of a single rotation of motor

// wait for a single step of stepper
#define MOTOR_DELAY 1

// ports used to control the stepper motor
// if your motor rotate to the opposite direction,
int port[4] = {33, 25, 26, 27};

// sequence of stepper motor control
int seq[8][4] = {
  {  LOW, HIGH, HIGH,  LOW},
  {  LOW,  LOW, HIGH,  LOW},
  {  LOW,  LOW, HIGH, HIGH},
  {  LOW,  LOW,  LOW, HIGH},
  { HIGH,  LOW,  LOW, HIGH},
  { HIGH,  LOW,  LOW,  LOW},
  { HIGH, HIGH,  LOW,  LOW},
  {  LOW, HIGH,  LOW,  LOW}
};

// stepper motor rotation
void rotate(int step) {
  int count = 0;
  static int phase = 0;
  int i, j;
  int delta = (step > 0) ? 1 : 7;

  step = (step > 0) ? step : -step;
  for (j = 0; j < step; j++) {
    phase = (phase + delta) % 8;
    for (i = 0; i < 4; i++) {
      digitalWrite(port[i], seq[phase][i]);
    }
    count++;
    delay(MOTOR_DELAY);
    if (step - j < 410) delay(MOTOR_DELAY); // deacceleration
    if (count < 100) delay(MOTOR_DELAY); // acceleration
    if (count < 50) delay(MOTOR_DELAY); // acceleration
    if (count < 20) delay(MOTOR_DELAY); // acceleration
    if (count < 10) delay(MOTOR_DELAY); // acceleration
  }
  // power cut
  for (i = 0; i < 4; i++) {
    digitalWrite(port[i], LOW);
  }
}

void findOrigin(void) {
  while (analogRead(34) > ORIGIN_THRES) { // if origin is sensed, back a bit
    rotate(-1);
  }
  while (analogRead(34) < ORIGIN_THRES) { // find origin
    rotate(1);
  }
  rotate(ORIGIN_COMPENSATION);
  delay(1000);
}

#define KILL_BACKLASH 10

// avoid error accumuration of fractional part of 4096 / 10
void rotStep(int s) {
  static long currentPos;
  static long currentStep;

  currentPos += s;
  long diff = currentPos * STEPS_PER_ROTATION / 10 - currentStep;
  if (diff < 0) diff -= KILL_BACKLASH;
  else diff += KILL_BACKLASH;
  rotate(diff);
  currentStep += diff;
}

void printDigit(Digit d) {
#if DEBUG
  String s = "        ";
  int i;

  for (i = 0; i < DIGIT; i++) {
    s.setCharAt(i, d.v[i] + '0');
  }
  Serial.println(s);
#endif
}

//increase specified digit
Digit rotUp(Digit current, int digit, int num) {
  int freeplay = 0;
  int i;

  for (i = digit; i < DIGIT - 1; i++) {
    int id = current.v[i];
    int nd = current.v[i + 1];
    if (id <= nd) id += 10;
    freeplay += id - nd - 1;
  }
  freeplay += num;
  rotStep(-1 * freeplay);
  current.v[digit] = (current.v[digit] + num) % 10;
  for (i = digit + 1; i < DIGIT; i++) {
    current.v[i] = (current.v[i - 1] + 9) % 10;
  }
#if DEBUG
  Serial.print("up end : ");
  printDigit(current);
#endif
  return current;
}

// decrease specified digit
Digit rotDown(Digit current, int digit, int num) {
  int freeplay = 0;
  int i;

  for (i = digit; i < DIGIT - 1; i++) {
    int id = current.v[i];
    int nd = current.v[i + 1];
    if (id > nd) nd += 10;
    freeplay += nd - id;
  }
  freeplay += num;
  rotStep( 1 * freeplay);
  current.v[digit] = (current.v[digit] - num + 10) % 10;
  for (i = digit + 1; i < DIGIT; i++) {
    current.v[i] = current.v[i - 1];
  }
#if DEBUG
  Serial.print("down end : ");
  printDigit(current);
#endif
  return current;
}

// decrease or increase specified digit
Digit rotDigit(Digit current, int digit, int num) {
  if (num > 0) {
    return rotUp(current, digit, num);
  }
  else if (num < 0) {
    return rotDown(current, digit, -num);
  }
  else return current;
}

// set single digit to the specified number
Digit setDigit(Digit current, int digit, int num) {
  if (digit == 0) { // most significant digit
    int rot = num - current.v[0];
    // use decreasing rotation because following digits tend to be 000 or 999
    if (rot > 1) rot -= 10;
    return rotDigit(current, digit, rot);
  }
  int cd = current.v[digit];
  int pd = current.v[digit - 1];
  if (cd == num) return current;

  // check if increasing rotation is possible
  int n2 = num;
  if (n2 < cd) n2 += 10;
  if (pd < cd) pd += 10;
  if (pd <= cd || pd > n2) {
    return rotDigit(current, digit, n2 - cd);
  }
  // if not, do decrease rotation
  if (num > cd) cd += 10;
  return rotDigit(current, digit, num - cd);
}

Digit current = {0, 0, 0, 0};
void setNumber(Digit n) {
  for (int i = 0; i < DIGIT; i++) {
    current = setDigit(current, i, n.v[i]);
  }
}

void getNTP(void) {
  //WiFi.mode(WIFI_STA);
  //WiFi.begin();

  do {
#if DEBUG
    Serial.println("retry..");
#endif
    delay(300);
  } while (WiFi.status() != WL_CONNECTED);

  configTime(TIMEZONE * 3600L, 0, "ntp.nict.jp", "pool.ntp.org", "ntp.jst.mfeed.ad.jp");//NTPの設定
#if DEBUG
  printLocalTime();
#endif

  delay(1000);
  //WiFi.disconnect();
}

void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %Y-%m-%d %H:%M:%S");
}
void setup() {


#if DEBUG
  Serial.begin(115200);
  Serial.println("start");

#endif

  WiFi.mode(WIFI_STA);
  WiFi.begin();
  for (int i = 0; i < 8; i++) {
    Serial.println("try connected wifi...");
    delay(5000);
    if (WiFi.status() == WL_CONNECTED) {
      break;
    }
    if (i == 6) {
      break;
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.beginSmartConfig();

    //Wait for SmartConfig packet from mobile
    Serial.println("Waiting for SmartConfig.");
    while (!WiFi.smartConfigDone()) {
      delay(500);
      Serial.print(".");
    }

    Serial.println("");
    Serial.println("SmartConfig received.");

    //Wait for WiFi to connect to AP
    Serial.println("Waiting for WiFi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println("WiFi Connected.");

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
  getNTP();

  pinMode(port[0], OUTPUT);
  pinMode(port[1], OUTPUT);
  pinMode(port[2], OUTPUT);
  pinMode(port[3], OUTPUT);

#if ORIGIN_SENSOR
  findOrigin();
#endif

  rotate(STEPS_PER_ROTATION * (DIGIT - 1));
}


void loop() {
  static int prevhour;
  struct tm tmtime;

#if TOUCH_CONTROL
  if (touchRead(T8) < 15) { // GPIO33 pin
    rotate(10);
    return;
  }
  else if (touchRead(T6) < 15) { // GPIO14 pin
    rotate(-10);
    return;
  }
#endif

  getLocalTime(&tmtime);
  if (tmtime.tm_hour != prevhour) {
    prevhour = tmtime.tm_hour;
    getNTP();
  }

  Digit n;
#if EIGHT_DIGIT
  n.v[0] = (tmtime.tm_mon + 1) / 10;
  n.v[1] = (tmtime.tm_mon + 1) % 10;
  n.v[2] = tmtime.tm_mday / 10;
  n.v[3] = tmtime.tm_mday % 10;
  n.v[4] = tmtime.tm_hour / 10;
  n.v[5] = tmtime.tm_hour % 10;
  n.v[6] = tmtime.tm_min / 10;
  n.v[7] = tmtime.tm_min % 10;
#else
  n.v[0] = tmtime.tm_hour / 10;
  n.v[1] = tmtime.tm_hour % 10;
  n.v[2] = tmtime.tm_min / 10;
  n.v[3] = tmtime.tm_min % 10;
#endif
  setNumber(n);

  delay(1000);
}
