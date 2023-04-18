#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#define buttonPin2 5
#define buttonPin1 4
#define buzzer 12
#define MPU_addr 0x68
#define UpperThreshold 518
#define LowerThreshold 490
#define id 1
#define protector 7

LiquidCrystal_I2C lcd(0x27, 16, 2);
TinyGPSPlus gps;
SoftwareSerial gpsSerial(2, 3);
SoftwareSerial sim(10, 11);
SoftwareSerial BTSerial(9, 8);

bool IgnoreReading = false;
bool FirstPulseDetected = false;
unsigned long FirstPulseTime = 0;
unsigned long SecondPulseTime = 0;
unsigned long PulseInterval = 0;
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
unsigned long now = 0;
unsigned long lastRefreshTime = 0;
unsigned long larefresh = 0;
int gps_st = 0;
int reading = 0;
float BPM = 0;
char b;
String latitude = "";
String logitude = "";
float ax = 0, ay = 0, az = 0;
int amplitude, PREampl;
int count = 0;
int check = 0;
int flag = 0;
bool condition = false;
int buttonPushCounter[2] = { 2, 2 };
int lastButtonState[2] = { 0, 0 };
bool alert_cancel = false;
enum State { F,
             L,
             B,
             R,
             BL,
             N,
             LF,
             RB,
             RF };
const char* stateStr[] = { "F",
                           "L",
                           "B",
                           "R",
                           "BL",
                           "N",
                           "LF",
                           "RB",
                           "RF",
                           "normal" };
State st = N;

void mpu_read(void) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr, 6);
  int16_t AcX = Wire.read() << 8 | Wire.read();
  int16_t AcY = Wire.read() << 8 | Wire.read();
  int16_t AcZ = Wire.read() << 8 | Wire.read();
  ax = (float)AcX / 4096 - 0.06;
  ay = (float)AcY / 4096 + 0.01;
  az = (float)AcZ / 4096 + 0.08;
  float raw_amplitude = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  amplitude = raw_amplitude * 10;
}

void software_Reset() {
  asm volatile("  jmp 0");
}


void setup() {
  Wire.setClock(4000000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.print(F("     System"));
  lcd.setCursor(0, 1);
  lcd.print(F(" Initialization"));
  delay(2000);
  Serial.begin(9600);
  gpsSerial.begin(9600);
  GpsInfo();
  BTSerial.begin(9600);
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(protector, OUTPUT);
  lcd.clear();
  lcd.print(F(" Fall Detection "));
  lcd.setCursor(0, 1);
  lcd.print(F("     System  "));
  sim.begin(9600);
  delay(1000);
}
void loop() {

  int currentState = digitalRead(buttonPin1);
  if (lastButtonState[0] != currentState) {
    lastButtonState[0] = currentState;
    if (currentState == HIGH) {
      buttonPushCounter[0]++;
      if (buttonPushCounter[0] == 3) buttonPushCounter[0] = 1;
      if (buttonPushCounter[0] == 1) {
        condition = true;
      } else {
        software_Reset();
      }
    }
  }

  if ((millis() - previousMillis) >= 10) {
    previousMillis = millis();
    reading = analogRead(0);
    if (reading > UpperThreshold && IgnoreReading == false) {
      if (FirstPulseDetected == false) {
        FirstPulseTime = millis();
        FirstPulseDetected = true;
      } else {
        SecondPulseTime = millis();
        PulseInterval = SecondPulseTime - FirstPulseTime;
        FirstPulseTime = SecondPulseTime;
      }
      IgnoreReading = true;
    }
    if (reading < LowerThreshold && IgnoreReading == true) IgnoreReading = false;
    if ((millis() - previousMillis2) >= 1000) {
      previousMillis2 = millis();
      BPM = (1.0 / PulseInterval) * 60.0 * 1000;
    }
  }
  int currentState2 = digitalRead(buttonPin2);
  if (lastButtonState[1] != currentState2) {
    lastButtonState[1] = currentState2;
    if (currentState2 == HIGH) {
      buttonPushCounter[1]++;
      if (buttonPushCounter[1] == 3) buttonPushCounter[1] = 1;
      if (buttonPushCounter[1] == 1) {
        alert_cancel = true;
      } else {
        alert_cancel = false;
      }
    }
  }
  mpu_read();
  if ((ax >= -1.30) && (ax <= 4.30) && (ay <= 2.50) && (ay >= -1.30) && (az <= 7.50) && (az >= -3.30)) st = N;
  if ((ax >= 0.55) && (ax <= 0.88) && (ay <= 0.40) && (ay >= -0.11) && (az <= 0.92) && (az >= 0.48)) st = BL;
  if ((ax >= -1.10) && (ax <= -0.30) && (ay <= 0.40) && (ay >= -0.11) && (az <= 1.10) && (az >= 0.16)) st = LF;
  if ((ax >= 0.29) && (ax <= 1.10) && (ay <= 0.22) && (ay >= -0.11) && (az >= -0.88) && (az <= 1.10)) st = RB;
  if ((ax >= -1.10) && (ax <= -0.29) && (ay <= 0.45) && (ay >= -0.20) && (az <= -0.26) && (az >= -1.10)) st = RF;
  if ((ax <= 2.12) && (ax >= 0.73) && (ay <= 0.33) && (ay >= -0.22) && (az <= 0.54) && (az >= -0.26)) st = B;
  if ((ax >= -0.33) && (ax <= 0.57) && (ay <= 0.41) && (ay >= -0.28) && (az <= 1.39) && (az >= 0.80)) st = L;
  if ((ax >= -0.30) && (ax <= 0.47) && (ay <= 0.45) && (ay >= -0.13) && (az <= -0.84) && (az >= -1.55)) st = R;
  if ((ax >= -1.40) && (ax <= -0.85) && (ay <= 0.46) && (ay >= -0.22) && (az <= 0.17) && (az >= -0.37)) st = F;
  BTSerial.listen();
  if (BTSerial.available() > 0) {
    b = BTSerial.read();
    if (b == 'S') {
      alert_cancel = true;
      buttonPushCounter[1] = 1;
      if (stateStr[9] == "Fall") SendMessage();
      b = " ";
    }
  }
  if ((millis() - now) >= 200) {
    now = millis();
    String btsend = (String)id + "," + (String)stateStr[9] + "," + (String)BPM;
    BTSerial.println(btsend);
  }
  if (alert_cancel == 1) {
    digitalWrite(buzzer, LOW);
    delay(50);
  }
  time_check();
  PREampl = amplitude;
  delay(50);
}

void GpsInfo() {
  do {
    gpsSerial.listen();
    while (gpsSerial.available() > 0)
      if (gps.encode(gpsSerial.read())) {
        if (gps.location.isValid()) {
          latitude = String(gps.location.lat(), 6);
          logitude = String(gps.location.lng(), 6);
          gps_st = -1;
          gpsSerial.end();
        } else {
          delay(1000);
          ++gps_st;
        }
      }
  } while (gps_st != -1);
}


void SendMessage() {
  GpsInfo();
  String SMS;
  SMS = "Patient Fall Alert, bpm=" + (String)BPM + ".The site of the patient\r";
  SMS += "http://maps.google.com/maps?q=loc:";
  SMS += latitude + "," + logitude;
  if (b == 'S') SMS = "Medical assistance reached the patient with an ID " + (String)id + "\r";
  
  if (alert_cancel == 0) {
    lcd.clear();
    lcd.print(F("  Patient Fall"));
    lcd.setCursor(0, 1);
    lcd.print(F("     Alert"));
  }  
  if(alert_cancel == 1){
    lcd.clear();
    lcd.print(F("  Medical cancel"));
    lcd.setCursor(0, 1);
    lcd.print(F("     Alert"));
  }
  
  sim.listen();
  delay(1000);
  sim.println("AT"); 
  updateSerial();
  sim.println("AT+CMGF=1"); 
  updateSerial();
  sim.println("AT+CMGS=\"+218919774686\""); 
  updateSerial();
  sim.print(SMS);  
  updateSerial();
  delay(500);
  sim.write(26);
  delay(5000);
  sim.end();
  gps_st = 0;
  
}

void updateSerial() {
  delay(500);
  while (Serial.available()) {
    sim.write(Serial.read()); 
  }
  while (sim.available()) {
    Serial.write(sim.read());  
  }
}
void time_check() {
  if ((amplitude - PREampl) >= 6) {
    flag = 1;
    lastRefreshTime = millis();
  }
  if (st == F || st == B || st == R || st == L || st == RB || st == RF || st == LF || st == BL) {
    count = count + 1;
    if (count == 1) larefresh = millis();
    check = 1;
  } else {
    count = 0;
    check = 0;
    stateStr[9] = "normal";
    digitalWrite(protector, LOW);
    lcd.clear();
    lcd.print(F("  Normal State"));
  }
  if (((millis() - lastRefreshTime) >= 2000) && (flag == 1) && (lastRefreshTime != 0)) {
    if (check == 1 && count >= 6 && alert_cancel == 0) {
      digitalWrite(buzzer, HIGH);
      digitalWrite(protector, HIGH);
      stateStr[9] = "Fall";
      SendMessage();
    }
    lastRefreshTime = 0;
    flag = 0;
  }
  if (((millis() - larefresh) >= 60000) && (larefresh != 0)) {
    if (check == 1 && alert_cancel == 0) {
      digitalWrite(buzzer, HIGH);
      digitalWrite(protector, HIGH);
      stateStr[9] = "Fall";
      SendMessage();
    }
    larefresh = 0;
    count = 0;
  }
}