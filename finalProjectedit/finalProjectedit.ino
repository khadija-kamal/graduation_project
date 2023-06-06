#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#define buttonPin2 3
#define buttonPin1 2
#define buzzer 12
#define MPU_addr 0x68
#define UpperThreshold 518
#define LowerThreshold 490
#define id 1
#define protector 4
LiquidCrystal_I2C lcd(0x27, 16, 2);
TinyGPSPlus gps;
SoftwareSerial gpsSerial(7, 6);
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
bool alert_cancel = false;
int counter1 = 0;
int counter2 = 0;
int onetime = 0;
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
                           "Normal" };
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

void ISR_1() {
  if (digitalRead(buttonPin1) == LOW) {
    counter1++;
    if (counter1 == 3) counter1 = 1;
    if (counter1 == 1) condition = true;
    if (counter1 == 2) asm volatile("  jmp 0");
  }
}

void ISR_2() {
  if (digitalRead(buttonPin2) == LOW) {
    counter2++;
    if (counter2 == 3) counter2 = 1;
    if (counter2 == 1) {
      onetime = 1;
      alert_cancel = true;
      digitalWrite(buzzer, LOW);
    }
    if (counter2 == 2) alert_cancel = false;
  }
}

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.print(F("     System"));
  lcd.setCursor(0, 1);
  lcd.print(F(" Initialization"));
  gpsSerial.begin(9600);
  GpsInfo();
  sim.begin(9600);
  delay(1000);
  BTSerial.begin(9600);
  pinMode(buttonPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin1), ISR_1, FALLING);
  pinMode(buttonPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin2), ISR_2, FALLING);
  pinMode(buzzer, OUTPUT);
  pinMode(protector, OUTPUT);
  lcd.clear();
  lcd.print(F(" Fall Detection "));
  lcd.setCursor(0, 1);
  lcd.print(F("     System  "));
  delay(500);
  Wire.setClock(4000000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}
void loop() {
  if (condition == 1) {
    digitalWrite(buzzer, LOW);
    digitalWrite(protector, LOW);
    lcd.clear();
    lcd.print(F("System is turned"));
    lcd.setCursor(0, 1);
    lcd.print(F("       off"));
    while (1);
  }
  if (condition == 0) {

    if (stateStr[9] == "Fall" && alert_cancel == 1 && onetime == 1) {
      onetime = 0;
      SendMessage(3);
    }
    mpu_read();
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
        BPM =   (1.0 / PulseInterval) * 60.0 * 1000;
        previousMillis2 = millis();
        lcd.clear();
        lcd.print(String("State: ") + String(stateStr[9]));
        lcd.setCursor(0, 1);
        lcd.print(String("Pulse: ") + String(BPM));
      }
    }
    if (sim.available()) {
      delay(1000);
      Serial.println(sim.readString());
    }
    if ((ax >= -1.30) && (ax <= 4.30) && (ay <= 2.50) && (ay >= -1.30) && (az <= 7.50) && (az >= -3.30)) st = N;
    if ((ax >= -0.38) && (ax <= 0.38) && (ay <= -0.35) && (ay >= -0.88) && (az >= 0.26) && (az <= 0.88)) st = RB;
    if ((ax >= -0.38) && (ax <= 0.38) && (ay <= -0.35) && (ay >= -0.88) && (az >= -0.88) && (az <= -0.26)) st = RF;
    if ((ax >= -0.38) && (ax <= 0.38) && (ay <= 0.88) && (ay >= 0.35) && (az >= 0.26) && (az <= 0.88)) st = BL;
    if ((ax >= -0.38) && (ax <= 0.38) && (ay <= 0.88) && (ay >= 0.35) && (az <= -0.26) && (az >= -0.88)) st = LF;
    if ((ax >= -0.38) && (ax <= 0.38) && (ay >= 0.80) && (ay <= 1.20) && (az <= 0.68) && (az >= -0.68)) st = L;
    if ((ax >= -0.38) && (ax <= 0.38) && (ay <= 0.68) && (ay >= -0.68) && (az <= 1.20) && (az >= 0.80)) st = B;
    if ((ax >= -0.38) && (ax <= 0.38) && (ay <= 0.68) && (ay >= -0.68) && (az <= -0.80) && (az >= -1.20)) st = F;
    if ((ax >= -0.38) && (ax <= 0.38) && (ay <= -0.80) && (ay >= -1.20) && (az <= 0.68) && (az >= -0.68)) st = R;
    BTSerial.listen();
    if (BTSerial.available() > 0) {
      b = BTSerial.read();
      if (b == 'S') {
        alert_cancel = true;
        if (stateStr[9] == "Fall") SendMessage(2);
        b = " ";
      }
    }
    if ((millis() - now) >= 1000) {
      now = millis();
      String btsend = (String)id + "," + (String)stateStr[9] + "," + (String)BPM;
      BTSerial.println(btsend);
    }

    time_check();
    Serial.print(amplitude);
    Serial.print(F("\t"));
    Serial.print(stateStr[st]);
    Serial.println(" ");

    PREampl = amplitude;
    delay(50);
  }
}

String readSerial() {
  delay(100);
  if (sim.available()) {
    return sim.readString();
  }
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
        } else {
          delay(1000);
          ++gps_st;
          Serial.println(gps_st);

        }
      }

  } while (gps_st != -1);
}

void SendMessage(int m) {
  String SMS;
  lcd.clear();
  if (m == 1) {
    GpsInfo();
    if (stateStr[st] == "F") {
      SMS = "Fall Alert: patient with ID " + (String)id + ", fell forward and BPM of " + (String)BPM + ".Location of the patient\r";
    } else {
      SMS = "Fall Alert: patient with ID " + (String)id + "and BPM of" + (String)BPM + ".Location of the patient\r";
    }
    SMS += "http://maps.google.com/maps?q=loc:";
    SMS += latitude + "," + logitude;
    lcd.print(F("  Patient Fall"));
    lcd.setCursor(0, 1);
    lcd.print(F("     Alert"));
  }
  if (m == 2 && b == 'S') {
    digitalWrite(buzzer, LOW);
    SMS = "Medical assistant has reached the patient with ID " + (String)id;
    lcd.print(F(" Medical cancel"));
    lcd.setCursor(0, 1);
    lcd.print(F("     Alert"));
  }
  if (m == 3) {
    SMS = "The patient with an ID " + (String)id + " is alright";
    lcd.print(F(" Patient cancel"));
    lcd.setCursor(0, 1);
    lcd.print(F("     Alert"));
  }
  delay(1000);
  sim.listen();
  sim.println("AT");
  Serial.println(readSerial());
  sim.println("AT+CBC");
  Serial.println(readSerial());
  sim.println("AT+CSQ");
  Serial.println(readSerial());
  sim.println("AT+CMGF=1");
  Serial.println(readSerial());
  sim.println("AT+CMGS=\"+218925247824\"");
  Serial.println(readSerial());
  sim.println(SMS);
  sim.println((char)26);
  Serial.println(readSerial());
  gps_st = 0;
  SMS = " ";
  delay(1000);
  delay(3000);

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
    // stateStr[9] = "Fall";
  } else {
    count = 0;
    check = 0;
    stateStr[9] = "Normal";
    digitalWrite(protector, LOW);
  }
  if (((millis() - lastRefreshTime) >= 2000) && (flag == 1) && (lastRefreshTime != 0)) {
    if (check == 1 && count >= 6) {
      stateStr[9] = "Fall";
      if (alert_cancel == 0) {
        digitalWrite(buzzer, HIGH);
        digitalWrite(protector, HIGH);
        SendMessage(1);
      }
    }
    lastRefreshTime = 0;
    flag = 0;
  }
  if (((millis() - larefresh) >= 60000) && (larefresh != 0)) {
    if (check == 1) {
      stateStr[9] = "Fall";
      if (alert_cancel == 0) {
        digitalWrite(buzzer, HIGH);
        digitalWrite(protector, HIGH);
        SendMessage(1);
      }
    }
    larefresh = 0;
    count = 0;
  }
}