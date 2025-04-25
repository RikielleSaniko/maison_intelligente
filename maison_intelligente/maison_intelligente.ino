#include <AccelStepper.h>
#include <HCSR04.h>
#include <LCD_I2C.h>
#include <U8g2lib.h>

#define MOTOR_INTERFACE_TYPE 4
#define IN_1 9
#define IN_2 10
#define IN_3 11
#define IN_4 12
#define TRIGGER_PIN 7
#define ECHO_PIN 6
#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN 32

AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
LCD_I2C lcd(0x27, 16, 2);

U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(U8G2_R0, CLK_PIN, DIN_PIN, CS_PIN, U8X8_PIN_NONE, U8X8_PIN_NONE);

String numeroEtudiant = "2410307";
String labo = "LABO 4B";
int MIN_ANGLE = 10;
int MAX_ANGLE = 170;
int targetAngle = 0;
const int MIN_DISTANCE = 30;
const int MAX_DISTANCE = 60;
unsigned long currentTime = 0;
int currentDistance = 0;
int minMoveTo = 0;
int maxSpeed = 1000;
int acceleration = 200;
int maxDegree = 360;
int distanceInterval = 50;
int redled = 5;
int blueled = 3;
int buzzer = 8;
int maxRawDist = 400;
int minRawDist = 0;
int ledInterval = 100;
unsigned long lastUpdateTime = 0;
const long UPDATE_INTERVAL = 100;
int min = 0;
int delai = 2000;
int serial = 9600;
int time=3000;

int alarmDistance = 15;
const unsigned long alarmEndTime = time;

float distance = 0;
bool alarmActive = false;
unsigned long lastAlarmTime = 0;
bool ledState = false;
bool etatled = LOW;
bool buzzerState = LOW;

enum etatSystem { AUTOMATIQUE,
                  TROP_PRES,
                  TROP_LOIN };
etatSystem etat = AUTOMATIQUE;

void setup() {
  Serial.begin(serial);
  lcd.begin();
  lcd.backlight();
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(redled, OUTPUT);
  pinMode(blueled, OUTPUT);

  myStepper.setMaxSpeed(maxSpeed);
  myStepper.setAcceleration(acceleration);

  u8g2.begin();
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.setContrast(5);
}

void demarrage() {
  lcd.setCursor(0, 0);
  lcd.print(numeroEtudiant);
  lcd.setCursor(0, 1);
  lcd.print(labo);
  delay(delai);
}

int calculeDistance() {
  static unsigned long lastTime = 0;
  distance = hc.dist();
  if (currentTime - lastTime >= distanceInterval) {
    lastTime = currentTime;

    if (distance <= minRawDist || distance > maxRawDist) return currentDistance;
  }
  return distance;
}

void moteur() {
  int maxMoveTo = 2038;
  int MAX_ANGLE = 170;
  int maxDegree = 360;
  if (etat == AUTOMATIQUE) {
    int targetSteps = map(targetAngle, min, MAX_ANGLE, min, (maxMoveTo * MAX_ANGLE) / maxDegree);
    if ((myStepper.currentPosition() - targetSteps > MIN_ANGLE) || (targetSteps - myStepper.currentPosition() > MIN_ANGLE)) {
      myStepper.moveTo(targetSteps);
    }
  } else {
    myStepper.stop();
  }
}


void etatSystem() {
  if (distance < MIN_DISTANCE) {
    etat = TROP_PRES;
    myStepper.disableOutputs();
    myStepper.stop();
  } else if (distance > MAX_DISTANCE) {
    etat = TROP_LOIN;
    myStepper.disableOutputs();
    myStepper.stop();
  } else {
    etat = AUTOMATIQUE;
    targetAngle = map(distance, MIN_DISTANCE, MAX_DISTANCE, MIN_ANGLE, MAX_ANGLE);
  }
}

void machineEtat() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print("cm");
  lcd.setCursor(0, 1);
  lcd.print("Obj: ");

  switch (etat) {
    case AUTOMATIQUE:

      lcd.print(targetAngle);
      lcd.print("deg");
      break;
    case TROP_PRES:
      lcd.print("Trop pres");
      break;
    case TROP_LOIN:
      lcd.print("Trop loin");
      break;
  }
}


void activateAlarm() {
  currentTime = millis();
  static unsigned long previousledMillis = 0;
  digitalWrite(buzzer, !buzzerState);

  if (currentTime - previousledMillis >= ledInterval) {

    previousledMillis = currentTime;

    if (ledState) {
      digitalWrite(redled, !etatled);
      digitalWrite(blueled, etatled);
    } else {
      digitalWrite(redled, etatled);
      digitalWrite(blueled, !etatled);
    }
    ledState = !ledState;
  }
}

void deactivateAlarm() {
  digitalWrite(buzzer, buzzerState);

  digitalWrite(redled, etatled);
  digitalWrite(blueled, etatled);
}

void alarmState() {
  if (distance <= alarmDistance && distance > min) {

    if (!alarmActive) {
      alarmActive = true;
    }
    lastAlarmTime = millis();

  } else {

    if (alarmActive && (millis() - lastAlarmTime >= alarmEndTime)) {
      alarmActive = false;
      deactivateAlarm();
    }
  }

  if (alarmActive) {
    activateAlarm();
  }
}

void commandGestion() {

  if (!Serial.available()) return;
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();

  if (command == "g_dist") {
    Serial.print("Arduino: ");
    Serial.println(distance);
    confirm();

  } else if (command.startsWith("cfg;alm;")) {
    int valeur = command.substring(8).toInt();
    alarmDistance = valeur;

    Serial.print("Il configure la distance de détection de l'alarme à ");
      Serial.print(valeur);
    Serial.println("cm");
    confirm();
  } else if (command.startsWith("cfg;lim_inf;")) {
    int value = command.substring(12).toInt();

    if (value < MAX_ANGLE) {
      MIN_ANGLE = value;
      Serial.print("Il configure la limite inférieur du moteur à ");
        Serial.print(value);
      Serial.println("cm");
      confirm();
    } else {
      Serial.println("Erreur – Limite inférieure plus grande que limite supérieure");
      error();
    }
  } else if (command.startsWith("cfg;lim_sup;")) {
    int val = command.substring(12).toInt();

    if (val > MIN_ANGLE) {
      MAX_ANGLE = val;
      Serial.print("Il configure la limite supérieur du moteur à ");
      Serial.print(val);
      Serial.println("cm");
      confirm();
    } else error();
  } else {
    inconnu();
  }
}

void confirm() {
  unsigned long screenDelay = millis();
  while (millis() - screenDelay < time) {
    u8g2.clearBuffer();
    u8g2.drawLine(1, 5, 3, 7);
    u8g2.drawLine(3, 7, 7, 1);
    u8g2.sendBuffer();
  }
  u8g2.clear();
}

void error() {
  unsigned long screenDelay = millis();
  while (millis() - screenDelay < time) {
    u8g2.clearBuffer();
    u8g2.drawCircle(3, 3, 3);
    u8g2.drawLine(0, 0, 7, 7);
    u8g2.sendBuffer();
  }
  u8g2.clear();
}

void inconnu() {
  unsigned long screenDelay = millis();
  while (millis() - screenDelay < time) {
    u8g2.clearBuffer();
    u8g2.drawLine(7, 7, 0, 0);
    u8g2.drawLine(0, 7, 7, 0);
    u8g2.sendBuffer();
  }
  u8g2.clear();
}

void loop() {
  unsigned long currentTime = millis();
  distance = calculeDistance();
  etatSystem();
  machineEtat();
  commandGestion();
  moteur();
  myStepper.run();

  alarmState();
}