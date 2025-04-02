
#ifndef NINA_GPIO0
  #define NINA_GPIO0 7
#endif

#include <SPI.h>
#include <WiFiS3.h>
#include <Servo.h>
#include <PID_v1.h>

// ======================
// Configurazione di rete
// ======================
char ssid[] = "Eolo_03f4ee";
char pass[] = "2b5jf3u4";

IPAddress local_IP(192, 168, 1, 50);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

// ======================
// Definizioni hardware robot
// ======================
#define Lpwm_pin 5   // PWM sinistro
#define Rpwm_pin 6   // PWM destro
int pinLB = 2;       // Direzione motore sinistro (IN1)
int pinLF = 4;       // Direzione motore sinistro (IN2)
int pinRB = 7;       // Direzione motore destro (IN3)
int pinRF = 8;       // Direzione motore destro (IN4)

#define US_TRIG A1   // Trigger ultrasuoni
#define US_ECHO A0   // Echo ultrasuoni
#define SERVO_PIN A2 // Servo per scansione
#define LEFT_IR_PIN 9
#define RIGHT_IR_PIN 10

Servo myservo;

// ======================
// Configurazione PID
// ======================
double Setpoint, Input, Output;
double Kp = 4, Ki = 0.2, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ======================
// Variabili per la dashboard web
// ======================
WiFiServer webServer(80);
String htmlResponse = "";
unsigned long lastSensorUpdate = 0;
float currentDistance = 0;
int currentLeftIR = 1;
int currentRightIR = 1;
String currentMovement = "🛑 Stopped";
int currentSpeedValue = 0;  // Velocità impostata (media)

// ======================
// Costanti per la navigazione avanzata
// ======================
#define MIN_FRONT_DISTANCE 25.0
#define CRITICAL_FRONT_DISTANCE 15.0
#define SCAN_POSITIONS 5   // Posizioni: 180, 135, 90, 45, 0
#define SCAN_DELAY 150
#define NORMAL_SPEED 70
#define TURNING_SPEED 100
#define ESCAPE_SPEED 120
#define SLOW_SPEED 50

// ======================
// Variabili globali per la navigazione
// ======================
float distances[SCAN_POSITIONS];  // Distanze scansionate
int leftIR, rightIR;
int movePattern = 0;
unsigned long lastObstacleTime = 0;
bool isTrapped = false;
int obstacleCounter = 0;
int consecutiveTurns = 0;
bool lastTurnDirection = false;

// Modalità di controllo: true = manuale, false = autonoma
bool manualMode = true;  // **Default: modalità manuale**

#define MANUAL_COMMAND_TIMEOUT 300
unsigned long lastManualCommandTime = 0;

// Struttura per la storia dei movimenti
#define HISTORY_SIZE 10
struct {
  int movements[HISTORY_SIZE];    // 0=avanti, 1=indietro, 2=sinistra, 3=destra
  float distances[HISTORY_SIZE];  // Distanze frontali
  int index;
} movementHistory;

// ======================
// Prototipi di funzioni
// ======================
float getUltrasonicDistance();
void scanEnvironment();
bool findBestDirection();
void setMotorSpeeds(unsigned char leftSpeed, unsigned char rightSpeed);
void moveForward(unsigned char speed);
void moveBackward(unsigned char speed);
void turnLeft(unsigned char speed, float turnRatio = 0.6);
void turnRight(unsigned char speed, float turnRatio = 0.6);
void rotateLeft(unsigned char speed);
void rotateRight(unsigned char speed);
void stopMotors();
void updateMovementHistory(int movementType, float distance);
bool checkIfTrapped();
void executeEscapeManeuver();
void adaptiveNavigation(float frontDistance);
void handleWebClient();
void sendHTML(WiFiClient &client);
void sendJSON(WiFiClient &client);
void processCommand(String request);
void sendCommandResponse(WiFiClient &client);
String getMovementStatus();

// ======================
// setup()
// ======================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Impostazione pin motori
  pinMode(pinLB, OUTPUT);
  pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinRF, OUTPUT);
  pinMode(Lpwm_pin, OUTPUT);
  pinMode(Rpwm_pin, OUTPUT);
  
  // Impostazione sensori
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
  
  // Inizializzazione servo
  myservo.attach(SERVO_PIN);
  myservo.write(90);
  
  // Inizializzazione storia movimenti
  for (int i = 0; i < HISTORY_SIZE; i++) {
    movementHistory.movements[i] = 0;
    movementHistory.distances[i] = 100.0;
  }
  movementHistory.index = 0;
  
  // Configurazione PID
  Setpoint = MIN_FRONT_DISTANCE + 10.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-50, 50);
  
  // Configurazione WiFi e debug via Serial
  WiFi.config(local_IP, dns, gateway, subnet);
  int status = WL_IDLE_STATUS;
  Serial.println("Connessione alla rete WiFi in corso...");
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  Serial.println("WiFi connesso!");
  Serial.print("Indirizzo IP: ");
  Serial.println(WiFi.localIP());
  
  // Avvio del server web e debug
  webServer.begin();
  Serial.println("Server web avviato e disponibile sulla porta 80");
  
  // Preparazione HTML per dashboard
  htmlResponse = 
    "<!DOCTYPE html><html><head>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<title>Robot Monitor</title>"
    "<style>"
    "body {font-family: Arial; text-align: center; background: #f0f0f0;}"
    ".dashboard {display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px; margin: 20px;}"
    ".card {background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 5px rgba(0,0,0,0.1);} "
    "h1 {color: #2c3e50;} "
    ".data-value {font-size: 24px; color: #3498db; margin: 10px 0;} "
    ".status {padding: 8px; border-radius: 5px; color: white;} "
    ".online {background: #2ecc71;} "
    ".offline {background: #e74c3c;} "
    "</style></head><body>"
    "<h1>🤖 Robot Monitoring Dashboard</h1>"
    "<div class='dashboard'>"
    "<div class='card'><h2>🚦 Movement</h2><div id='movement' class='data-value'>-</div></div>"
    "<div class='card'><h2>📏 Distance</h2><div id='distance' class='data-value'>- cm</div></div>"
    "<div class='card'><h2>🔴 IR Left</h2><div id='irLeft' class='status'>-</div></div>"
    "<div class='card'><h2>🔵 IR Right</h2><div id='irRight' class='status'>-</div></div>"
    "<div class='card'><h2>🚀 Speed</h2><div id='speed' class='data-value'>- %</div></div>"
    "<div class='card'><h2>📶 WiFi</h2><div id='wifi' class='data-value'>- dBm</div></div>"
    "</div>"
    "<script>"
    "let lastUpdate = 0;"
    "async function updateData() {"
    "  try {"
    "    const res = await fetch('/data?t=' + Date.now());"
    "    const data = await res.json();"
    "    if(data.timestamp > lastUpdate) {"
    "      lastUpdate = data.timestamp;"
    "      document.getElementById('distance').textContent = data.distance.toFixed(1) + ' cm';"
    "      document.getElementById('irLeft').textContent = data.irLeft ? '⚠️ Obstacle' : '✅ Clear';"
    "      document.getElementById('irLeft').className = `status ${data.irLeft ? 'offline' : 'online'}`;"
    "      document.getElementById('irRight').textContent = data.irRight ? '⚠️ Obstacle' : '✅ Clear';"
    "      document.getElementById('irRight').className = `status ${data.irRight ? 'offline' : 'online'}`;"
    "      document.getElementById('movement').textContent = data.movement;"
    "      document.getElementById('speed').textContent = data.speed + ' %';"
    "      document.getElementById('wifi').textContent = data.wifiStrength + ' dBm';"
    "    }"
    "  } catch(e) { console.error(e); }"
    "  setTimeout(updateData, 100);"
    "}"
    "updateData();"
    "</script></body></html>";
  
  // Scansione iniziale e inizio movimento in avanti (modalità autonoma)
  scanEnvironment();
  moveForward(NORMAL_SPEED);
}

// ======================
// loop()
// ======================
void loop() {
  // Aggiornamento dei sensori ogni 50ms
  if (millis() - lastSensorUpdate >= 50) {
    float frontDistance = getUltrasonicDistance();
    leftIR = digitalRead(LEFT_IR_PIN);
    rightIR = digitalRead(RIGHT_IR_PIN);
    currentDistance = frontDistance;
    currentLeftIR = leftIR;
    currentRightIR = rightIR;
    currentMovement = getMovementStatus();
    lastSensorUpdate = millis();
  }
  
  // Modalità autonoma
  if (!manualMode) {
    float frontDistance = getUltrasonicDistance();
    updateMovementHistory(0, frontDistance);  // 0 = avanti
    
    if(obstacleCounter > 5 && checkIfTrapped()){
      isTrapped = true;
    }
    
    if(isTrapped) {
      executeEscapeManeuver();
      isTrapped = false;
      obstacleCounter = 0;
    }
    else {
      bool obstacleDetected = false;
      if(frontDistance < MIN_FRONT_DISTANCE) {
        obstacleDetected = true;
        obstacleCounter++;
        Serial.print("Ostacolo frontale: ");
        Serial.println(frontDistance);
      }
      if(leftIR == 0 || rightIR == 0) {
        obstacleDetected = true;
        obstacleCounter++;
        if(leftIR == 0) { Serial.println("Ostacolo laterale sinistro"); }
        if(rightIR == 0) { Serial.println("Ostacolo laterale destro"); }
      }
      
      if(!obstacleDetected) {
        obstacleCounter = 0;
        moveForward(NORMAL_SPEED);
        adaptiveNavigation(frontDistance);
      }
      else {
        stopMotors();
        delay(100);
        bool needBackup = false;
        if(frontDistance < CRITICAL_FRONT_DISTANCE || (leftIR == 0 && rightIR == 0)) {
          needBackup = true;
        }
        if(needBackup) {
          updateMovementHistory(1, frontDistance);  // 1 = retromarcia
          moveBackward(TURNING_SPEED);
          delay(600);
          stopMotors();
          delay(100);
        }
        scanEnvironment();
        if(leftIR == 0 && rightIR == 1) {
          turnRight(TURNING_SPEED, 0.3);
          updateMovementHistory(3, frontDistance);  // 3 = destra
          delay(300 + frontDistance * 5);
        }
        else if(rightIR == 0 && leftIR == 1) {
          turnLeft(TURNING_SPEED, 0.3);
          updateMovementHistory(2, frontDistance);  // 2 = sinistra
          delay(300 + frontDistance * 5);
        }
        else {
          bool turnDir = findBestDirection();
          if(turnDir) {
            if(distances[3] < 30 && distances[4] < 30) {
              rotateRight(TURNING_SPEED);
              delay(500);
            } else {
              turnRight(TURNING_SPEED);
              delay(300 + (30 - min(30.0, frontDistance)) * 20);
            }
            updateMovementHistory(3, frontDistance);
          } else {
            if(distances[0] < 30 && distances[1] < 30) {
              rotateLeft(TURNING_SPEED);
              delay(500);
            } else {
              turnLeft(TURNING_SPEED);
              delay(300 + (30 - min(30.0, frontDistance)) * 20);
            }
            updateMovementHistory(2, frontDistance);
          }
        }
        stopMotors();
        delay(50);
      }
    }
  }
  // Modalità manuale: se non riceve comandi per più di MANUAL_COMMAND_TIMEOUT ms, ferma i motori
  else {
    if (millis() - lastManualCommandTime > MANUAL_COMMAND_TIMEOUT) {
      stopMotors();
    }
  }
  
  // Gestione client web (dati e comandi)
  handleWebClient();
}

// ======================
// Funzioni di supporto
// ======================
float getUltrasonicDistance() {
  float samples[3];
  for (int i = 0; i < 3; i++) {
    digitalWrite(US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG, LOW);
    float duration = pulseIn(US_ECHO, HIGH, 23500);
    if(duration == 0) {
      samples[i] = 400.0;
    } else {
      samples[i] = duration / 58.0;
    }
    delay(10);
  }
  if(samples[0] > samples[1]) { float temp = samples[0]; samples[0] = samples[1]; samples[1] = temp; }
  if(samples[1] > samples[2]) { float temp = samples[1]; samples[1] = samples[2]; samples[2] = temp; }
  if(samples[0] > samples[1]) { float temp = samples[0]; samples[0] = samples[1]; samples[1] = temp; }
  return samples[1];
}

void scanEnvironment() {
  int angles[SCAN_POSITIONS] = {180, 135, 90, 45, 0};
  for (int i = 0; i < SCAN_POSITIONS; i++) {
    myservo.write(angles[i]);
    delay(SCAN_DELAY);
    distances[i] = getUltrasonicDistance();
    Serial.print("Angolo ");
    Serial.print(angles[i]);
    Serial.print(": ");
    Serial.println(distances[i]);
  }
  myservo.write(90);
  delay(SCAN_DELAY);
}

bool findBestDirection() {
  float leftScore = (distances[0] * 0.7) + (distances[1] * 0.3);
  float rightScore = (distances[3] * 0.3) + (distances[4] * 0.7);
  
  if(consecutiveTurns > 2) {
    if(lastTurnDirection) { 
      leftScore *= 1.3;
    } else {
      rightScore *= 1.3;
    }
  }
  
  Serial.print("Punteggio sinistra: ");
  Serial.print(leftScore);
  Serial.print("  Punteggio destra: ");
  Serial.println(rightScore);
  
  return (rightScore > leftScore);
}

void setMotorSpeeds(unsigned char leftSpeed, unsigned char rightSpeed) {
  analogWrite(Lpwm_pin, leftSpeed);
  analogWrite(Rpwm_pin, rightSpeed);
  currentSpeedValue = (leftSpeed + rightSpeed) / 2;
}

void moveForward(unsigned char speed) {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  setMotorSpeeds(speed, speed);
}

void moveBackward(unsigned char speed) {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  setMotorSpeeds(speed, speed);
}

void turnLeft(unsigned char speed, float turnRatio) {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, LOW);
  
  unsigned char rightSpeed = speed;
  unsigned char leftSpeed = 0;
  if(turnRatio > 0) {
    digitalWrite(pinLB, HIGH);
    digitalWrite(pinLF, LOW);
    leftSpeed = speed * turnRatio;
  }
  setMotorSpeeds(leftSpeed, rightSpeed);
  lastTurnDirection = false;
  consecutiveTurns = (lastTurnDirection == false) ? consecutiveTurns + 1 : 1;
}

void turnRight(unsigned char speed, float turnRatio) {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  
  unsigned char leftSpeed = speed;
  unsigned char rightSpeed = 0;
  if(turnRatio > 0) {
    digitalWrite(pinRB, LOW);
    digitalWrite(pinRF, HIGH);
    rightSpeed = speed * turnRatio;
  }
  setMotorSpeeds(leftSpeed, rightSpeed);
  lastTurnDirection = true;
  consecutiveTurns = (lastTurnDirection == true) ? consecutiveTurns + 1 : 1;
}

void rotateLeft(unsigned char speed) {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  setMotorSpeeds(speed, speed);
  lastTurnDirection = false;
}

void rotateRight(unsigned char speed) {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  setMotorSpeeds(speed, speed);
  lastTurnDirection = true;
}

void stopMotors() {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, LOW);
  setMotorSpeeds(0, 0);
}

void updateMovementHistory(int movementType, float distance) {
  movementHistory.movements[movementHistory.index] = movementType;
  movementHistory.distances[movementHistory.index] = distance;
  movementHistory.index = (movementHistory.index + 1) % HISTORY_SIZE;
}

bool checkIfTrapped() {
  int forwardCount = 0;
  int backwardCount = 0;
  int leftCount = 0;
  int rightCount = 0;
  
  for (int i = 0; i < HISTORY_SIZE; i++) {
    switch(movementHistory.movements[i]) {
      case 0: forwardCount++; break;
      case 1: backwardCount++; break;
      case 2: leftCount++; break;
      case 3: rightCount++; break;
    }
  }
  
  if(backwardCount > 3 && (leftCount + rightCount) > 5) {
    return true;
  }
  
  float sumDistances = 0;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    sumDistances += movementHistory.distances[i];
  }
  float avgDistance = sumDistances / HISTORY_SIZE;
  
  return (avgDistance < MIN_FRONT_DISTANCE && backwardCount > 2);
}

void executeEscapeManeuver() {
  Serial.println("Esecuzione manovra di fuga!");
  moveBackward(ESCAPE_SPEED);
  delay(800);
  stopMotors();
  delay(200);
  scanEnvironment();
  if(findBestDirection()) {
    rotateRight(TURNING_SPEED);
  } else {
    rotateLeft(TURNING_SPEED);
  }
  delay(800);
  obstacleCounter = 0;
  isTrapped = false;
  consecutiveTurns = 0;
}

void adaptiveNavigation(float frontDistance) {
  Input = frontDistance;
  Setpoint = MIN_FRONT_DISTANCE + 10.0;
  myPID.Compute();
  
  int adaptiveSpeed;
  if(frontDistance < MIN_FRONT_DISTANCE) {
    adaptiveSpeed = map(frontDistance, CRITICAL_FRONT_DISTANCE, MIN_FRONT_DISTANCE, SLOW_SPEED, NORMAL_SPEED);
    adaptiveSpeed = constrain(adaptiveSpeed, SLOW_SPEED, NORMAL_SPEED);
  } else {
    adaptiveSpeed = map(frontDistance, MIN_FRONT_DISTANCE, MIN_FRONT_DISTANCE * 4, NORMAL_SPEED, NORMAL_SPEED + 30);
    adaptiveSpeed = constrain(adaptiveSpeed, NORMAL_SPEED, NORMAL_SPEED + 30);
  }
  
  if(Output > 10) {
    float ratio = map(Output, 10, 50, 0, 30) / 100.0;
    setMotorSpeeds(adaptiveSpeed, adaptiveSpeed * (1.0 - ratio));
  }
  else if(Output < -10) {
    float ratio = map(-Output, 10, 50, 0, 30) / 100.0;
    setMotorSpeeds(adaptiveSpeed * (1.0 - ratio), adaptiveSpeed);
  }
  else {
    setMotorSpeeds(adaptiveSpeed, adaptiveSpeed);
  }
}

// ======================
// Gestione client web
// ======================
void handleWebClient() {
  WiFiClient client = webServer.available();
  if (client) {
    Serial.println("Client connesso");
    String request = client.readStringUntil('\r');
    Serial.print("Richiesta ricevuta: ");
    Serial.println(request);
    client.flush();

    if (request.indexOf("GET /data") != -1) {
      sendJSON(client);
      Serial.println("Inviato JSON");
    }
    else if (request.indexOf("GET /command") != -1) {
      processCommand(request);
      sendCommandResponse(client);
      Serial.println("Comando processato");
    }
    else {
      sendHTML(client);
      Serial.println("Inviata pagina HTML");
    }
    
    client.stop();
    Serial.println("Client disconnesso\n");
  }
}

void sendHTML(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println(htmlResponse);
}

void sendJSON(WiFiClient &client) {
  String json = "{";
  json += "\"timestamp\":" + String(millis()) + ",";
  json += "\"distance\":" + String(currentDistance, 1) + ",";
  json += "\"irLeft\":" + String(currentLeftIR) + ",";
  json += "\"irRight\":" + String(currentRightIR) + ",";
  json += "\"movement\":\"" + currentMovement + "\",";
  json += "\"speed\":" + String(currentSpeedValue) + ",";
  json += "\"wifiStrength\":" + String(WiFi.RSSI());
  json += "}";
  
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Cache-Control: no-cache, no-store, must-revalidate");
  client.println("Pragma: no-cache");
  client.println("Expires: 0");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();
  client.println(json);
}

// Funzione per processare i comandi manuali e il cambio modalità
void processCommand(String request) {
  // Cambio modalità
  if (request.indexOf("mode=manual") != -1) {
    manualMode = true;
    Serial.println("Modalità manuale attivata");
    lastManualCommandTime = millis(); // Reset del timer manuale
  } 
  else if (request.indexOf("mode=autonomous") != -1) {
    manualMode = false;
    Serial.println("Modalità autonoma attivata");
    scanEnvironment();
  }
  
  // Se siamo in modalità manuale, processa i comandi di movimento
  if (manualMode) {
    // Aggiorna il timer ad ogni comando manuale
    lastManualCommandTime = millis();
    if (request.indexOf("cmd=forward") != -1) {
      moveForward(NORMAL_SPEED);
      Serial.println("Comando: Avanti");
    } 
    else if (request.indexOf("cmd=backward") != -1) {
      moveBackward(TURNING_SPEED);
      Serial.println("Comando: Indietro");
    } 
    else if (request.indexOf("cmd=left") != -1) {
      turnLeft(TURNING_SPEED, 0.3);
      Serial.println("Comando: Sinistra");
    } 
    else if (request.indexOf("cmd=right") != -1) {
      turnRight(TURNING_SPEED, 0.3);
      Serial.println("Comando: Destra");
    } 
    else if (request.indexOf("cmd=stop") != -1) {
      stopMotors();
      Serial.println("Comando: Stop");
    }
  }
}

void sendCommandResponse(WiFiClient &client) {
  String json = "{\"status\":\"ok\"}";
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();
  client.println(json);
}

String getMovementStatus() {
  if(digitalRead(pinLF) == HIGH) return "⬆️ Forward";
  if(digitalRead(pinRF) == HIGH) return "⬇️ Backward";
  if(digitalRead(pinLB) == HIGH) return "⬅️ Left Turn";
  if(digitalRead(pinRB) == HIGH) return "➡️ Right Turn";
  return "🛑 Stopped";
}