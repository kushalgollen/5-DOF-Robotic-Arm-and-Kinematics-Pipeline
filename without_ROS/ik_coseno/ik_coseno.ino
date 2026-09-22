#include <Servo.h>

// Dichiarazione dei pin dei servo
const int basePin = 9;
const int shoulderPin = 10;
const int elbowPin = 11;
const int wristPin = 12;
const int gripPin = 13;

// Dichiarazione degli oggetti servo
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripServo;

// Lunghezza delle giunture (in cm)
const float L1 = 12.0; // Lunghezza base-spalla
const float L2 = 12.0; // Lunghezza spalla-gomito
const float L3 = 12.0;  // Lunghezza gomito-polso

// Altezza della base del braccio robotico (distanza tra la base e l'asse Z)
const float baseHeight = 5.0;

// Funzione per calcolare la cinematica inversa
void inverseKinematics(float x, float y, float z) {
  // Calcolo dell'angolo di rotazione della base (theta0)
  float theta0 = atan2(y, x);

  // Calcolo della distanza orizzontale proiettata (ipotenusa del triangolo nella proiezione XY)
  float hypotenuseXY = sqrt(x * x + y * y);

  // Calcolo dell'altezza dello strumento rispetto alla base (asse Z)
  float height = z - baseHeight;

  // Calcolo della distanza tra la base e la posizione dell'asse Z (ipotenusa nel piano XZ)
  float hypotenuseXZ = sqrt(hypotenuseXY * hypotenuseXY + height * height);

  // Calcolo dell'angolo tra l'ipotenusa e l'asse Z (theta2)
  float theta2 = acos((L1 * L1 + hypotenuseXZ * hypotenuseXZ - L2 * L2) / (2 * L1 * hypotenuseXZ));

  // Calcolo dell'angolo del polso (theta3)
  float theta3 = acos((L2 * L2 + L3 * L3 - hypotenuseXZ * hypotenuseXZ) / (2 * L2 * L3));

  // Calcolo dell'angolo di gomito (theta2 + theta3)
  float elbowAngle = theta2 + theta3;

  // Conversione da radianti a gradi
  theta0 = degrees(theta0);
  theta2 = degrees(theta2);
  elbowAngle = degrees(elbowAngle);

  // Assegnamento degli angoli ai servo
  baseServo.write(theta0);
  shoulderServo.write(theta2);
  elbowServo.write(elbowAngle);
  wristServo.write(height);

  // Altri controlli e calcoli per la presa, se necessario.
}

// Funzione per muovere la presa
void moveGrip(int gripAngle) {
  gripServo.write(gripAngle);
}

void setup() {
  // Inizializzazione dei servo
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  wristServo.attach(wristPin);
  gripServo.attach(gripPin);

  // Posizione iniziale della presa (da calibrare in base alle tue specifiche)
  moveGrip(90);

  // Posizione iniziale del braccio (es. braccio allungato lungo l'asse X)
  inverseKinematics(12, 24, baseHeight + L1 + L2 + L3);
}

void loop() {
  // Puoi implementare altre logiche di movimento qui, ad esempio leggendo input da sensori o dal computer.
}
