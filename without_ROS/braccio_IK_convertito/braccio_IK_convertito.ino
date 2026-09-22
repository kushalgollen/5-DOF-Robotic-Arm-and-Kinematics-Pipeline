#include <cvzone.h>

SerialData serialData(2,1);
int valsRec[2];


#include <Servo.h>
#include <math.h>

Servo girospalla, spalla, gomito, polso, presa;

void kin(double x1, double y1, double z1, double ang3);
void servosetup();

int x=0; int z=0;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  serialData.begin();
  servosetup();
  kin(0,12,12,0);
  
}
  

void loop() {

  serialData.Get(valsRec);
  

  if(valsRec[0] == 1){
    x++;
    kin(x, 12, 12+z,0);
  }
  if(valsRec[0] == 2){
    x--;
    kin(x, 12, 12+z,0);
  }
  if(valsRec[1] == 1){
    z++;
    kin(x, 12, 12+z,0);
  }
  if(valsRec[1] == 2){
    z--;
    kin(x, 12, 12+z,0);
  }


  valsRec[0]=0;
  valsRec[1]=0;
  Serial.print(x);
  Serial.println(z);
  

}

void servosetup() {
  girospalla.attach(9);
  spalla.attach(10);
  gomito.attach(11);
  polso.attach(12);
  presa.attach(13);
}


void kin(double x1, double y1, double z1, double ang3) {
  double l_1 = 12;
  double l_2 = 12;
  double l_3 = 12;

  double ang4 = round(atan(x1 / y1) * 180 / M_PI);
  double new_y = sqrt(pow(x1, 2) + pow(y1, 2));

  double y = new_y - (l_3 * cos(ang3 * M_PI / 180));
  double z = z1 - (l_3 * sin(ang3 * M_PI / 180));

  double s = (pow(z, 2) + pow(y, 2) - pow(l_1, 2) - pow(l_2, 2)) / (2 * l_1 * l_2);

  double q_2 = -round(acos(s) * 180 / M_PI);

  double q_1 = (atan(z / y)) - (atan((l_1 * sin(q_2 * M_PI / 180)) / (l_1 + l_2 * cos(q_2 * M_PI / 180))));

  q_1 = round(q_1 * 180 / M_PI);
  q_2 = q_2;
  double q_3 = round(ang3 - q_1 - q_2);  // ang3
  double ang1 = (q_1*65)/90;
  double ang2 = -(q_2);

  girospalla.write(90+ang4);
  spalla.write(ang1);
  gomito.write(ang2);
  polso.write(q_3+100);

}

