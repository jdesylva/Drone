#include <Wire.h>

#define DELROUGE          A3
#define DELVERTE          A2
#define DELVERTE_ON       0x01
#define DELVERTE_OFF      0x00
#define DELVERTE_MASK     0x01
#define DEL_VERTE_DELAI   0x40

#define ACCELADD          0x4C
#define ACCELWRTADD       0x98
#define ACCELREADADD      0x99

#define ACCEL_XOUT        0x00
#define ACCEL_YOUT        0x01
#define ACCEL_ZOUT        0x02
#define ACCEL_TILT        0x03
#define ACCEL_SRST        0x04  // Sample rate Status
#define ACCEL_SPCNT       0x05  // Sleep Count
#define ACCEL_INTSU       0x06  // Interrupt Setup
#define ACCEL_MODE        0x07
#define ACCEL_SR          0x08  // Sample rate
#define ACCEL_PDET        0x09  // Tap Detection
#define ACCEL_PD          0x0A  // Tap debounced count

#define GYROADD           0x6A

#define GYRO_WHO_AM_I     0x0F
//#define GYRO_WHO_AM_I     0x1E
#define GYRO_CTRL1        0x20
#define GYRO_CTRL2        0x21
#define GYRO_CTRL3        0x22
#define GYRO_CTRL4        0x23
#define GYRO_CTRL5        0x24
#define GYRO_REF          0x25
#define GYRO_OUT_TEMP     0x26
#define GYRO_STATUS       0x27
#define GYRO_OUT_X_L      0x28
#define GYRO_OUT_X_H      0x29
#define GYRO_OUT_Y_L      0x2A
#define GYRO_OUT_Y_H      0x2B
#define GYRO_OUT_Z_L      0x2C
#define GYRO_OUT_Z_H      0x2D
#define GYRO_FIFO_CTRL    0x2E
#define GYRO_FIFO_SRC     0x2F
#define GYRO_IG_CFG       0x30
#define GYRO_IG_SRC       0x31
#define GYRO_IG_THS_XH    0x32
#define GYRO_IG_THS_XL    0x33
#define GYRO_IG_THS_YH    0x34
#define GYRO_IG_THS_YL    0x35
#define GYRO_IG_THS_ZH    0x36
#define GYRO_IG_THS_ZL    0x37
#define GYRO_IG_DURATION  0x38
#define GYRO_LOW_ODR      0x39

#define MOTEUR_1          10
#define MOTEUR_2          11
#define MOTEUR_3          5
#define MOTEUR_4          13

#define OFFSET_XY_MAX     31
#define OFFSET_XY_MIN     -32
#define VITESSE           20  // 20 si on debug avec des DELs; 120 avec les moteurs
#define FACTEUR_ECHELLE   1    // 1 si on debug avec des DELs; 3 avec les moteurs

#define THRESHOLD_XY      3
#define THRESHOLD_Z       19   // 1g == 21.33

#define PWM_MAX           255

ISR(TIMER0_COMPA_vect)
{
  static boolean state = false;
  static int intCount = 0;



  if (++intCount % 500 == 0) {
    state = !state;  // toggle
    digitalWrite (DELROUGE, state ? HIGH : LOW);
    intCount = 0;
  }
}


class DroneError {

  public:
    void showError(unsigned int errNo, unsigned int del) {

      while (errNo--) {
        digitalWrite(del, HIGH);   // turn the LED on (HIGH is the 3.3V voltage level)
        delay(250);
        digitalWrite(del, LOW);   // turn the LED off (LOW is the 0V voltage level)
        delay(250);
      }
    }

    void showFastError(unsigned char errNo, unsigned char del) {

      while (errNo--) {
        digitalWrite(del, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(100);
        digitalWrite(del, LOW);   // turn the LED on (HIGH is the voltage level)
        delay(100);
      }
    }
};

/////////////////////////////////////////////////////////////////

class Gyroscope {
  private:
    unsigned char errorCode = 0;
    DroneError *mPtrError;

  public:
    Gyroscope(DroneError*);
    unsigned char getID();

};

Gyroscope::Gyroscope(DroneError *err) {
  this->mPtrError = err;
}

unsigned char Gyroscope::getID() {

  unsigned char returnValue = 0;

  // activer l'accéléromètre
  Wire.beginTransmission(GYROADD);
  Wire.write(GYRO_WHO_AM_I);
  errorCode = Wire.endTransmission(false);       // repeated start transmitting
  delay(10);
  this->mPtrError->showError(errorCode, DELROUGE);
  errorCode = 0;

  Wire.requestFrom(GYROADD, 1, false); // récupérer le ID
  delay(10);

  if (Wire.available())
  {
    returnValue = Wire.read();             // lire le ID (0xD7)
    if (0xD7 == returnValue) {
      this->mPtrError->showError(1, DELVERTE);
    } else {
      this->mPtrError->showFastError(returnValue, DELROUGE);
    }
    delay(250);
  } else {
    this->mPtrError->showError(3, DELROUGE);
  }

  return returnValue;

}

/////////////////////////////////////////////////////////////////

class Accelerometre {
  private:
    unsigned char errorCode = 0;
    signed char x[3] = {0, 0, 0}; // x[0] est le dernier échantillon.
    signed char y[3] = {0, 0, 0};
    signed char z[3] = {0, 0, 0};
    DroneError *mPtrError;

  public:

    Accelerometre(DroneError*);

    void process();

    signed char getCorrectedAccel(signed char accel);

    signed char getX() {
      return (x[0] >> 1) + (x[1] >> 2) + (x[2] >> 2);
    }
    signed char getY() {
      return (y[0] >> 1) + (y[1] >> 2) + (y[2] >> 2);
    }
    signed char getZ() {
      return (z[0] >> 1) + (z[1] >> 2) + (z[2] >> 2);
    }

};

Accelerometre::Accelerometre(DroneError *err) {
  this->mPtrError = err;
}

signed char Accelerometre::getCorrectedAccel(signed char accel) {

  if (accel & 0x20) accel |= 0xC0;
  else accel &= 0x1F;

  return accel;

}

//void Accelerometre::process() {
//
//  signed char old_x = x; old_x = x;
//  signed char old_y = y; old_y = y;
//  signed char old_z = z; old_z = z;
//
//  // lire l'axe "X" de l'accéléromètre
//  Wire.beginTransmission(ACCELADD);
//
//  Wire.write(ACCEL_XOUT);
//  x = Wire.endTransmission(false);       // repeated start transmitting
//  delayMicroseconds(2);
//  mPtrError->showError(x, DELROUGE);
//
//  Wire.requestFrom(ACCELADD, 1, false); // récupérer l'accélaration en "X"
//
//  if (Wire.available())
//  {
//    x = Wire.read();             // lire l'accélération sur l'axe des "x"
//    if (x & 0x40) {              // Si occupé, conserver la valeur précédente
//      x = old_x;
//    } else {
//      x = getCorrectedAccel(x);
//    }
//
//  } else {
//    mPtrError->showError(3, DELROUGE);
//  }
//
//  // lire l'axe "Y" de l'accéléromètre
//  Wire.beginTransmission(ACCELADD);
//  Wir
//  } else {
//    mPtrError->showError(3, DELROUGE);
//  }
//
//  // lire l'axe "Z" de l'accéléromètre
//  Wire.beginTransmission(ACCELADD);
//  Wire.write(ACCEL_ZOUT);
//  Wire.endTransmission(false);       // repeated start transmitting
//  delayMicroseconds(2);
//
//  Wire.requestFrom(ACCELADD, 1, false); // récupérer l'accélaration en "Z"
//
//  if (Wire.available())
//  {
//    z = Wire.read();             // lire l'accélération sur l'axe des "z"
//    if (z & 0x40) {              // Si occupé, conserver la valeur précédente
//      z = old_z;
//    } else {
//      z = getCorrectedAccel(z);
//    }
//  } else {
//    mPtrError->showError(3, DELROUGE);
//  }
//}

void Accelerometre::process() {

  signed char xx = 0;
  signed char yy = 0;
  signed char zz = 0;

  x[2] = x[1];
  x[1] = x[0];

  // lire l'axe "X" de l'accéléromètre
  Wire.beginTransmission(ACCELADD);

  Wire.write(ACCEL_XOUT);
  xx = Wire.endTransmission(false);       // repeated start transmitting
  delayMicroseconds(2);
  mPtrError->showError(xx, DELROUGE);

  Wire.requestFrom(ACCELADD, 1, false); // récupérer l'accélaration en "X"

  if (Wire.available())
  {
    xx = Wire.read();             // lire l'accélération sur l'axe des "x"
    if (xx & 0x40) {              // Si occupé, conserver la valeur précédente
      x[0] = x[1];
    } else {
      x[0] = getCorrectedAccel(xx);
    }

  } else {
    mPtrError->showError(3, DELROUGE);
  }

  y[2] = y[1];
  y[1] = y[0];

  // lire l'axe "Y" de l'accéléromètre
  Wire.beginTransmission(ACCELADD);
  Wire.write(ACCEL_YOUT);
  Wire.endTransmission(false);       // repeated start transmitting
  delayMicroseconds(2);

  Wire.requestFrom(ACCELADD, 1, false); // récupérer l'accélaration en "Y"

  if (Wire.available())
  {
    yy = Wire.read();             // lire l'accélération sur l'axe des "y"
    if (yy & 0x40) {              // Si occupé, conserver la valeur précédente
      y[0] = y[1];
    } else {
      y[0] = getCorrectedAccel(yy);
    }

  } else {
    mPtrError->showError(3, DELROUGE);
  }

  z[2] = z[1];
  z[1] = z[0];

  // lire l'axe "Z" de l'accéléromètre
  Wire.beginTransmission(ACCELADD);
  Wire.write(ACCEL_ZOUT);
  Wire.endTransmission(false);       // repeated start transmitting
  delayMicroseconds(2);

  Wire.requestFrom(ACCELADD, 1, false); // récupérer l'accélaration en "Z"

  if (Wire.available())
  {
    zz = Wire.read();             // lire l'accélération sur l'axe des "z"
    if (zz & 0x40) {              // Si occupé, conserver la valeur précédente
      z[0] = z[1];
    } else {
      z[0] = getCorrectedAccel(zz);
    }
  } else {
    mPtrError->showError(3, DELROUGE);
  }
}

/////////////////////////////////////////////////////////////////

class Moteur {
  private:
    unsigned char errorCode = 0;
    unsigned char mES = 0;        // Identifie la broche du moteur contrôlé par cet objet.
    DroneError *mPtrError;

  public:
    Moteur(DroneError*, unsigned char ES);
    void setPWM(int valeur);

};

Moteur::Moteur(DroneError *err, unsigned char ES) {
  mPtrError = err;
  mES = ES;
  pinMode(mES, OUTPUT);

}

void Moteur::setPWM(int valeur) {

  valeur = (valeur < 0) ? 0 : valeur;
  valeur = (valeur > PWM_MAX) ? PWM_MAX : valeur;

  analogWrite(mES, (unsigned char)valeur);

}

/////////////////////////////////////////////////////////////////

class Drone {
  private:

    unsigned char vitesse = VITESSE;
    unsigned char orientation = 0;

    int v1, v2, v3, v4;

    signed char offsetX = 0;  // Lecture de l'accéléromètre en "X"
    signed char offsetY = 0;  // Lecture de l'accéléromètre en "Y"

    Moteur *mPtrMoteur1 = NULL;
    Moteur *mPtrMoteur2 = NULL;
    Moteur *mPtrMoteur3 = NULL;
    Moteur *mPtrMoteur4 = NULL;

    Gyroscope *mPtrGyroscope = NULL;
    Accelerometre *mPtrAccelerometre = NULL;
    DroneError *mPtrError = NULL;

  public:
    Drone();
    void vole();
    void test();
    void mayday();
    void showError(unsigned int errNo, unsigned int del);
};

Drone::Drone() {

  mPtrMoteur1 = new Moteur(mPtrError, MOTEUR_1);
  mPtrMoteur2 = new Moteur(mPtrError, MOTEUR_2);
  mPtrMoteur3 = new Moteur(mPtrError, MOTEUR_3);
  mPtrMoteur4 = new Moteur(mPtrError, MOTEUR_4);

  mPtrError = new DroneError();
  mPtrGyroscope = new Gyroscope(mPtrError);
  mPtrAccelerometre = new Accelerometre(mPtrError);

}

void Drone::showError(unsigned int errNo, unsigned int del) {
  mPtrError->showError(errNo, del);
}

void Drone::vole() {

  unsigned char delai, etat_delverte = 0;
  signed char x, y, z;
  char *buffer = "x = \0              ";
  char *buffer1 = "              ";

  v1 = v2 = v3 = v4 = vitesse;

  mPtrMoteur1-> setPWM(v1);
  mPtrMoteur2-> setPWM(v2);
  mPtrMoteur3-> setPWM(v3);
  mPtrMoteur4-> setPWM(v4);


  while (1) {

    if (++delai >= DEL_VERTE_DELAI) {
      digitalWrite(DELVERTE, etat_delverte ^= DELVERTE_MASK);   // blink the Green LED
      delai = 0;
    }

    // mPtrGyroscope->getID();
    mPtrAccelerometre->process();

    // Ici on ne veut pas changer la vitesse, cette variable est contrôlée ailleur.
    // On veut seulement compenser pour garder le drone de niveau horizontalement (x et y == 0)
    x = mPtrAccelerometre->getX();
    y = mPtrAccelerometre->getY();
    z = mPtrAccelerometre->getZ();

    offsetX = x < OFFSET_XY_MAX ? x : OFFSET_XY_MAX;
    offsetX = offsetX > OFFSET_XY_MIN ? offsetX : OFFSET_XY_MIN;
    offsetY = y < OFFSET_XY_MAX ? y : OFFSET_XY_MAX;
    offsetY = offsetY > OFFSET_XY_MIN ? offsetY : OFFSET_XY_MIN;
    //    offsetX = x;
    //    offsetY = y;

    strcpy(buffer, "x=");
    *buffer1 = 0;
    Serial1.print(strcat(buffer, itoa(x, buffer1, 10)));
    strcpy(buffer, "y=");
    *buffer1 = 0;
    Serial1.print(strcat(buffer, itoa(y, buffer1, 10)));
    strcpy(buffer, "z=");
    *buffer1 = 0;
    Serial1.println(strcat(buffer, itoa(z, buffer1, 10)));

    //    strcpy(buffer, "offsetX=");
    //    *buffer1 = 0;
    //    Serial1.print(strcat(buffer, itoa(offsetX, buffer1, 10)));
    //    strcpy(buffer, "offsetY=");
    //    *buffer1 = 0;
    //    Serial1.println(strcat(buffer, itoa(offsetY, buffer1, 10)));

    Serial1.flush();
    delay(10);

    if ((z > -THRESHOLD_Z) && (z < -3)) { // Z varie entre -32 et +31; Drone inversé;
      if ((offsetX > THRESHOLD_XY) || (offsetX < -THRESHOLD_XY)) {   // Si on doit ajuster la vitesse en "X"
        if ((offsetY > THRESHOLD_XY) || (offsetY < -THRESHOLD_XY)) { // Si on doit ajuster la vitesse en "Y"
          v1 = vitesse - (FACTEUR_ECHELLE * offsetX) + (FACTEUR_ECHELLE * offsetY); (v1 < 0) ? 0 : v1; v1 > PWM_MAX ? PWM_MAX : v1;
          mPtrMoteur1-> setPWM(v1);
          v2 = vitesse - (FACTEUR_ECHELLE * offsetX) - (FACTEUR_ECHELLE * offsetY); (v2 < 0) ? 0 : v2; v2 > PWM_MAX ? PWM_MAX : v2;
          mPtrMoteur2-> setPWM(v2);
          v3 = vitesse + (FACTEUR_ECHELLE * offsetX) - (FACTEUR_ECHELLE * offsetY); (v3 < 0) ? 0 : v3; v3 > PWM_MAX ? PWM_MAX : v3;
          mPtrMoteur3-> setPWM(v3);
          v4 = vitesse + (FACTEUR_ECHELLE * offsetX) + (FACTEUR_ECHELLE * offsetY); (v4 < 0) ? 0 : v4; v4 > PWM_MAX ? PWM_MAX : v4;
          mPtrMoteur4-> setPWM(v4);
          // digitalWrite(DELVERTE, etat_delverte ^= DELVERTE_MASK);    // blink the Green LED
        } else {                                                     // Si on doit ajuster SEULEMENT la vitesse en "X"
          v1 = vitesse - (FACTEUR_ECHELLE * offsetX); (v1 < 0) ? 0 : v1; v1 > PWM_MAX ? PWM_MAX : v1;
          mPtrMoteur1-> setPWM(v1);
          mPtrMoteur2-> setPWM(v1);
          v3 = vitesse + (FACTEUR_ECHELLE * offsetX); (v3 < 0) ? 0 : v3; v3 > PWM_MAX ? PWM_MAX : v3;
          mPtrMoteur3-> setPWM(v3);
          mPtrMoteur4-> setPWM(v3);
          // digitalWrite(DELVERTE, etat_delverte ^= DELVERTE_MASK);    // blink the Green LED
        }
      } else if ((offsetY > THRESHOLD_XY) || (offsetY < -THRESHOLD_XY)) {
        mPtrMoteur1-> setPWM(vitesse + (FACTEUR_ECHELLE * offsetY) );                    // Si on doit ajuster SEULEMENT la vitesse en "Y"
        mPtrMoteur2-> setPWM(vitesse - (FACTEUR_ECHELLE * offsetY) );
        mPtrMoteur3-> setPWM(vitesse - (FACTEUR_ECHELLE * offsetY) );
        mPtrMoteur4-> setPWM(vitesse + (FACTEUR_ECHELLE * offsetY) );
      }
    } else if (z > 10) {
      mayday();
    }
  }
}


void Drone::test() {

  unsigned char delai, etat_delverte = 0;
  signed char x, y, z;

  v1 = v2 = v3 = v4 = VITESSE;

  mPtrMoteur1-> setPWM(v1);
  mPtrMoteur2-> setPWM(v2);
  mPtrMoteur3-> setPWM(v3);
  mPtrMoteur4-> setPWM(v4);

  while (1) {

    if (++delai >= DEL_VERTE_DELAI) {
      digitalWrite(DELVERTE, etat_delverte ^= DELVERTE_MASK);   // blink the Green LED
      delai = 0;
    }

    // mPtrGyroscope->getID();
    mPtrAccelerometre->process();

    x = mPtrAccelerometre->getX();   // X varie entre -32 et +31
    y = mPtrAccelerometre->getY();   // Y varie entre -32 et +31
    z = mPtrAccelerometre->getZ();   // Z varie entre -32 et +31

    if ((z < THRESHOLD_Z) && (z > 0)) { // Z varie entre -32 et +31

      if (x < -THRESHOLD_XY) {
        v1 -= (x / 2);
        v1 = (v1 > PWM_MAX) ? PWM_MAX : v1;
        v2 -= (x / 2);
        v2 = (v2 > PWM_MAX) ? PWM_MAX : v2;
      } else if (x > THRESHOLD_XY) {
        v1 -= (x / 2);
        v1 = (v1 < 0) ? 0 : v1;
        v2 -= (x / 2);
        v2 = (v2 < 0) ? 0 : v2;
      }

      if (y > THRESHOLD_XY) {
        v1 -= (y / 2);
        v1 = (v1 < 0) ? 0 : v1;
        v4 -= (y / 2);
        v4 = (v4 < 0) ? 0 : v4;
      } else if (y < -THRESHOLD_XY) {
        v1 -= (y / 2);
        v1 = (v1 > PWM_MAX) ? PWM_MAX : v1;
        v4 -= (y / 2);
        v4 = (v4 > PWM_MAX) ? PWM_MAX : v4;
      }

      mPtrMoteur1-> setPWM(v1);
      mPtrMoteur2-> setPWM(v2);
      mPtrMoteur3-> setPWM(v3);
      mPtrMoteur4-> setPWM(v4);

      // delay(10);
    }
  }
}

void Drone::mayday() {

  char *buffer = "x = \0              ";
  char *buffer1 = "              ";

  // Couper les moteurs
  mPtrMoteur1->setPWM(0);
  mPtrMoteur2->setPWM(0);
  mPtrMoteur3->setPWM(0);
  mPtrMoteur4->setPWM(0);

  // Faire clignoter les 2 DELS
  while (1) {
    delay(500);
    digitalWrite(DELROUGE, LOW);
    digitalWrite(DELVERTE, LOW);
    delay(500);
    digitalWrite(DELROUGE, HIGH);
    digitalWrite(DELVERTE, HIGH);

    strcpy(buffer, "Mayday!\n");
    //    *buffer1 = 0;
    //    Serial1.print(strcat(buffer, itoa(x, buffer1, 10)));
    Serial1.print(buffer);


  }

}

/////////////////////////////////////////////////////////////////

Drone *leDrone = new Drone();

void setup() {

  // initialiser le port serie à 9600 bits par seconde:
  Serial1.begin(115200);
  while (!Serial1) ;
  Serial1.print("Hello World!\n");
  Serial1.flush();

  // initialiser le bus I2C
  Wire.begin();

  // initialize digital pin 13 as an output.
  pinMode(DELROUGE, OUTPUT);
  pinMode(DELVERTE, OUTPUT);
  digitalWrite(DELROUGE, LOW);   // turn the LED off (LOW is the voltage level)
  digitalWrite(DELVERTE, LOW);   // turn the LED off (LOW is the voltage level)

  for (byte i = 8; i < 120; i++)
  { Wire.beginTransmission (i);
    delay (5);
    if (Wire.endTransmission () == 0)
    {
      if ((i == GYROADD) || (i == ACCELADD)) {
        leDrone->showError(1, DELVERTE);
      } else {
        // On ne devrait jamais passer ici...
        leDrone->showError(1, DELROUGE);
      }
      delay (500);
    } // end of good response

    delay (5);  // give devices time to recover

  } // end of for loop

  // On effectue une comparairon de Timer0 avec une valeur arbitraire.
  OCR0A = 0x99;
  // On active les interruptions du comparateur A du Timer 0.
  TIMSK0 |= _BV(OCIE0A);

  // attachInterrupt(21, callback, RISING);  // attaches callback() as a timer 1 overflow interrupt

  // set up Timer 1
  //  TCCR1A = 0;          // normal operation
  //  TCCR1B = bit(WGM12) | bit(CS10) | bit (CS12);   // CTC, scale to clock / 1024
  //  OCR1A =  999;       // compare A register value (1000 * clock speed)
  //  TIMSK1 = bit (OCIE1A);             // interrupt on Compare A Match

  // activer l'accéléromètre
  Wire.beginTransmission(ACCELADD);
  Wire.write(ACCEL_MODE);
  Wire.write(0x01);
  unsigned char x = Wire.endTransmission();       // stop transmitting
  delayMicroseconds(2);
  leDrone->showError(x, DELROUGE);

  delay(10000);

}

void loop() {

  interrupts();
  // leDrone->test();
  Serial1.print("leDrone->vole()\n");
  Serial1.flush();
  delay(1000);
  leDrone->vole();

  leDrone->mayday();


}

