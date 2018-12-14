

#define kp 16.8
#define ki 0.18
#define kd 0.85

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

//pid data
double error = 0, max_speed = 100, lasterror = 0, derevative = 0, integ = 0, spd = 0;


int lmp = 3, lmd = 4, rmd = 8, rmp = 9;
int x,w,sp,y;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

double erc(double A)  // calculate the error value
{ double X = (A-3.4-sp) ;
  return X;

}
void pid()                                               // calculation of pid
{
  derevative = error - lasterror;
  integ = integ + error;          // value not set
  if (integ > 195)
    integ = 195;
  if (integ < (-195))
    integ = -195;

  spd = (kp * error) + (ki * integ) + (kd * derevative);
  lasterror = error;

}

void balance()
{ 
  if (spd < 0)
    spd = (-1) * spd;
  if(spd>200)
    spd=200;
    
  if(y>1)
  {if (error < 0) //back tilt 
  { 
    analogWrite(lmp, spd+y);
    digitalWrite(lmd, 0);
    analogWrite(rmp, spd-y);
    digitalWrite(rmd, 0);
  }
  else if (error > 0)                //front tilt 
  {
    digitalWrite(lmd, 1);
    analogWrite(lmp, spd-y);
    digitalWrite(rmd, 1);
    analogWrite(rmp, spd+y);
  }
  }
  else if(y<-1)
  {
    if (error < 0) //back tilt 
  
   { 
    analogWrite(lmp, spd-y);
    digitalWrite(lmd, 0);
    analogWrite(rmp, spd+y);
    digitalWrite(rmd, 0);
   }
  else if (error > 0)                //front tilt 
   {
    digitalWrite(lmd, 1);
    analogWrite(lmp, spd+y);
    digitalWrite(rmd, 1);
    analogWrite(rmp, spd-y);
   }
 }
  else
  {if (error < 0) //back tilt 
  { 
    analogWrite(lmp, spd);
    digitalWrite(lmd, 0);
    analogWrite(rmp, spd);
    digitalWrite(rmd, 0);
   }
  else if (error > 0)                //front tilt 
   {
    digitalWrite(lmd, 1);
    analogWrite(lmp, spd);
    digitalWrite(rmd, 1);
    analogWrite(rmp, spd);
    }
    
   else                              //no tilt
  { analogWrite(lmp, 0);
    digitalWrite(lmd, 0);
    analogWrite(rmp, 0);
    digitalWrite(rmd, 0);
  }
  }
}



// TODO: Make calibration routine

void setup() {
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void loop() {
  x=analogRead(A0);
  w=analogRead(A2);
  sp=map(x,0,1023,-5,5);
  y=map(w,0,1023,-45,45);
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;


  Serial.print(" kp = ");
  Serial.print(kp);
  Serial.print("\t");
  Serial.print(" ki = ");
  Serial.print(ki);
  Serial.print("\t");
  Serial.print(" kd = ");
  Serial.print(kd);
  Serial.print("\t");
  Serial.print(spd);
  Serial.print("\t");
   Serial.print(" p = ");
   Serial.print(error);
  Serial.print("\t");
   Serial.print(" d = ");
  Serial.print(derevative);
  Serial.print("\t");
   Serial.print(" i = ");
  Serial.print(integ);
  Serial.print("\t");
  Serial.print(compAngleX);
  Serial.print("\t");
  Serial.print(sp);
  Serial.print("\t");
  Serial.println(y);


  error = erc(compAngleX);
  pid();
  balance();

}
