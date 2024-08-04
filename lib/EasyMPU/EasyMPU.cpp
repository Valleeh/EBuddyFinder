
#include "EasyMPU.h"

void EasyMPU::Init()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println("PID tuning Each Dot = 100 readings");
  /*A tidbit on how PID (PI actually) tuning works. 
    When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and 
    integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral 
    uses the error from set-point (set-point is zero), it takes a fraction of this error (error * ki) and adds it 
    to the integral value. Each reading narrows the error down to the desired offset. The greater the error from 
    set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the 
    integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the 
    noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100 
    readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to 
    the fact it reacts to any noise.
  */
  CalibrateAccel(6);
  CalibrateGyro(6);
  Serial.println("\nat 600 Readings");
  PrintActiveOffsets();

  Serial.println(F("Enabling DMP..."));
  setDMPEnabled(true);
  // get expected DMP packet size for later comparison
  packetSize = dmpGetFIFOPacketSize();
}

void EasyMPU::reading()
{

  if (dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    /// Erste Messung
    //mpu.read();
    /* Get new sensor events with the readings */
    // display real acceleration, adjusted to remove gravity
    dmpGetQuaternion(&q, fifoBuffer);
    dmpGetAccel(&aa, fifoBuffer);
    dmpGetGravity(&gravity, &q);
    dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //Serial.print("\t areal\t");
    //Serial.print(aaReal.x);
    //Serial.print("\t");
    //Serial.print(aaReal.y);
    //Serial.print("\t");
    //Serial.print(aaReal.z);
    //Serial.print("\t sum: ");
    //Serial.print(abs(aaReal.x)+abs(aaReal.y)+abs(aaReal.z));
    //sensors_event_t a, g, temp;
    //getEvent(&a, &g, &temp);
    xm = x - aaReal.x;
    ym = y - aaReal.x;
    zm = z - aaReal.x;

    x = aaReal.x;
    y = aaReal.x;
    z = aaReal.x;
    stat_xyz[0].add(x);
    stat_xyz[1].add(y);
    stat_xyz[2].add(z);
    ymp = abs(pow(1024, ym));
    xmp = abs(pow(1024, xm));
    zmp = abs(pow(1024, zm));
  }
}
