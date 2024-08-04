#ifndef _ACCELERATIONSENSORHPP
#define _ACCELERATIONSENSORHPP
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>

#define MEAN_NUM 10

#include <MPU6050_6Axis_MotionApps20.h>
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "Tasks.hpp"
#include <Statistic.h>
int maximum = 1000000;
SmartSerial sAccel(Serial, "Accel/", "\t");
template < size_t N>
class Moving_Average {
public:
    Moving_Average& operator()(float sample)
    {
        sample = abs(sample);
        total_ += sample;
        if (num_samples_ < N)
            samples_[num_samples_++] = sample;
        else {
            float& oldest = samples_[num_samples_++ % N];
            total_ -= oldest;
            oldest = sample;
        }
        // maximum_ -= maximum_/30+1;
        if (sample>maximum_)
        { 
            maximum_ = sample;
        }
        minimum_ += minimum_/10+1;
        if (sample<minimum_)
        { 
            minimum_ = sample;
        }
        return *this;
    }

    operator float() const { 
        // T normalized = abs((total_ / std::min(num_samples_, N))-minimum_)/(abs((maximum_-minimum_))+10);
        float normalized = (total_ / min(num_samples_, N))-minimum_;

        // return pow(maximum_,2)/pow(normalized,2); 
        return normalized*100;
    }

private:
    float samples_[N];
    float maximum_ { 0 };
    float minimum_ { 0 };
    size_t num_samples_ { 0 };
    float total_ { 0 };
};

class slowMax{
public:
    slowMax& operator()(float sample)
    {

        buf_ -= maximum_/(milliTimes/ACCELTIME);
        if (sample>buf_)
        { 
            maximum_ = sample;
            buf_ = sample;
        }
        return *this;
    }

    operator double() const { 
        return buf_; }

private:
    float buf_ { 0 };
    uint16 milliTimes = 5000;
    float maximum_ { 0 };
};
class AccelerationSensor : public MPU6050, public Task {

protected:
    Scheduler* iS;
    // MPU ontrol/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t ntStatus;       // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion quater;   // [w, x, y, z]         quaternion container
    VectorInt16 aa;      // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity; // [x, y, z]            gravity vector
    float euler[3];      // [psi, theta, phi]    Euler angle container
    float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    float x = 0, y = 0, z = 0, xm = 0, ym = 0, zm = 0, xmp = 0, ymp = 0, zmp = 0;

    Statistic stat_xyz[6];
    Statistic stat_xyz_backup[6];
    Moving_Average<5  > ma[6];
    // Moving_Average<float, float, 120> ma_backup[6];
    slowMax smax[6];

    bool Callback();

public:
    float movement_accel = 0;
    float movement_accel_backup = 0;
    int movement = -1;
    bool dead_flag = false;
    AccelerationSensor(unsigned long period, Scheduler* aS, Scheduler* aSensors);
    void Check_Acceleration();
    uint8_t bright = 5;
};

bool AccelerationSensor::Callback()
{
    // if programming failed, don't try to do anything
    if (!dmpReady) {
        Serial.println("MPU init failed - no action in Callback");
        return true;
    }
    // read a packet from FIFO
    if (dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
#ifdef OUTPUT_READABLE_QUATERNION
                                               // display quaternion values in easy matrix form: w x y z
        dmpGetQuaternion(&q, fifoBuffer);
        Serial.print("quat\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
        // display Euler angles in degrees
        dmpGetQuaternion(&q, fifoBuffer);
        dmpGetEuler(euler, &q);
        Serial.print("euler\t");
        Serial.print(euler[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        dmpGetQuaternion(&q, fifoBuffer);
        dmpGetGravity(&gravity, &q);
        dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
#endif
#define OUTPUT_READABLE_REALACCEL
#ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        dmpGetQuaternion(&quater, fifoBuffer);
        dmpGetAccel(&aa, fifoBuffer);
        dmpGetGravity(&gravity, &quater);
        dmpGetLinearAccel(&aaReal, &aa, &gravity);
        // Serial.print("\t areal\t");
        // Serial.print(aaReal.x);
        // Serial.print("\t");
        // Serial.print(aaReal.y);
        // Serial.print("\t");
        // Serial.print(aaReal.z);
        // Serial.print("\t sum: ");
        // // movement = abs(aaReal.x) + abs(aaReal.y) + abs(aaReal.z);
        // movement_accel = abs(aa.x) + abs(aa.y) + abs(aa.z);
        // sAccel << "\t Movement: " << movement << "\t Movement Accel: " << movement_accel;
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        dmpGetQuaternion(&q, fifoBuffer);
        dmpGetAccel(&aa, fifoBuffer);
        dmpGetGravity(&gravity, &q);
        dmpGetLinearAccel(&aaReal, &aa, &gravity);
        dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        Serial.print("\t aworld\t");
        Serial.print(aaWorld.x);
        Serial.print("\t");
        Serial.print(aaWorld.y);
        Serial.print("\t");
        Serial.println(aaWorld.z);
#endif

        // blink LED to indicate activity
        // blinkState = !blinkState;
        // digitalWrite(LED_PIN, blinkState);
    }
    Check_Acceleration();
    return false;
}

AccelerationSensor::AccelerationSensor(unsigned long period, Scheduler* aS, Scheduler* aSensors)
    : Task(period, TASK_FOREVER, aS, false)
{
    /* iS = aSensors; */
    // setTimeout(1000 * TASK_MILLISECOND);
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    /* Serial.begin(115200); */
    /* while (!Serial); // wait for Leonardo enumeration, others continue immediately */

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    initialize();
    Serial.print(getDeviceID());
    // pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
#define DEBUG_PRINT
    devStatus = dmpInitialize();
    //        X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
    // OFFSETS    -1096,    -831,    3634,      69,      73,     102
    // supply your own gyro offsets here, scaled for min sensitivity
    setXGyroOffset(69);
    setYGyroOffset(73);
    setZGyroOffset(102);
    setXAccelOffset(-1096);
    setYAccelOffset(-831);
    setZAccelOffset(3634); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        CalibrateAccel(6);
        CalibrateGyro(6);
        PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        setDMPEnabled(true);

        // enable Arduino interrupt detection
        // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        // Serial.println(F(")..."));
        // attachInterrupt(digitalPinToInterrupt(D8), dmpDataReady, RISING);
        ntStatus = getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = dmpGetFIFOPacketSize();

    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    enable();
}
void AccelerationSensor::Check_Acceleration()
{
    // sAccel << "Backup_count: " << stat_xyz_backup[2].count() << ssEndl;
    // sAccel << "count: " << stat_xyz[2].count() << ssEndl;
    // size_t backup_count=10;
    // if (stat_xyz_backup[0].count() > backup_count) {
    // sAccel << "Abs0: " << stat_xyz_backup[0].average() << "\t";
    // for (size_t i = 0; i < 6; i++) {
    //     float buf = stat_xyz_backup[i].average();
    //     stat_xyz_backup[i].clear();
    //     for (size_t k = 0; k < backup_count/2; k++) {
    //         stat_xyz_backup[i].add(buf);
    //     }
    // }
    // // setXGyroOffset(stat_xyz[0].average() );
    // // setYGyroOffset(stat_xyz[1].average() );
    // // setZGyroOffset(stat_xyz[2].average() );
    // // setXAccelOffset(stat_xyz[3].average() );
    // // setYAccelOffset(stat_xyz[4].average() );
    // // setZAccelOffset(stat_xyz[5].average() ); // 1688 factory default for my test chip
    // }
    // // if (stat_xyz[0].count() > 10) {
    // //     movement = abs(stat_xyz[0].average() - stat_xyz_backup[0].average())
    // //         + abs(stat_xyz[1].average() - stat_xyz_backup[1].average())
    //         + abs(stat_xyz[2].average() - stat_xyz_backup[2].average());
    //     movement_accel = abs(stat_xyz[3].average() - stat_xyz_backup[3].average())
    //         + abs(stat_xyz[4].average() - stat_xyz_backup[4].average())
    //         + abs(stat_xyz[5].average() - stat_xyz_backup[5].average());
    //     sAccel(SS_FUNC) << "Movement: " << movement_accel << "\t";
    //     sAccel << "Abs0: " << abs(stat_xyz[0].average() - stat_xyz_backup[0].average()) << "\t";
    //     sAccel << "Abs1: " << abs(stat_xyz[1].average() - stat_xyz_backup[1].average()) << "\t";
    //     sAccel << "Abs2: " << abs(stat_xyz[2].average() - stat_xyz_backup[2].average()) << "\t";
    //     // sAccel << "Backup: " << stat_xyz_backup[0].average() << "\t";
    //     // sAccel << "Std: " << stat_xyz[0].average() << "\t";
    //     sAccel << "Bright: " << bright << "\t";
    //     sAccel << ssEndl;
    //     for (size_t i = 0; i < 6; i++) {
    //         stat_xyz_backup[i].add(stat_xyz[i].average());
    //         stat_xyz[i].clear(); // explicitly start clean
    //     }
    //     // bright = map(movement_accel, 0, 500, 255, 0);
    // }
    ma[0](aaReal.x);
    ma[1](aaReal.y);
    ma[2](aaReal.z);
    ma[3](aa.x);
    ma[4](aa.y);
    ma[5](aa.z);
    // ma_backup[0](aaReal.x);
    // ma_backup[1](aaReal.y);
    // ma_backup[2](aaReal.z);
    // ma_backup[3](aa.x);
    // ma_backup[4](aa.y);
    // ma_backup[5](aa.z);
    // smax[0](aaReal.x);
    // smax[1](aaReal.y);
    // smax[2](aaReal.z);
    // smax[3](aa.x);
    // smax[4](aa.y);
    // smax[5](aa.z);
    // movement_accel = abs(ma[3]-ma_backup[3])
    //     + abs(ma[4]-ma_backup[4])
    //     + abs(ma[5]-ma_backup[5]);
    // movement = abs(ma[0] - ma_backup[0])
    //     + abs(ma[1] - ma_backup[1])
    //     + abs(ma[2] - ma_backup[2]);
    // movement_accel = abs(ma[3] - ma_backup[3])
    //     + abs(ma[4] - ma_backup[4])
    //     + abs(ma[5] - ma_backup[5]);
    // movement_accel_backup = abs(ma[3] - ma_backup[3])
    //     + abs(ma[4] - ma_backup[4])
    //     + abs(ma[5] - ma_backup[5]);
    movement = abs(ma[0] - smax[0])
        + abs(ma[1]+(abs(smax[1])+1)) 
        + abs(ma[2]+(abs(smax[2])+1));
    movement_accel = abs(ma[3]+(abs(smax[3])+1)) 
        + abs(ma[4]+(abs(smax[4])+1)) 
        + abs(ma[5]+(abs(smax[5])+1));
    movement = abs(ma[0])
        + abs(ma[1])
        + abs(ma[2]);
        // + abs(ma[3])
        // + abs(ma[4]);
        // + abs(ma[5]);
    // movement=0;
    //     for (int i = 0; i < (sizeof(ma) / sizeof(ma[0])); i++) {
    //           movement = movement+pow(ma[i],2);
   // }
    // movement = sqrt(movement);
    sAccel(SS_FUNC) << "Movement before: " << movement << "\t";
    movement = 0.001*pow(movement,2)+movement;
    movement = smax[0](movement);
    sAccel << "Movement: " << movement << "\t";
        // movement += movement_accel ;
    if(movement>maximum)
        movement=maximum;
    bright = map(movement, 0, maximum, 255, 0);
    // sAccel << "Movement backup: " << movement_accel_backup << "\t";
    sAccel << ssEndl;
    sAccel << "ma:\t" << ma[0] << "\t" << ma[1] << "\t" << ma[2]<< "\t" << ma[3]<< "\t" << ma[4]<< "\t" << ma[5]<< "\t";
    sAccel << ssEndl;
    sAccel << "Accel\t" << aaReal.x << "\t" << aaReal.y << "\t" << aaReal.z << "\t" << aa.x << "\t" << aa.y << "\t" << aa.z << "\t";
    sAccel << ssEndl;
    sAccel << "Bright: " << bright << "\t";
    // sAccel << ssEndl;
    // sAccel << "Smax\t" << smax[0] << "\t" << smax[1] << "\t" << smax[2]<< "\t" << smax[3]<< "\t" << smax[4]<< "\t" << smax[5]<< "\t";
    // sAccel << ssEndl;
    // // bright = map(movement_accel, 0, 500, 255, 0);
    // stat_xyz[0].add(aaReal.x);
    // stat_xyz[1].add(aaReal.y);
    // stat_xyz[2].add(aaReal.z);
    // stat_xyz[3].add(aa.x);
    // stat_xyz[4].add(aa.y);
    // stat_xyz[5].add(aa.z);

    if (bright < 10) {
        bright = 0;
        dead_flag = true;
        // void SetupBlackAndWhiteStripedPalette();
        // Alive_Task.disable();
        // changeColor_Task.disable();
        // changeColor();
    }
    // brightness = bright;
    // FastLED.setBrightness(brightness);
}
// for both classes must be in the include path of your project
#endif
