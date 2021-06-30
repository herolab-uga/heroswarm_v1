
#ifndef SWARMBOT_H
#define SWARMBOT_H
    #include <Arduino.h>
    #include <IMU.h>
    #include <DRV8835MotorShield.h>

    extern imu IMU;
    extern DRV8835MotorShield motors;
    class SwarmBot{
        //Motor Driver and Encoder Definitions 
        byte leftMotorIN1;
        byte leftMotorIN2;
        byte leftEncoderA;
        byte leftEncoderB;
        byte leftEncoderPWR;
        byte leftEncoderGND;

        byte rightMotorIN1;
        byte rightMotorIN2;
        byte rightEncoderA;
        byte rightEncoderB;
        byte rightEncoderPWR;
        byte rightEncoderGND;

        byte driverMode;

        //Physical Constants
        int encoderCPP;
        int gearRatio;
        float wheelDiameter;
        float wheelBase;
        double toDegrees;
        double toRadians;
        float ticksPerMeter;

        //Odometery Constants
        volatile int leftLastEncoded;
        volatile int leftEncoderValue;
        double leftDeltaTicks;
        double leftLastEncoderValue;
        float leftOmegaDeg;
        float leftOmegaRad;
        float leftRPM;
        float leftVelocity;
        float leftDegrees;
        float leftDeltaDegrees;
        float leftDeltaRevolutions;

        volatile int rightLastEncoded;
        volatile int rightEncoderValue;
        double rightDeltaTicks;
        double rightLastEncoderValue;
        float rightOmegaDeg;
        float rightOmegaRad;
        float rightRPM;
        float rightVelocity;
        float rightDegrees;
        float rightDeltaDegrees;
        float rightDeltaRevolutions;

        double lastTime;

        float X;
        float Y;
        float velocityX;
        float velocityY;

        float targetX;
        float targetY;

        float thetaRadOdom;
        float thetaDegOdom;
        float thetaRadIMU;
        float thetaDegIMU;

        float linearVelocity;
        float angularVelocityOdom;
        float angularVelocityIMU;
        float vTheta;

        float lKp;
        float lKi;
        float lKd;

        float aKp;
        float aKi;
        float aKd;
        float aFF;
        float angleIntegral;
        float angleDerivative;
        float angleLastError;

        float lmKp;
        float lmKi;
        float lmKd;
        float lmFF;
        float leftMotorIntegral;
        float leftMotorDerivative;
        float leftMotorLastError;

        float rmKp;
        float rmKi;
        float rmKd;
        float rmFF;
        float rightMotorIntegral;
        float rightMotorDerivative;
        float rightMotorLastError;

        public:

            //SwarmBot Constructor
            SwarmBot(byte leftIN1 = 13, byte leftIN2 = 12, byte leftA = A2, byte leftB = A1, byte leftPWR = A3, byte leftGND = A0, byte rightIN1 = 11, byte rightIN2 = 10, byte rightA = 5, byte rightB = 6, byte rightPWR = A4, byte rightGND = A5, byte mode = 9, int CPP = 28, int ratio = 50, float wheelDiameter = 0.00325, float wheelBase = 0.0073);
            void initializePorts();
            byte getLeftEncoderA();
            byte getLeftEncoderB();
            byte getRightEncoderA();
            byte getRightEncoderB();
            byte getLeftIn1();
            byte getLeftIn2();
            byte getRightIn1();
            byte getRightIn2();
            byte getMode();
            //Base Motor Control
            void setLeftMotorSpeed(float speed);
            void setRightMotorSpeed(float speed);
            void setLeftMotorPos(float pos);
            void setRightMotorPos(float pos);

            //Odometery Functions
            void updateLeftEncoder();
            void updateRightEncoder();
            void updateWheels();
            float angleWrap(float angle, bool isRad = true);
            float angleWrap2(float angle, bool isRad = true);
            void updateOdometery();
            void callibrateOdometery(float inputArray[5]);
            void moveToPoint(float x, float y);
            //Chassis Movement
            void setVelocity(float linearVelocity, float angularVelocity);
            void setPosition(float X, float Y);
            void moveDistance(float distance);
            void moveToPID();
            void setMotorSpeed(float m1, float m2);
            //Sensors
            float getHeading();
            //Output
            void printOdom();
    };

#endif