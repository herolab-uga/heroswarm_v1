#include <Swarmbot.h>



SwarmBot::SwarmBot(byte leftIN1, byte leftIN2, byte leftA, byte leftB, byte leftPWR, byte leftGND, byte rightIN1, byte rightIN2, byte rightA, byte rightB, byte rightPWR, byte rightGND, byte mode, int CPP, int ratio, float wheelDiameterM, float wheelBaseM){
    //Motor Driver and Encoder Definitions
    leftMotorIN1 = leftIN1;
    leftMotorIN2 = leftIN2;
    leftEncoderA = leftA;
    leftEncoderB = leftB;
    leftEncoderPWR = leftPWR;
    leftEncoderGND = leftGND;

    rightMotorIN1 = rightIN1;
    rightMotorIN2 = rightIN2;
    rightEncoderA = rightA;
    rightEncoderB = rightB;
    rightEncoderPWR = rightPWR;
    rightEncoderGND = rightGND;   

    driverMode = mode;

    //Physical Constants
    encoderCPP = CPP;
    gearRatio = ratio;
    wheelDiameter = wheelDiameterM;
    wheelBase = wheelBaseM;
    toDegrees = (180/PI);
    toRadians = (PI/180);
    ticksPerMeter = (wheelDiameter*PI)/(encoderCPP * gearRatio);
    ticksPerMeter *= 10;

    //Odometery Constants
    leftLastEncoded = 0;
    leftEncoderValue = 0;
    leftDeltaTicks = 0;
    leftLastEncoderValue = 0;
    leftOmegaDeg = 0;
    leftOmegaRad = 0;
    leftRPM = 0;
    leftDegrees = 0;
    leftDeltaDegrees = 0;
    leftDeltaRevolutions = 0;

    rightLastEncoded = 0;
    rightEncoderValue = 0;
    rightDeltaTicks = 0;
    rightLastEncoderValue = 0;
    rightOmegaDeg = 0;
    rightOmegaRad = 0;
    rightRPM = 0;
    rightDegrees = 0;
    rightDeltaDegrees = 0;
    rightDeltaRevolutions = 0;

    lastTime = 0; 

    X = 0;
    Y = 0;
    velocityX = 0;
    velocityY = 0;
    thetaRadOdom = 0;
    thetaDegOdom = 0;
    thetaRadIMU = 0;
    thetaDegIMU = 0;

    linearVelocity = 0;
    angularVelocityOdom = 0;
    angularVelocityIMU =  0;
    vTheta = 0;

    targetX = 0;
    targetY = 0;

    //Linear Velocity PID Constants
    lKp = 0.1;
    lKi = 0;
    lKd = 0;
    //Angular Velocity PID Constants
    aKp = 0.06; //0.06
    aKi = 0.001; //0.001
    aKd = 0.00;
    aFF = 165; //165
    angleIntegral = 0;
    angleDerivative = 0;
    angleLastError = 0;

    //LeftMotor Velocity PID Constants
    lmKp = 0.25;
    lmKi = 0.05;
    lmKd = 0.00;
    lmFF = 160;
    leftMotorIntegral = 0;
    leftMotorDerivative = 0;
    leftMotorLastError =0;

    //RightMotor
    rmKp = .25;
    rmKi = 0.05; //0.05
    rmKd = 0.00;
    rmFF = 165;
    rightMotorIntegral = 0;
    rightMotorDerivative = 0;
    rightMotorLastError = 0;

   // imu IMU;
  //  IMU.setupIMU();
    
}

byte SwarmBot::getLeftEncoderA(){
    return leftEncoderA;
}
byte SwarmBot::getLeftEncoderB(){
    return leftEncoderB;
}
byte SwarmBot::getRightEncoderA(){
    return rightEncoderA;
}
byte SwarmBot::getRightEncoderB(){
    return rightEncoderB;
}


byte SwarmBot::getLeftIn1(){
    return leftMotorIN1;
}
byte SwarmBot::getLeftIn2(){
    return leftMotorIN2;
}
byte SwarmBot::getRightIn1(){
    return rightMotorIN1;
}
byte SwarmBot::getRightIn2(){
    return rightMotorIN2;
}
byte SwarmBot::getMode(){
    return driverMode;
}


void SwarmBot::initializePorts(){
  digitalWrite(leftEncoderPWR, HIGH);
  digitalWrite(leftEncoderGND, LOW);
  pinMode(leftEncoderGND, OUTPUT);
  pinMode(leftEncoderPWR, OUTPUT);

  pinMode(leftEncoderA, INPUT_PULLUP);
  pinMode(leftEncoderB, INPUT_PULLUP);
  digitalWrite(leftEncoderA, HIGH);
  digitalWrite(leftEncoderB, HIGH);

  digitalWrite(rightEncoderPWR, HIGH);
  digitalWrite(rightEncoderGND, LOW);
  pinMode(rightEncoderGND, OUTPUT);
  pinMode(rightEncoderPWR, OUTPUT);

  pinMode(rightEncoderA, INPUT_PULLUP);
  pinMode(rightEncoderB, INPUT_PULLUP);
  digitalWrite(rightEncoderA, HIGH);
  digitalWrite(rightEncoderB, HIGH);  

  digitalWrite(driverMode, LOW);

}

void SwarmBot::updateLeftEncoder(){
    int MSB = digitalRead(leftEncoderA); //MSB = most significant bit
    int LSB = digitalRead(leftEncoderB); //LSB = least significant bit

    int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
    int sum  = (leftLastEncoded << 2) | encoded; //adding it to the previous encoded value

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) leftEncoderValue --;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) leftEncoderValue ++;

    leftLastEncoded = encoded; //store this value for next time
}

void SwarmBot::updateRightEncoder(){
    int MSB = digitalRead(rightEncoderA); //MSB = most significant bit
    int LSB = digitalRead(rightEncoderB); //LSB = least significant bit

    int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
    int sum  = (rightLastEncoded << 2) | encoded; //adding it to the previous encoded value

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) rightEncoderValue --;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) rightEncoderValue ++;

    rightLastEncoded = encoded; //store this value for next time
}

void SwarmBot::updateWheels(){
   
    //Left Motor 
    leftDeltaTicks = leftEncoderValue - leftLastEncoderValue; 
    leftDeltaRevolutions = leftDeltaTicks/(encoderCPP*gearRatio);
    leftDeltaDegrees = leftDeltaRevolutions*360;

    //Right Motor
    rightDeltaTicks = rightEncoderValue - rightLastEncoderValue;
    rightDeltaRevolutions = rightDeltaTicks/(encoderCPP*gearRatio);
    rightDeltaDegrees = rightDeltaRevolutions*360;

    //Archive Values
    leftLastEncoderValue = leftEncoderValue;
    rightLastEncoderValue = rightEncoderValue;
}

float SwarmBot::angleWrap(float angle, bool isRad){
    double conversionFactor;
    if (isRad == true) conversionFactor = PI;
    else conversionFactor = 180.0;

    if(angle > (2.0*conversionFactor)) angle -= (2.0*conversionFactor);
    if(angle < 0.0) angle += (2.0*conversionFactor);
    return angle;
}
float SwarmBot::angleWrap2(float angle, bool isRad){
    double conversionFactor;
    if (isRad == true) conversionFactor = PI;
    else conversionFactor = 180.0;

    if(angle < ((-1)*conversionFactor)) angle += (2.0*conversionFactor);
    if(angle > conversionFactor) angle -= (2.0*conversionFactor);
    return angle;
}
void SwarmBot::updateOdometery(){
    updateWheels();
  //  IMU.updateIMU();
   // float heading = IMU.getHeading(true);
    ////Position
    //Convert Encoder Ticks to Linear Distance
    double dLeft = leftDeltaTicks * ticksPerMeter;
    double dRight = rightDeltaTicks * ticksPerMeter;
    double dCenter = (dLeft + dRight)/2;
    double dPhi = (dRight - dLeft)/wheelBase;
    
    //Position Output
    thetaRadOdom += dPhi;
    X += dCenter * cos(thetaRadOdom);
    Y += dCenter * sin(thetaRadOdom);

    ////Velocity 
    long currentTime = millis();
    if(currentTime < lastTime){
        currentTime = lastTime;
    } 

    //Time Conversions
    double deltaTime = (currentTime - lastTime);
    double deltaTimeSeconds = deltaTime/1000;
    double deltaTimeMinutes = deltaTime/60000;

    leftRPM = leftDeltaRevolutions/deltaTimeMinutes;
    rightRPM = rightDeltaRevolutions/deltaTimeMinutes;

    double vLeft = dLeft/deltaTimeSeconds;
    double vRight = dRight/deltaTimeSeconds;
    double vCenter = (vRight + vLeft)/2;

    //Velocity Outputs
    velocityX = vCenter * cos(thetaRadOdom);
    velocityY = vCenter * sin(thetaRadOdom);

    linearVelocity = sqrt(pow(velocityX, 2) + pow(velocityY,2));
    angularVelocityOdom = dPhi/deltaTimeSeconds;

    //Theta To Degrees
    thetaRadOdom = angleWrap(thetaRadOdom, true);
    thetaDegOdom = thetaRadOdom*toDegrees;

    lastTime = currentTime;
    //delay(20);
}

float SwarmBot::getHeading(){
   // return IMU.getHeading();
}
void SwarmBot::printOdom(){

   // float heading = IMU.getHeading(false);
    float angle = angleWrap2(thetaDegOdom, false);
    Serial.print("X: "); Serial.print(X,4); Serial.print(" Y: "); Serial.print(Y,4); Serial.print("targetX: "); Serial.print(targetX,4); Serial.print(" targetY: "); Serial.print(targetY,4); Serial.print(" Theta: "); Serial.println(angle);
   // delay(20);
}

void SwarmBot::moveDistance(float distance){
    float error = X - distance;

}

void SwarmBot::moveToPID(){


    float deltaX = targetX - X;
    float deltaY = targetY - Y;

    float distance = hypot(deltaX, deltaY);
    float absAngle = atan2(deltaY, deltaX);
    float relAngle = angleWrap2(absAngle, true);
    relAngle *= toDegrees;
    float currentAngle = angleWrap2(thetaDegOdom, false);

    float error = relAngle - currentAngle;
    angleWrap(error, false);
    float absError = abs(error);
    if(angleIntegral < 3000){
        angleIntegral += absError;
    }
    else{
        angleIntegral = 0;
    }

    angleDerivative = absError - angleLastError;
    angleLastError = absError;

    float output;

    output = (aKp * absError) + (aKi * angleIntegral) + (aKd * angleDerivative) + aFF;

    
    if(abs(error) > 20){
        // if(error > 0){
        //     leftOutput = -output;
        //     rightOutput = output;
        // }
        // else if (error < 0){
        //     leftOutput = output;
        //     rightOutput = -output;
        // }
        if (error > 0){
            analogWrite(rightMotorIN1, 0);
            analogWrite(rightMotorIN2, output);

            analogWrite(leftMotorIN1, output);
            analogWrite(leftMotorIN2, 0);

        }
        else if (error < 0){
            analogWrite(rightMotorIN1, output);
            analogWrite(rightMotorIN2, 0);

            analogWrite(leftMotorIN1, 0);
            analogWrite(leftMotorIN2, output);
        }
    }
    else{
        if(distance > 0.02){
            analogWrite(rightMotorIN1, 0);
            analogWrite(rightMotorIN2, 200);

            analogWrite(leftMotorIN1, 0);
            analogWrite(leftMotorIN2, 200);
        }
        else{
            analogWrite(rightMotorIN1, 0);
            analogWrite(rightMotorIN2, 0);

            analogWrite(leftMotorIN1, 0);
            analogWrite(leftMotorIN2, 0);
        }
    }
    Serial.println(distance);
    //Serial.print(output); Serial.print(","); Serial.print(currentAngle); Serial.print(","); Serial.println(error);
}

void SwarmBot::setRightMotorSpeed(float speed){
    float errorRight = (speed-rightRPM);

    // if (abs(rightMotorIntegral) < 5000){
    //     rightMotorIntegral += error;
    // }
    // else{
    //     rightMotorIntegral = 5000;
    // }
    rightMotorIntegral += errorRight;
    rightMotorDerivative = errorRight - rightMotorLastError;
    rightMotorLastError = errorRight;


    float outputRight;
    outputRight = (rmKp*errorRight) + (rmKi*rightMotorIntegral) + (rmKd * rightMotorDerivative) + rmFF; 
    //Serial.print("Right: "); Serial.print(error); Serial.print(" ");Serial.print(output); Serial.print(" ");
    //Serial.print("Right: ");
   // Serial.print(rightRPM); Serial.println(",");
    if (speed > 0){
        analogWrite(rightMotorIN1, 0);
        analogWrite(rightMotorIN2, outputRight);
    }
    else if (speed < 0){
        analogWrite(rightMotorIN1, outputRight);
        analogWrite(rightMotorIN2, 0);
    }   
}

void SwarmBot::setLeftMotorSpeed(float speed){
    float errorLeft = (speed-leftRPM);

    // if (abs(leftMotorIntegral) < 5000){
    //     leftMotorIntegral += error;
    // }
    
    leftMotorIntegral += errorLeft;
    leftMotorDerivative = errorLeft - leftMotorLastError;
    leftMotorLastError = errorLeft;

    float outputLeft;
    outputLeft = (lmKp*errorLeft) + (lmKi*leftMotorIntegral) + (lmKd * leftMotorDerivative) + lmFF; 

    //  Serial.print("Left: "); Serial.print(error); Serial.print(" ");Serial.print(output); Serial.print(" "); 
    //Serial.print("Left: ");
    //Serial.print(leftRPM); Serial.println(",");
    if (speed > 0){
        analogWrite(leftMotorIN1, 0);
        analogWrite(leftMotorIN2, outputLeft);
    }
    else if (speed < 0){
        analogWrite(leftMotorIN1, outputLeft);
        analogWrite(leftMotorIN2, 0);
    }   
}

void SwarmBot::setMotorSpeed(float m1, float m2){
    // setLeftMotorSpeed(m1);
    // setRightMotorSpeed(m2);
   // Serial.print(leftRPM); Serial.print(","); Serial.println(rightRPM);
   
    if (m1 > 0){
        m1 += lmFF;
        analogWrite(leftMotorIN1, 0);
        analogWrite(leftMotorIN2, m1);

    }
    else {
        m1 *= -1;
        m1 += lmFF;
        analogWrite(leftMotorIN1, m1);
        analogWrite(leftMotorIN2, 0);
    }


    if (m2 > 0){
        m2 += rmFF;
        analogWrite(rightMotorIN1, 0);
        analogWrite(rightMotorIN2, m2);
    }
    else {
        m2 *= -1;
        m2 += rmFF;
        analogWrite(rightMotorIN1, m2);
        analogWrite(rightMotorIN2, 0);
    }

}


void SwarmBot::callibrateOdometery(float inputArray[5]){
    X = inputArray[0];
    Y = inputArray[1];
    targetX = inputArray[2];
    targetY = inputArray[3];
    thetaRadOdom = toRadians*inputArray[4];
    
}
void SwarmBot::moveToPoint(float x, float y){
    // float deltaX = x - X;
    // float deltaY = y - Y;

    // float distance = hypot(deltaX, deltaY);
    // float absAngle = atan2(deltaY, deltaX);
    // float relAngle = angleWrap2(absAngle, true);
    // relAngle *= toDegrees;
    // //float turnOut = anglePID(relAngle);
    // float distanceOut = 0;
    // float leftOut = -turnOut;
    // float rightOut = turnOut;

   // setMotorSpeed(leftOut, rightOut);

   // Serial.print(distance); Serial.print(",");Serial.print(relAngle); Serial.print(",");Serial.println(turnOut);

}
