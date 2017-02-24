package leJOSEV3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;

public class FinalJackTwo {
    
    static EV3LargeRegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);
    static EV3LargeRegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.D);
    static EV3LargeRegulatedMotor[] toSynchronise = new EV3LargeRegulatedMotor[]{motorRight};
    static double wheelCircumference=2*2.75*Math.PI;
    static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S1);
    static SampleProvider angleProvider = gyroSensor.getAngleMode();
    static double currentAngle;
    static double errorAngle;
    static double errorLastAngle = 0;
    static double kP = 50;
    static double kD = 50;
    static double derivativeAngle;
    static double correctionAngle;
    static double RIGHT_ANGLE = 90.0;
    static float[] angleSamples = new float[1];
    
    
    
    
    public static void moveForward(double distance){
        
        int measure = (int)Math.round((distance/wheelCircumference)*360.0);
        
        motorLeft.synchronizeWith(new EV3LargeRegulatedMotor[]{motorRight});
        motorLeft.startSynchronization();
        
        motorLeft.rotate(measure,true);
        motorRight.rotate(measure,true);
        
        motorLeft.endSynchronization();
        motorLeft.waitComplete();
        motorRight.waitComplete();
        
    }
    
    public static void moveBackward(double distance) {
        int measure = (int)Math.round((distance/wheelCircumference)*360.0 * (-1));
        
        motorLeft.synchronizeWith(new EV3LargeRegulatedMotor[]{motorRight});
        motorLeft.startSynchronization();
        
        motorLeft.rotate(measure,true);
        motorRight.rotate(measure,true);
        
        motorLeft.endSynchronization();
        motorLeft.waitComplete();
        motorRight.waitComplete();
    }
    
    public static void rotateLeft() {
        gyroSensor.reset();
        while(true){
            currentAngle = readAngle(angleSamples);
            errorAngle = RIGHT_ANGLE - currentAngle;
            derivativeAngle = errorAngle + errorLastAngle;
            correctionAngle = kP*errorAngle + kD*derivativeAngle;
            
            motorLeft.synchronizeWith(new EV3LargeRegulatedMotor[]{motorRight});
            motorLeft.startSynchronization();
            
            motorLeft.setSpeed((int) correctionAngle * -1);
            motorRight.setSpeed((int) correctionAngle);
            
            motorLeft.forward();
            motorRight.forward();
            
            motorLeft.endSynchronization();
            errorLastAngle = errorAngle;
        }
    }
    
    public static float readAngle(float[] sampleArray){
        angleProvider.fetchSample(sampleArray, 0);
        return sampleArray[0];
    }
    
    
    
    
    public static void main(String args[]) {
        
        motorLeft.setSpeed(90);
        motorRight.setSpeed(90);
        motorLeft.setAcceleration(2000);
        motorRight.setAcceleration(2000);
        
        moveForward(5);
        Delay.msDelay(1000);
        moveBackward(5);
        
    }
    
}
