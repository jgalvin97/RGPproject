package leJOSEV3;

import java.util.HashMap;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class FinalJackTwo {
    
    static EV3ColorSensor colorSensor;
    static EV3GyroSensor gyroSensor;
    static EV3TouchSensor impactSensor;
    static EV3UltrasonicSensor soundSensor;
    static EV3LargeRegulatedMotor motorLeft;
    static EV3LargeRegulatedMotor motorRight;
    
    static SampleProvider colourProvider;
    static float[] lightSamples;
    static boolean lightValue;
    
    static SampleProvider angleProvider;
    static float[] angleSamples;
    static double currentAngle;
    
    static SampleProvider impactProvider;
    static float[] impactSamples;
    static float impactValue;
    
    static SampleProvider soundProvider;
    static float[] soundSamples;
    static float soundValue;
    
    static EV3LargeRegulatedMotor[] toSynchronise;
    static double wheelCircumference;
    
    static double errorAngle;
    static double errorLastAngle;
    static double kP;
    static double kD;
    static double derivativeAngle;
    static double correctionAngle;
    static double RIGHT_ANGLE;
    
    static boolean[] lineValues;
    static boolean notLocalised;
    static HashMap<Integer, Double> lineProbabilities;
    static double normValue;
    static double totalProb;
    
    static double sensorWorks;
    static double sensorNot;
    static double motorCorrect;
    static double motorWrong;
    
    static int stateTracker;
    
    public static void initialiser() {
        colorSensor = new EV3ColorSensor(SensorPort.S4);
        gyroSensor = new EV3GyroSensor(SensorPort.S1);
        impactSensor = new EV3TouchSensor(SensorPort.S3);
        soundSensor = new EV3UltrasonicSensor(SensorPort.S2);
        motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);
        motorRight = new EV3LargeRegulatedMotor(MotorPort.D);
        
        colourProvider = colorSensor.getRedMode();
        angleProvider = gyroSensor.getAngleMode();
        impactProvider = impactSensor.getTouchMode();
        soundProvider = soundSensor.getDistanceMode();
        lightSamples = new float[1];
        angleSamples = new float[1];
        impactSamples = new float[1];
        soundSamples = new float[1];
        
        toSynchronise = new EV3LargeRegulatedMotor[]{motorRight};
        
        wheelCircumference = 2*2.75*Math.PI;
        errorLastAngle = 0;
        kP = 0.1;
        kD = 0.1;
        //RIGHT_ANGLE = 90.0;
        
        motorLeft.setSpeed(90);
        motorRight.setSpeed(90);
        motorLeft.setAcceleration(2000);
        motorRight.setAcceleration(2000);
        
        //True is BLUE - False is WHITE
        lineValues = new boolean[]{true, false, true, false, false, true, true, false, true, true, false, false, true, true, true, false, true, true, true, false, false, true, true, true, false, false, false, true, true, true, true, false, false, false};;
        notLocalised = true;
        lineProbabilities = new HashMap<Integer, Double>();
        for(int i =0; i < 34; i++) {
            lineProbabilities.put(i, (double) 1/34);
        }
        normValue = 1;
        sensorWorks = 0.98;
        sensorNot = 0.02;
        
        stateTracker = 0; 
    }
    
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
    
    public static void rotateLeft(double targetAngle) {
        gyroSensor.reset();
        while(true){
            currentAngle = readAngle(angleSamples);
            System.out.println(currentAngle);
            errorAngle = targetAngle - currentAngle;
            derivativeAngle = errorAngle + errorLastAngle;
            correctionAngle = kP*errorAngle + kD*derivativeAngle;
            
            motorLeft.synchronizeWith(new EV3LargeRegulatedMotor[]{motorRight});
            motorLeft.startSynchronization();
            
            motorLeft.setSpeed((int) correctionAngle);
            motorRight.setSpeed((int) correctionAngle);
            if(currentAngle > targetAngle) {
                motorLeft.backward();
                motorRight.forward();
            } else {
                motorLeft.forward();
                motorRight.backward();
            }
            
            motorLeft.endSynchronization();
            errorLastAngle = errorAngle;
        }
    }
    
    public static void rotateRight(double targetAngle) {
        gyroSensor.reset();
        while(true){
            currentAngle = readAngle(angleSamples);
            System.out.println(currentAngle);
            errorAngle = targetAngle - currentAngle;
            derivativeAngle = errorAngle + errorLastAngle;
            correctionAngle = kP*errorAngle + kD*derivativeAngle;
            
            motorLeft.synchronizeWith(new EV3LargeRegulatedMotor[]{motorRight});
            motorLeft.startSynchronization();
            
            motorLeft.setSpeed((int) correctionAngle);
            motorRight.setSpeed((int) correctionAngle);
            
            motorLeft.forward();
            motorRight.backward();
            
            motorLeft.endSynchronization();
            errorLastAngle = errorAngle;
        }
    }
    
    public static float readAngle(float[] sampleArray){
        angleProvider.fetchSample(sampleArray, 0);
        return sampleArray[0];
    }
    
    public static float readLight(float[] sampleArray) {
        colourProvider.fetchSample(sampleArray, 0);
        return sampleArray[0];
    }
    
    public static float readImpact(float[] sampleArray) {
        impactProvider.fetchSample(sampleArray, 0);
        return sampleArray[0];
    }
    
    public static float readSound(float[] sampleArray) {
        soundProvider.fetchSample(sampleArray, 0);
        return sampleArray[0];
    }
    
    //Increase precision...
    public static boolean processLight(double raw){
        if(raw < 0.6){
            return true;
        }
        else{
            return false;
        }
    }
    
    //Task One...
    public static void localiseLine(){
        while(notLocalised) {
            //Take a light reading...
            lightValue = processLight(readLight(lightSamples));
            totalProb = 0;
            for(double entry : lineProbabilities.values()) {
                totalProb += entry;
            }
            normValue = 1/totalProb;
            
            for(int i = 0; i < lineValues.length; i++) {
                if(lightValue == lineValues[i]){
                    lineProbabilities.put(i,normValue*sensorWorks*lineProbabilities.get(i));
                }
                else{
                    lineProbabilities.put(i,normValue*sensorNot*lineProbabilities.get(i));
                }
                
            }
            moveForward(2);
            for(int i = 0; i < lineValues.length; i++) {
                double newValue;
                if(i == 0){
                    newValue = normValue*((lineProbabilities.get(i+1))*motorWrong + (lineProbabilities.get(i))*motorWrong);
                } else {
                    newValue = normValue*(lineProbabilities.get(i-1))*motorCorrect + (lineProbabilities.get(i/*+1*/))*motorWrong + (lineProbabilities.get(i))*motorWrong; 
                }
                lineProbabilities.put(i, newValue);
            }
            for(double i:lineProbabilities.keySet()){
                System.out.print(""+i+" ");
            }
            
            //for(int i =0; i < lineValues.length; i++) {
            //if(lineProbabilities.get(i) > 0.9) {
            //break;
            //}
            //}
            
        }
        stateTracker++;
    }

    public static void impactStop() {
        impactValue = (int)(readImpact(impactSamples));
        if(impactValue == 1) {
            stopMotors();
            playBeep();
        }
    }
    
    public static void stopMotors() {
        motorLeft.stop();
        motorRight.stop();
    }
    
    public static void playBeep() {
        Sound.beep();
    }
    
    public static void main(String args[]) {
        initialiser();
        
        localiseLine();
        
        /* 
        while(true){
            if(stateTracker == 0){
                localiseLine();
            } 
            else 
                if(stateTracker == 1){
                    potentialPlan();
                }
                else if(stateTracker == 2){
         
                }
         }*/
    }   
}