package leJOSEV3;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class FinalJackTwo {
    
    static EV3LargeRegulatedMotor motorLeft;
    static EV3LargeRegulatedMotor motorRight;
    static EV3LargeRegulatedMotor[] toSynchronise;
    static double wheelCircumference;
    static EV3GyroSensor gyroSensor;
    static SampleProvider angleProvider;
    static double currentAngle;
    static double errorAngle;
    static double errorLastAngle;
    static double kP;
    static double kD;
    static double derivativeAngle;
    static double correctionAngle;
    static double RIGHT_ANGLE;
    static float[] angleSamples;
    
    static boolean[] lineValues;
    static boolean notLocalised;
    static SampleProvider colourProvider;
    static EV3ColorSensor colorSensor;
    static boolean lightValue;
    static float[] lightSamples;
    static HashMap<Integer, Double> lineProbabilities;
    static double normValue;
    static double totalProb;
    
    //Test probabilities...
    static double sensorWorks;
    static double sensorNot;
    static double motorCorrect;
    static double motorWrong;
    
    static EV3TouchSensor impactSensor;
    static SampleProvider impactProvider;
    static float[] impactSamples;
    static float impactValue;
    
    static EV3UltrasonicSensor soundSensor;
    static SampleProvider soundProvider;
    static float[] soundSamples;
    static float soundValue;
    
    static int stateTracker;
    
    public static void initialiser() {
        motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);
        motorRight = new EV3LargeRegulatedMotor(MotorPort.D);
        toSynchronise = new EV3LargeRegulatedMotor[]{motorRight};
        
        gyroSensor = new EV3GyroSensor(SensorPort.S1);
        wheelCircumference = 2*2.75*Math.PI;
        angleProvider = gyroSensor.getAngleMode();
        errorLastAngle = 0;
        kP = 0.1;
        kD = 0.1;
        //RIGHT_ANGLE = 90.0;
        angleSamples = new float[1];
        lightSamples = new float[1];
        impactSamples = new float[1];
        
        motorLeft.setSpeed(90);
        motorRight.setSpeed(90);
        motorLeft.setAcceleration(2000);
        motorRight.setAcceleration(2000);
        
        //True is BLUE - False is WHITE
        lineValues = new boolean[]{true, false, true, false, false, true, true, false, true, true, false, false, true, true, true, false, true, true, true, false, false, true, true, true, false, false, false, true, true, true, true, false, false, false};;
        colorSensor = new EV3ColorSensor(SensorPort.S4);
        colourProvider = colorSensor.getRedMode();
        notLocalised = true;
        lineProbabilities = new HashMap<Integer, Double>();
        for(int i =0; i < 34; i++) {
            lineProbabilities.put(i, (double) 1/34);
        }
        normValue = 1;
        sensorWorks = 0.98;
        sensorNot = 0.02;
        
        impactSensor = new EV3TouchSensor(SensorPort.S3);
        impactProvider = impactSensor.getTouchMode();
        
        soundSensor = new EV3UltrasonicSensor(SensorPort.S2);
        soundProvider = soundSensor.getDistanceMode();
        
        stateTracker = -1;
        stateTracker++;
        
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
        if(raw < 0.78){
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
    
    //Task Two...
    public static void potentialPlan() {
        
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
        //		System.out.println("test");
        //	    Button.waitForAnyPress();
        //		for (Map.Entry<Integer, Double> entry : lineProbabilities.entrySet()) {
        //		    System.out.print(" "+entry.getValue() +" ");
        //		}
        //	    Button.waitForAnyPress();
        //System.out.println(lineProbabilities.values());
        //Button.waitForAnyPress();
        
        localiseLine();
        
        
        /*initialiser();
         while(true){
         
         if(stateTracker == 0){
         
         localiseLine();
         
         }
         
         else if(stateTracker == 1){
         
         potentialPlan();
         
         }
         
         else if(stateTracker == 2){
         
         }
         }*/
    }
    
}

