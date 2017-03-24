package leJOSEV3;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class taskTwoCode {
    
    static EV3LargeRegulatedMotor motorLeft;
    static EV3LargeRegulatedMotor motorRight;
    
    static EV3TouchSensor impactSensor;
    static EV3GyroSensor gyroSensor;
    static EV3ColorSensor colorSensor;
    
    static SampleProvider gyroProvider;
    static SampleProvider gyroSamples;
    static float[] angle = {0.0f};
    static float gyroTacho;
    
    static SampleProvider impactProvider;
    static float[] impactSamples;
    static float impactValue;
    
    static SampleProvider colourProvider;
    static float[] lightSamples;
    
    static double wheelCircumference;
    
    static boolean[] lineValues;
    
    static float colourGoal;
    
    public static void init(){
        colorSensor = new EV3ColorSensor(SensorPort.S4);
        motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);
        motorRight = new EV3LargeRegulatedMotor(MotorPort.D);
        
        motorLeft.setSpeed(90);
        motorRight.setSpeed(90);
        motorLeft.setAcceleration(2000);
        motorRight.setAcceleration(2000);
        
        gyroSensor = new EV3GyroSensor(SensorPort.S1);
        gyroSamples = gyroSensor.getAngleMode();
        gyroTacho = 0;
        
        impactSensor = new EV3TouchSensor(SensorPort.S3);
        impactProvider = impactSensor.getTouchMode();
        impactSamples = new float[1];
        
        colourProvider = colorSensor.getRedMode();
        lightSamples = new float[1];
        
        wheelCircumference = 2*2.75*Math.PI;
        
        lineValues = new boolean[]{true, false, true, false, false, true, true, false, true, true, false, false, true, true, true, false, true, true, true, false, false, true, true, true, false, false, false, true, true, true, true, false, false, false};
    }
    
    public static boolean processLight(float[] sampleArray){
        colourProvider.fetchSample(sampleArray,0);
        if(sampleArray[0] < 0.6) {
            return true;
        }
        else{
            return false;
        }
    }
    
    //Read Light
    public static float readLight(float[] sampleArray) {
        colourProvider.fetchSample(sampleArray, 0);
        return sampleArray[0];
    }
    
    //Read Impact
    public static float readImpact(float[] sampleArray) {
        impactProvider.fetchSample(sampleArray, 0);
        return sampleArray[0];
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
    
    public static void rotateLeft(double degrees) {
        resetGyro();
        while(true) {
            motorLeft.backward();
            motorRight.forward();
            
            while (getGyroAngle() < degrees) {
                Thread.yield();
            }
            motorLeft.stop();
            motorRight.stop();
            
            Delay.msDelay(100);
            
            if (getGyroAngle() > degrees) {
                motorLeft.setSpeed(30);
                motorRight.setSpeed(30);
                //turn left slowly to correct the rotation angle
                motorLeft.forward();
                motorRight.backward();
                while (getGyroAngle() > degrees + 3) {
                    Thread.yield();
	               }
                motorLeft.stop();
                motorRight.stop();
            }
            resetGyroTacho();
        }
    }
    
    public static void rotateRight(double degrees) {
        resetGyro();
        
        while(true) {
            motorLeft.forward();
            motorRight.backward();
            
            while (getGyroAngle() < degrees) {
                Thread.yield();
            }
            motorLeft.stop();
            motorRight.stop();
            
            Delay.msDelay(100);
            
            if (getGyroAngle() > degrees) {
                motorLeft.setSpeed(30);
                motorRight.setSpeed(30);
                //turn left slowly to correct the rotation angle
                motorRight.forward();
                motorLeft.backward();
                while (getGyroAngle() > degrees + 3) {
                    Thread.yield();
	               }
                motorLeft.stop();
                motorRight.stop();
            }
            resetGyroTacho();
        }
    }
    
    public static float getGyroAngleRaw() {
        gyroSamples.fetchSample(angle, 0);
        return angle[0];
	   }
    
    public static float getGyroAngle() {
        float rawAngle = getGyroAngleRaw();
        return rawAngle - gyroTacho;
	   }
    
    public static void resetGyroTacho() {
        gyroTacho += getGyroAngle();
	   }
    
    public static void resetGyro() {
        if (gyroSensor != null) {
            gyroSensor.reset();
            gyroSamples = gyroSensor.getAngleMode();
            gyroTacho = 0;
        }
	   }
    
    public static double localise(boolean[] lineValues,float sensorWorks,float motorWorks){
        boolean lightValue;
        double totalProb;
        double[] lineProbs = new double[lineValues.length];
        
        for(int i = 0; i < lineProbs.length;i++){
            lineProbs[i] = 1/(float) lineValues.length;
        }
        System.out.println("press button :)");
        
        while(true){
            Button.waitForAnyPress();
            lightValue = processLight(lightSamples);
            
            /*Update*/
            for(int i=0;i<lineProbs.length;i++){
                
                if(lightValue == lineValues[i]) {
                    lineProbs[i]=(sensorWorks*lineProbs[i]);
                }
                else{
                    lineProbs[i]=((1-sensorWorks)*lineProbs[i]);
                }
            }
            
            /*SUM*/
            totalProb = 0;
            for(int i=0;i<lineProbs.length;i++){
                totalProb+=lineProbs[i];
            }
            
            /*NORMALISE*/
            for(int i=0;i<lineProbs.length;i++){
                lineProbs[i] = lineProbs[i]/totalProb;
            }
            //Moves the robot forward 2.0cm...
            moveForward(1.8);
            /*UPDATE*/
            for(int i=lineProbs.length -1; i >= 0; i--){
                if(i==0){
                    lineProbs[i]=(lineProbs[i]*(1-motorWorks));
                }
                else{
                    lineProbs[i]=(lineProbs[i]*(1-motorWorks)+lineProbs[i-1]*motorWorks);
                }
            }
            
            /* SUM */
            totalProb = 0; 
            for(int i=0;i<lineProbs.length;i++){
                
                totalProb+=lineProbs[i];
            }
            
            /* NORMALISE */
            for(int i=0;i<lineProbs.length;i++){
                lineProbs[i] = lineProbs[i]/totalProb;
            }
            
            double maxVal = 0;
            int maxInd = 0;
            for(int i = 0; i < lineProbs.length; i++){
                if(lineProbs[i] > maxVal) {
                    maxVal = lineProbs[i];
                    maxInd = i;
                }
            }
            
            if(maxVal > 0.75) {
                return (maxInd*2.0);
            }
            
        }
    }
    
    public static void senseBeepReverse(){
        moveForward(18.0);
        while((int)(readImpact(impactSamples)) != 1){
            moveForward(1.0);
        }
        //Impact Seen, Stop Motors Immediately
        motorLeft.stop();
        motorRight.stop();
        //Read Colour
        colourGoal = readLight(lightSamples);
        //Signal goal achieved
        Sound.beep();
        //Move Backwards
        moveBackward(18.0);
        //rotate right
        rotateRight(65);
    }
    
    public static void main(String[] args) {
        init();
        localise(lineValues,(float)0.95,(float)0.95);
    }
}
