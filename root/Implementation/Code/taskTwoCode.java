package leJOSEV3;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class taskTwoCode {
	
	static EV3LargeRegulatedMotor motorLeft;
	static EV3LargeRegulatedMotor motorRight;
	
	static EV3ColorSensor colorSensor;	
	static SampleProvider colourProvider;
	static float[] lightSamples;
	
	static double wheelCircumference;
	
	static boolean[] lineValues;
	
	public static void init(){
		colorSensor = new EV3ColorSensor(SensorPort.S4);
		motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);
		motorRight = new EV3LargeRegulatedMotor(MotorPort.D);
		
		motorLeft.setSpeed(90);
		motorRight.setSpeed(90);
		motorLeft.setAcceleration(2000);
		motorRight.setAcceleration(2000);

		
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


	public static void localise(boolean[] lineValues,float sensorWorks,float motorWorks){
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
			totalProb = 0;
			for(int i=0;i<lineProbs.length;i++){
				totalProb+=lineProbs[i];
			}
			//System.out.println(totalProb);
			for(int i=0;i<lineProbs.length;i++){
				if(lightValue == lineValues[i]){
					lineProbs[i]=(sensorWorks*lineProbs[i])/totalProb;
				}
				else{
					lineProbs[i]=((1-sensorWorks)*lineProbs[i])/totalProb;
				}
			}
			moveForward(2);
			totalProb = 0;
			for(int i=0;i<lineProbs.length;i++){
				totalProb+=lineProbs[i];
			}
			System.out.println(totalProb);
			for(int i=0;i<lineProbs.length;i++){
				if(i==0){					
					lineProbs[i]=(lineProbs[i]*(1-motorWorks))/totalProb;
				}
				else{
					lineProbs[i]=(lineProbs[i]*(1-motorWorks)+lineProbs[i-1]*motorWorks)/totalProb;
				}
			}
			double maxVal = 0;
			int maxInd = 0;
			for(int i = 0; i < lineProbs.length; i++){
				if(lineProbs[i] > maxVal){
					maxVal = lineProbs[i];
					maxInd = i;
				}
			}
			//System.out.println(maxInd + " " + maxVal);
		}
	}
	public static void main(String[] args){
		init();
		localise(lineValues,(float)0.95,(float)0.95);
	}
}
