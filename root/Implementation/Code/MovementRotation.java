package leJOSEV3;

import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.port.Port;
import lejos.hardware.port.MotorPort;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class MovementRotation {
	static double wheelDiameter = 6.5;
	static double robotTrack = 12.2;
	
	
	public static void main(String[] args) {
		/* Size of square to move around */
		int squareSize = 40;
		double wheelCircumference = MovementRotation.wheelDiameter*Math.PI;
		
		Brick brick = BrickFinder.getDefault();
		
		EV3LargeRegulatedMotor motorLeft =new EV3LargeRegulatedMotor(brick.getPort("B"));
		EV3LargeRegulatedMotor motorRight=new EV3LargeRegulatedMotor(brick.getPort("C"));

		
		motorLeft.setAcceleration(400);
		motorRight.setAcceleration(400);
		
		motorLeft.setSpeed(400);
		motorRight.setSpeed(400);
		
		for(int i = 0; i < 4; i++) {
			int degrees = (int) Math.round((squareSize/wheelCircumference)*360.0);
			motorLeft.rotate(degrees,true);
			motorRight.rotate(degrees);
			
			degrees = (int) Math.round((((robotTrack*Math.PI)/4.0)/wheelCircumference)*360.0);
			
			double K = 0.98;
			degrees = (int) (K*degrees);
			
			motorLeft.rotate(-degrees,true);
			motorRight.rotate(degrees);
			

		}
		
		motorLeft.close();
		motorRight.close();
	}
}
