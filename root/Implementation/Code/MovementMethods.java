package leJOSEV3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;


public class MovementMethods {
	
	static EV3LargeRegulatedMotor motorLeft;
	static EV3LargeRegulatedMotor motorRight;
	static MovePilot movePilot;
	static Wheel wheelOne;
	static Wheel wheelTwo;
	static Chassis chassis;
	
	
	static double wheelDiameter;
	static double footPrint;
	static double wheelCircumference;
	
	public static void initialise() {
		
		wheelDiameter = 5.6;
		footPrint = 13.4;
		
		motorRight = new EV3LargeRegulatedMotor(MotorPort.D);
		motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);
		
//		motorLeft.setSpeed(0);
//		motorRight.setSpeed(0);
		
		wheelCircumference = MovementMethods.wheelDiameter*Math.PI;
		
		
		
	}
	
	
	public static void moveForward(long distance) {
		
		int measure = (int)Math.round((distance/wheelCircumference)*360.0);
		
		motorLeft.forward();
		motorRight.forward();
		motorLeft.rotate((int)measure);
		motorRight.rotate((int)measure);
		motorRight.stop();
		motorLeft.stop();
		
		motorLeft.close();
		motorRight.close();
		
	}
	
	public static void moveBackward(long distance) {
		
int measure = (int)Math.round((distance/wheelCircumference)*360.0);
		
		motorLeft.backward();
		motorRight.backward();
		motorLeft.rotate((int)measure);
		motorRight.rotate((int)measure);
		
		motorLeft.close();
		motorRight.close();
		
	}
	
	public static void rotateLeft(double degree) {
	
	}

	public static void rotateRight(double degree) {
		
	}
	
	
	
	public static void main(String args[]) {
		
		initialise();
		moveForward(20);
		
	//	Delay.msDelay(5000);
		
	//	moveBackward(15);
		//rotateLeft(85.0);
		//rotateRight(65.0);
		
	}
	
	

}
