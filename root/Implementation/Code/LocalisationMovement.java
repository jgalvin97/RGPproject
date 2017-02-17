package leJOSEV3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;


public class LocalisationMovement {

	static EV3LargeRegulatedMotor motorLeft;
	static EV3LargeRegulatedMotor motorRight;
	static MovePilot movePilot;
	static Wheel wheelOne;
	static Wheel wheelTwo;
	static Chassis chassis;

	static double wheelDiameter;
	static double footPrint;
	static double wheelCircumference;

	public static void main(String args[]){
		EV3LargeRegulatedMotor motorLeft =new EV3LargeRegulatedMotor(brick.getPort("B"));
		EV3LargeRegulatedMotor motorRight=new EV3LargeRegulatedMotor(brick.getPort("C"));

		LocalisationInitialisation()
		MovementInitialisation();
	}

	//Given a board, assign uniform distribution to all candidate board cells
	public static void LocalisationInitialisation(int[][] board){
		int total = board.length * board[0].length;
		for(int i = 0; i < board.length; ++i){
			for(int j = 0; j < board.length; ++j){
				//Keep track of avliable boards which can be navigated still.
				if(board[i][j] == 1){
					--total;
				}
			}
		}
		double position[][] = new double[board.length][board[0].length];
		for(int i = 0; i < board.length; ++i){
			for(int j = 0; j < board.length; ++j){
				if(board[i][j] == 0){
					 moveForward(20);
					// position[i][j] = 1.0 / total;
				}
			}
		}
	}

	public static void Sense(){
		
	}

	public static void ProcessCommand(){
		
	}

	public static void localise(double position[][]){
		int imax = 0;
		int jmax = 0;
		for(int i = 0; i < board.length; ++i){
			for(int j = 0; j < board.length; ++j){
				if(position[i][j] > position[imax][jmax]){
					imax = i;
					jmax = j;
				}
			}
		}
	}

	//Compute the cost of a grid cell
	public static void dynamicProgrammingSolver(){
		
	}

	// given a grid of the world, this function computes what the sensor would ideally
	// predict in every grid cell, if the sensor were perfect. 
	//This is needed for the sense() function
	public static void makeSensorGrid(){

	}

	//Move() and GenerateRandomMotions() method below
	public static void MovementInitialisation() {
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
}