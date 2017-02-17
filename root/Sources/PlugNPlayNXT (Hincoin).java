
// Plug-N-Play Autonomous Mindstorms NXT Driver
// Written by Alejandro Lucena
import java.util.Arrays;
import java.util.Random;

import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
public class HelloRobot 
{
	
	public static final int[] FORWARD = {-1,0};		// Drive forward 1 grid cell
	public static final int[] NOOP = {0,0}; 	    // No-Op.. do nothing
	public static final int[] GOAL = {0,3}; 		// Goal is application specific
	public static final int[] BACKWARD = {1,0}; 	// Drive backwards 1 grid cell
	public static final int[] RIGHT = {0,1}; 		// Drive right 1 grid cell
	public static final int[] LEFT = {0,-1}; 		// Drive left 1 grid cell
	
	/* THE FOLLOWING VARIABLES VARIABLE MUST CHANGE DEPENDING ON YOUR ENVIRONMENT AND ROBOT BUILD*/
	public static final double FLOORLENGTH = 705.692567624174429;  //Distance (millimeters) to cover per grid cell. 
	static DifferentialPilot pilot = new DifferentialPilot(43.20f,178.0f,Motor.B,Motor.C); // setup according to your own robot
	static UltrasonicSensor us = new UltrasonicSensor(SensorPort.S2); // setup according to your US sensor
	public static final double TURN_ANGLE = 82; // how much to turn on right turns, left turns. 
	public static final double SENSOR_ACCURACY = .9; // how accurate are the sensors? Should be determined empiracally
	public static final double MOVE_ACCURACY = .9;   // how accurate is robot motion? Should be determined empiracally


	


	
	public static double[][] initialize(int[][] grid) // given a grid, assign uniform distribution to all candidate grid cells
	{
		int total = grid.length * grid[0].length;
		for(int i =0 ; i < grid.length; ++i)
		{
			for(int j =0 ; j < grid[0].length; ++j)
			{
				if(grid[i][j]==1)
					--total; // keep track of available (navigable) grids
			}
		}
		double p[][] = new double[grid.length][grid[0].length];
		double nit = 1.0 / total; // normalize
		for(int i =0 ; i < grid.length; ++i)
		{
			for(int j=0 ; j < grid[0].length; ++j)
			{
				if(grid[i][j]==0)p[i][j] = nit;
			}
		}
		return p;
		
	}
	
	// Apply Theorem of Total probability to compute the prior P(X | U)
	// Variables lateral, and sideways define how far to move up/down (lateral) or right/left (sideways)
	// the double[][] p matrix is the grid we have for probabilities
	// int [][] grid is the environmental grid 
	// move_accuracy defines how accurate robot motion is (This is defined above as constant)
	public static double[][] move(int lateral, int sideways,double[][] p,int [][]grid,double move_accuracy)
	{
		double q[][] = new double[p.length][p[0].length]; // build an auxillary matrix to compute new probabilities
		for(double d[] : q){Arrays.fill(d,0.0);}
		for(int i =0 ; i < p.length; ++i)
		{
			for(int j =0; j < p[0].length; ++j)
			{
				
				// determine where we might have come  if we took action (lateral,sideways)
				int moveUp = i-lateral;
				int moveSide = j-sideways;
				
				double s = 0;
				
				// if where we might have come from is feasible ( no out of grid,out of bounds, or from an obstacle)
				if(moveUp >= 0 &&  moveUp < p.length && moveSide >=0 && moveSide < p[0].length && grid[i][j] != 1)
				{ 
					// add to the probability,  P(X|U,Desired_Previous_State) * P(Desired_Previous_State)
					s +=  move_accuracy * p[moveUp][moveSide];
				}
				// also add P(X|U,Other_State) * P(Other_State)
				s += (1.0-move_accuracy) * p[i][j];
				
				// assign to cell i,j the value after total probability is applied
				q[i][j]=s;
			}
		}
		return q; // return the updated probabilties matrix
	}
	
	// Apply Bayes Rule to compute the posterior P(X|Z)
	// sensor_measurement is either a 1 or 0 corresponding to whether or not there is an obstacle
	// directly ahead of us.
	// p[][] is the same probabilities matrix, sensor_grid[][] is what the sensor should 
	// ideally pickup on every grid cell if the sensor were perfect
	// sensor_accuracy is also defined as a constant above
	public static double[][] sense(int sensor_measurement,double p[][],  int sensor_grid[][], double sensor_accuracy)
	{
		double q[][] = new double[p.length][p[0].length];// auxillary matrix
		for(double d[] : q){Arrays.fill(d,0.0);}
		for(int i =0 ; i < p.length; ++i)
		{
			for(int j =0; j < p[0].length; ++j)
			{
				
				// compute P(Z|X) *P (X)
				int correct = (sensor_measurement == sensor_grid[i][j]) ? 1 : 0;
				//System.out.println("CORRECT = " + correct);
				double s =0;
				s += (correct * sensor_accuracy * p[i][j]);
				s += ((1-correct) * (1.0-sensor_accuracy) * p[i][j]);
				q[i][j]=s; // assign un-normalized value to the aux matrix
			}
		}
		// normalize the aux matrix
		double total =0;
		for(int i =0 ; i < p.length; ++i){for(int j=0; j < p[0].length; ++j){total += q[i][j];}}
		for(int i= 0; i < p.length; ++i){for(int j =0 ; j < p[0].length; ++j){q[i][j] /= total;}}
		return q;
	}
	
	// process a motion command
	public static void processCommand(int[] command) throws Exception
	{
		if(command == FORWARD) // drive forward
		{
			System.out.println("FORWARD");

			pilot.travel(FLOORLENGTH,true);
			while(pilot.isMoving())Thread.yield();
	
			
		}
		else if(command == BACKWARD)
		{
			System.out.println("BACKWARD");

			pilot.travel(-FLOORLENGTH,true);
			while(pilot.isMoving())Thread.yield();


		}
		// face right, go forward,and go back to face forward
		else if(command == RIGHT)
		{
			System.out.println("RIGHT");
			pilot.rotate(-TURN_ANGLE);
			pilot.travel(FLOORLENGTH,true);
			while(pilot.isMoving())Thread.yield();

			//Motor.B.stop();
			pilot.rotate(TURN_ANGLE);
		}
		// face left, go forward, and then turn back to face forward
		else if(command == LEFT)
		{
		
		//	pilot.travel(350.0,false);
			System.out.println("LEFT");
			pilot.rotate(TURN_ANGLE);
			pilot.travel(FLOORLENGTH,true);
			while(pilot.isMoving())Thread.yield();

			//Motor.C.stop();
			pilot.rotate(-(TURN_ANGLE-5));
		
		}
		// Motion was == NOOP
		else
		{
			//No-Op
			System.out.println("YOU MADE IT TO THE GOAL!");
		}
	}
	
	// pick the most likely position given the matrix
	public static int[] localize(double p[][])
	{
		int max_i = 0, max_j = 0;
		for(int i = 0 ; i < p.length; ++i)
		{
			for(int j = 0 ; j < p[0].length; ++j)
			{
				if(p[i][j] > p[max_i][max_j]){max_i = i; max_j = j;}
			}
		}
		return new int[]{max_i,max_j};
	}
	
	
	// robot explores the world
	// the last 2 parameters, num_movements, and percentage_random are meant to twiddle around with until
	// you find a configuration that suits you. Basically, num_movements will dictate how many
	// motions (and measurements) the robot will do in order to familiarize (and localize) itself in the grid
	
	
	//percentage_random is an exploration factor. This is a concept similar to reinforcment learning
	// by having the bot sometimes (with given percentage) take a random action, it can explore the world
	// a bit more and hopefully localize better. Use with caution.. too much exploration may ruin the effects of learning
	public static double[][] generateRandomMotions(int[][] grid, int[][] sensor_grid,double sensor_accuracy,double move_accuracy, int num_movements,double percentage_random) throws Exception
	{
		
		if(percentage_random > 1 || percentage_random < 0)
		{
			for(int i =0;i < 10; ++i)
				Sound.beep();
			throw new IllegalArgumentException("Percentage is invalid. Valid values are in the range [0,1])");
			
		}
		
		if(percentage_random == 0)percentage_random=-1;
		
		// begin with uniform distribution per grid cell. Of course, this can be modified
		// if you have information regarding starting state
		double p[][] = initialize(grid);
		double threshold = FLOORLENGTH/10; // values from the US sensor must be <= threshold to be considered obstacles
		Random r = new Random();
		
		for(int i = 0; i < num_movements; ++i)
		{
				
			int this_motion[] = FORWARD; double turn_back = 0;
			if(us.getDistance() <= threshold)
			{
				pilot.rotate(-TURN_ANGLE); this_motion = RIGHT; turn_back -= TURN_ANGLE;
				if(us.getDistance() <= threshold)
				{
					pilot.rotate(-TURN_ANGLE);this_motion=BACKWARD; turn_back -= TURN_ANGLE;
					if(us.getDistance() <= threshold) this_motion = LEFT; turn_back = TURN_ANGLE;
						pilot.rotate(-TURN_ANGLE);
				}
			}
			else 
			{
				int x = r.nextInt(101);
				if(x <= (percentage_random*100)) // with some chance, do another action
				{
					pilot.rotate(-TURN_ANGLE); this_motion = RIGHT; turn_back -= TURN_ANGLE;
					if(us.getDistance() <= threshold)
					{
						pilot.rotate(-TURN_ANGLE);this_motion=BACKWARD; turn_back -= TURN_ANGLE;
						if(us.getDistance() <= threshold) this_motion = LEFT; turn_back = TURN_ANGLE;
							pilot.rotate(-TURN_ANGLE);
					}
				}
				
			}
			
			pilot.rotate(-turn_back);
			processCommand(this_motion); // process the command
			
			p = move(this_motion[0],this_motion[1],p,grid,move_accuracy); // apply the move() method for
																		  // updating probabilities based on movement
			int sig = (us.getDistance() <= threshold) ? 1 : 0; // get sensor readings
			p = sense(sig,p,sensor_grid,sensor_accuracy); // use Bayes rule, sense(), to get the posterior
			
			
			
		}
		
		return p; // return updated probabilities matrix
		
	}
	
	// use Dynamic-Programming to create a policy for the grid
	// cost[][] is just in case some grid cells cost more than others.
	// If it "costs" the same to traverse all grid cells, initialize cost[][] as all 1's (or any positive value)
	public static int[][][] dp_solver(int grid[][], double cost[][])
	{
		int dp[][] = new int[grid.length][grid[0].length];
		int actions[][][] = new int[grid.length][grid[0].length][];
		for(int[] arr : dp) {Arrays.fill(arr,Integer.MAX_VALUE-1);}
		for(int[][] arr : actions)Arrays.fill(arr,NOOP);
		boolean did_change = true;
		while(did_change)
		{
			did_change = false;
			for(int i = 0; i < grid.length; ++i)
			{
				for(int j =0 ; j < grid[0].length; ++j)
				{
					if(grid[i][j]==1)continue; // no update for obstacles
					
					if(i == GOAL[0] && j == GOAL[1] && dp[i][j] != 0) // goal
					{
						dp[i][j] = 0; 
						did_change = true;
					}
					else
					{
						for(int q = -1; q <= 1; ++q)
						{
							for(int r = -1; r <= 1; ++r)
							{

								// at least on the them has to be 0
								if(r == 0 && q== 0) continue;
								if(r != 0 && q != 0) continue;


								int dx = i + q;
								int dy = j + r;
								if(dx < 0 || dx >=grid.length || dy < 0 || dy >= grid[0].length || grid[dx][dy] == 1) continue;
								if(dp[dx][dy]+cost[dx][dy] < dp[i][j])
								{
									dp[i][j] = dp[dx][dy]+1;
									actions[i][j] = (q == 0? (r == 1? RIGHT : LEFT) : (q == 1? BACKWARD : FORWARD));
									did_change=true;
								}
							}
						}
					}
					
				}
			}
			
			
			
		}
		return actions;
		
	}
	
	// given a grid of the world, this function computes what the sensor would ideally
	// predict in every grid cell, if the sensor were perfect. This is needed for the sense() function
	static int[][] make_sensor_grid(int grid[][])
	{
		int sensor_grid[][]= new int[grid.length][grid[0].length];
		for(int i =0 ; i < grid.length; ++i)
		{
			for(int j =0 ; j < grid[0].length; ++j)
			{
				
				
				
				// obstacles in the actual map are given -1 to denote they shouldn't be counted for
				if(grid[i][j]== 1)
				{
					sensor_grid[i][j] = -1;continue;
				}
				

				// the sensor grid has a 1 in location [x,y] if location [(x-1),y] (where x-1 is the location above (x,y) ) contains an obstacle
				int forward_x = i-1;
				if(forward_x < 0)continue;
				if(grid[forward_x][j]==1)
					
					sensor_grid[i][j]=1;
				
			}
		}
		return sensor_grid;
	}
	public static void main(String[] args) throws Exception
	{
			// map of the world. 1s are obstacles and 0s are open spaces
			// The grid can of course be either done manually or via a threshold-based Occupancy-Grid mapping algorithm

			// the top row of 1s and bottom row of 1s serves as a barrier for the move() and sense() algorithms. 
			// Since the world isn't cyclic, these barriers need to be in-place to prevent cyclic calculations
			
			int grid[][] = {{1,1,1,1}, // TOP BARRIER

								// Map of the world
							{1,1,0,0},		
							{1,0,0,1},
							{1,0,0,1},
							{1,1,0,0},
							{1,0,0,0},	
							   // End map of the world
							{1,1,1,1}}; // BOTTOM BARRIER
			
		// give additional costs to certain grid poses. Maybe being in certain
	    // locations is worse. For this example, all locations are equally "good" and cost the same
		double heuristics[][] = new double[grid.length][grid[0].length];
		for(double[] x : heuristics)Arrays.fill(x,1.0);
		
		
		int dp[][][] = dp_solver(grid,heuristics); // use dynamic-programming to make a policy
	
		int sensor_grid[][] = make_sensor_grid(grid); // compute the sensor grid. 1s represent that there exists
													 // an obstacle directly in front. -1s represent that a location
													// IS an obstacle. Borders are 1s so our move() method doesn't
													// try to search below the end of the grid or above the top
		
		pilot.setAcceleration(100); // tends to make turning and driving straigther...
									//replace this with PID controller ( future commit)

		// the robot moves around and gets a feel for its environment and consequently, localizes.
		// the returned matrix is a grid of probabilities
		double p[][] = generateRandomMotions(grid,sensor_grid,SENSOR_ACCURACY,MOVE_ACCURACY,4,.01);
		
		// find the most probable grid cell
		int[] locate = localize(p);
		
		// do a happy dance
		for(int i = 0; i < 5; ++i)
		{
			System.out.println("Localized: " + locate[0] + " " + locate[1]);
			Thread.sleep(1000);
		
		}
		
		// Navigate to the goal
		while(!(locate[0] == GOAL[0] && locate[1] == GOAL[1]))
		{
			int[] motion = dp[locate[0]][locate[1]];
			processCommand(motion);
			locate[0] += motion[0];
			locate[1] += motion[1];
			
		}
		// do another happy dance
		for(int i =0 ; i < 5; ++i)
			Sound.beep();
		
		}
		

}

