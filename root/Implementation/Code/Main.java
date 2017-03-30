package leJOSEV3;

import java.util.ArrayList;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.SampleProvider;

public class Main {
    
    //Node Class
    private static class Node{
        public Node parent;
        public int xPos;
        public int yPos;
        public float g;
        public float h;
        public float f;
        
        //Constructor for the private Node Class
        public Node(int xPos, int yPos){
            this.xPos = xPos;
            this.yPos = yPos;
        }
    }
    
    static EV3LargeRegulatedMotor motorLeft;
    static EV3LargeRegulatedMotor motorRight;
    
    static EV3TouchSensor impactSensor;
    static EV3GyroSensor gyroSensor;
    static EV3ColorSensor colorSensor;
    
    static SampleProvider gyroProvider;
    static float[] gyroSamples;
    static float[] angle = {0.0f};
    static float gyroTacho;
    static double currentAngle;
    static double errorAngle;
    static double derivativeAngle;
    static double errorLastAngle;
    static double correctionAngle;
    static double kP;
    static double kD;
    
    static SampleProvider impactProvider;
    static float[] impactSamples;
    static float impactValue;
    
    static SampleProvider colourProvider;
    static SampleProvider obstacleProvider;
    static float[] lightSamples;
    
    static double wheelCircumference;
    static double wheelDistance;
    
    static boolean[] lineValues;
    
    static String colourGoal;
    
    static int tileReached;
    
    static boolean[][] task1task2GoingArena;
    static boolean[][] task1ReturningArena;
    static boolean[][] task2ReturningArena;
    
    //Initializer Method
    public static void init(){
        colorSensor = new EV3ColorSensor(SensorPort.S4);
        motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);
        motorRight = new EV3LargeRegulatedMotor(MotorPort.D);
        
        motorLeft.setSpeed(90);
        motorRight.setSpeed(90);
        motorLeft.setAcceleration(2000);
        motorRight.setAcceleration(2000);
        
        gyroSensor = new EV3GyroSensor(SensorPort.S1);
        //gyroSamples = gyroSensor.getAngleMode();
        gyroTacho = 0;
        
        impactSensor = new EV3TouchSensor(SensorPort.S3);
        impactProvider = impactSensor.getTouchMode();
        impactSamples = new float[1];
        
        colourProvider = colorSensor.getRedMode();
        obstacleProvider = colorSensor.getRedMode();
        lightSamples = new float[1];
        
        wheelCircumference = 2*2.75*Math.PI;
        wheelDistance = 12.35;
        
        lineValues = new boolean[]{true, false, true, false, false, true, true, false, true, true, false, false, true, true, true, false, true, true, true, false, false, true, true, true, false, false, false, true, true, true, true, false, false, false};
    }
    
    //Process Light
    public static boolean processLight(float[] sampleArray){
        colourProvider.fetchSample(sampleArray,0);
        if(sampleArray[0] < 0.6) {
            return true;
        }else{
            return false;
        }
    }
    
    //Process Obstacle
    public static String processObstacle(float[] sampleArray) {
        obstacleProvider.fetchSample(sampleArray, 0);
        
        if(sampleArray[0] < 0.5) {
            return "GREEN";
        }else {
            return "RED";
        }
    }
    
    //Read Light
    public static float readLight(float[] sampleArray) {
        colourProvider.fetchSample(sampleArray, 0);
        return sampleArray[0];
    }
    
    //Read Angle
    public static float readAngle(float[] sampleArray) {
        gyroProvider.fetchSample(sampleArray, 0);
        return sampleArray[0];
    }
    
    //Read Impact
    public static float readImpact(float[] sampleArray) {
        impactProvider.fetchSample(sampleArray, 0);
        return sampleArray[0];
    }
    
    //Move Forward
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
    
    //Move Backward
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
    
    //Rotate left by 90 degrees
    public static void rotateLeft90(){
        motorLeft.rotate(-202,true);
        motorRight.rotate(202);
    }
    
    //Rotate right by 90 degrees
    public static void rotateRight90(){
        motorLeft.rotate(202,true);
        motorRight.rotate(-202);
    }
   
    //Rotate Right by 45 degrees
    public static void rotateRight45() {
        motorLeft.rotate(101,true);
        motorRight.rotate(-101);
    }
    
    //Rotate Left by 90 degrees
    public static void rotateLeft45() {
        motorLeft.rotate(-101, true);
        motorRight.rotate(101);
    }
    
    //Localization Method
    public static int localise(boolean[] lineValues,float sensorWorks,float motorWorks){
        boolean lightValue;
        double totalProb;
        double[] lineProbs = new double[lineValues.length];
        
        for(int i = 0; i < lineProbs.length;i++){
            lineProbs[i] = 1/(float) lineValues.length;
        }
        
        while(true){
            lightValue = processLight(lightSamples);
            
            /*Update*/
            for(int i=0;i<lineProbs.length;i++){
                if(lightValue == lineValues[i]) {
                    lineProbs[i]=(sensorWorks*lineProbs[i]);
                }else{
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
            if(maxVal > 0.62) {
                motorLeft.stop();
                motorRight.stop();
                tileReached = maxInd+1;
                return (tileReached);
            }
        }
    }
    
    //A* Path Planning Algorithm...
    public static int calcMan(Node a,Node b){
        return Math.abs(a.xPos-b.xPos)+Math.abs(a.yPos-b.yPos);
    }
    
    public static Node lowestInOpen(ArrayList<Node> a){
        if(a.isEmpty()){
            System.out.println("ArrayList is empty");
            return null;
        }else{
            Node tmp = a.get(0);
            float i = a.get(0).f;
            for(Node n:a){
                if(n.f<i){
                    tmp = n;
                }
            }
            return tmp;
        }
    }
    
    public static void evaluate(Node subject,Node start,Node end){
        subject.g = calcMan(start,subject);
        subject.h = calcMan(subject,end);
        subject.f = subject.g + subject.h;
    }
    
    public static Node calculateStart(int tileReached) {
        int nodeX;
        int nodeY;
        if(tileReached < 15) {
            nodeX = 1;
            nodeY = 1;
        }else if(tileReached > 15 && tileReached < 30) {
            nodeX = 2;
            nodeY = 2;
        }else {
            nodeX = 2;
            nodeY = 2;
        }
        Node startNode = new Node(nodeX, nodeY);
        return startNode;
    }
    
    public static boolean checkNode(ArrayList<Node> open, ArrayList<Node> closed, Node n,boolean grid[][]){
        for(Node o:open){
            if(n.xPos == o.xPos && n.yPos == o.yPos && o.f <= n.f){
                return false;
            }
        }
        for(Node c:closed){
            if(n.xPos == c.xPos && n.yPos == c.yPos && c.f <= n.f){
                return false;
            }
        }
        if(n.xPos<0 || n.yPos<0 || n.xPos >= grid.length || n.yPos >= grid[0].length){
            return false;
        }else if(grid[n.xPos][n.yPos]){
            return true;
        }else{
            return false;
        }
    }
    
    public static ArrayList<Node> findPath(Node start, Node end,boolean[][] grid){
        Node q;
        Node north;
        Node east;
        Node south;
        Node west;
        ArrayList<Node> openList = new ArrayList<Node>();
        ArrayList<Node> closedList = new ArrayList<Node>();
        ArrayList<Node> adjacent = new ArrayList<Node>();
        openList.add(start);
        start.f = 0;
        while(!openList.isEmpty()){
            q = lowestInOpen(openList);
            openList.remove(q);
            north = new Node(q.xPos,q.yPos+1);
            adjacent.add(north);
            east = new Node(q.xPos+1,q.yPos);
            adjacent.add(east);
            south = new Node(q.xPos,q.yPos-1);
            adjacent.add(south);
            west = new Node(q.xPos-1,q.yPos);
            adjacent.add(west);
            for(Node n:adjacent){
                n.parent = q;
                evaluate(n,start,end);
                if(n.xPos == end.xPos && n.yPos == end.yPos){
                    ArrayList<Node> path = new ArrayList<Node>();
                    ArrayList<Node>reversePath = new ArrayList<Node>();
                    Node a = n;
                    path.add(a);
                    while(a.xPos!=start.xPos||a.yPos!=start.yPos){
                        a = a.parent;
                        path.add(a);
                    }
                    System.out.println("PATH FOUND:");
                    for(int i = path.size()-1;i>=0;i--){
                        reversePath.add(path.get(i));
                    }
                    return reversePath;
                }
                if(checkNode(openList,closedList,n,grid)){
                    openList.add(n);
                }
            }
            closedList.add(q);
            adjacent.clear();
        }
        return null;
    }
    
    //On reaching outside goal
    // - Move into goal, 
    // - Signal arrival, 
    // - Reverse out of the goal
    // - turn right by 90 degrees
    // - Return the Color read within the goal
    public static String senseBeepReverse(){
        moveForward(18.0);
        while(readImpact(impactSamples) != 1) {
            moveForward(1.0);
        }
        Sound.beep();
        motorLeft.stop();
        motorRight.stop();
        colourGoal = processObstacle(lightSamples);
        moveBackward(23.0);
        rotateRight90();
        
        return colourGoal;
    }
    
    //Move to Grid 
    public static void moveToGrid(int tileReached){
        if(tileReached > 17){
            int val = tileReached - 17;
            moveBackward(val*2.0);
        }else if(tileReached < 17){
            int val = 17 - tileReached;
            moveForward(val*2.0);
        }
    }
    
    //Move along the path to the Goal
    public static void moveFromPath(ArrayList<Node> path){
        int counter = 4000;
        double dist = 16;
        for(int i = 1;i < path.size();i++){
            System.out.println(path.get(i).xPos+" , "+path.get(i).yPos);
            if(path.get(i).xPos == path.get(i-1).xPos){
                //vertical movement
                if(path.get(i).yPos == path.get(i-1).yPos + 1){
                    //move north
                    switch(counter%4){
                        case 0: moveForward(dist);break;
                        case 1: rotateLeft90();moveForward(dist);counter--;break;
                        case 2: rotateLeft90();rotateLeft90();moveForward(12);counter-=2;break;
                        case 3: rotateRight90();moveForward(dist);counter++;break;
                    }
                }else if(path.get(i).yPos == path.get(i-1).yPos - 1){
                    //move south
                    switch(counter%4){
                        case 0: rotateLeft90();rotateLeft90();moveForward(dist);counter+=2;break;
                        case 1: rotateRight90();moveForward(dist);counter++;break;
                        case 2: moveForward(dist);break;
                        case 3: rotateLeft90();moveForward(dist);counter--;break;
                    }
                }
            }else if(path.get(i).yPos==path.get(i-1).yPos){
                //horizontal movement
                if(path.get(i).xPos == path.get(i-1).xPos + 1){
                    //move east
                    switch(counter%4) {
                        case 0: rotateRight90();moveForward(dist);counter++;break;
                        case 1: moveForward(dist);break;
                        case 2: rotateLeft90();moveForward(dist);counter--;break;
                        case 3: rotateRight90();rotateRight90();moveForward(dist);counter+=2;break;
                            
                    }
                }else if(path.get(i).xPos == path.get(i-1).xPos - 1){
                    //move west
                    switch(counter%4){
                        case 0: rotateLeft90();moveForward(dist);counter--;break;
                        case 1: rotateLeft90();rotateLeft90();moveForward(dist);counter+=2;break;
                        case 2: rotateRight90();moveForward(12);counter++;break;
                        case 3: moveForward(dist);break;
                    }
                }
            }            
        }
    }
    
    public static void main(String[] args) {
        //Task 1 and Task 2 reaching the goal and stopping just outside the goal
        task1task2GoingArena = new boolean[][] {    
            {false, false, false, false, false, false, false, false},
            {false, true, true, true, true, true, true, true},
            {false, false, false, false, false, false, false, true},
            {false, false, false, true, true, false, true, true},
            {false, false, false, false, false, true, true, true},
            {false, false, false, true, true, true, true, true},
            {false, false, false, false, false, false, false, false},
            {false, false, false, false, false, false, false, false}
        };
        
        //Task 1 returning back to the starting point from the same route and on arrival signal
        task1ReturningArena = new boolean[][] {     
            {false, false, false, false, false, false, false, false},
            {false, true, true, true, true, true, true, true},
            {false, false, false, false, false, false, false, true},
            {false, false, false, true, true, false, true, true},
            {false, false, false, false, false, true, true, true},
            {false, false, false, true, true, true, true, true},
            {false, false, false, false, false, false, false, false},
            {false, false, false, false, false, false, false, false}
        };
        
        //Task 2 returning back to the starting point from a new route and on arrival signal
        task2ReturningArena = new boolean[][] {
            {false, false, false, false, false, false, false, false},
            {false, false, false, false, false, false, false, false},
            {false, false, false, false, false, false, false, false},
            {false, false, false, false, false, false, false, false},
            {true, true, true, true, true, false, false, true},
            {true, true, true, true, true, true, true, true},
            {true, true, true, true, true, true, true, false},
            {true, true, true, true, true, true, false, false},
        };
        
        //Initialize the variables
        init();
        
        //On a press of a button start to initialzie
        Button.waitForAnyPress();
        int i = localise(lineValues,(float)0.95,(float)0.95);
        moveToGrid(i);
        rotateLeft45();
        
        
        //Task 1
        //From point of initialization, start A* path planning
        Node start1 = new Node(2,2);
        Node end1 = new Node(4,7);
        ArrayList<Node> path = findPath(start1, end1, task1task2GoingArena);
        for(Node n : path){
            System.out.println(n.xPos+" , "+n.yPos);
        }
        moveFromPath(path);
        rotateLeft90();
        rotateLeft90();
        //turn(180);
        ArrayList<Node> reversePath = new ArrayList<Node>();
        for(int a = path.size()-1; a >= 0; a--) {
            reversePath.add(path.get(a));
        }
        moveFromPath(reversePath);
        rotateLeft45();
        while(readImpact(impactSamples) != 1) {
            moveForward(1.0);
        }
        
       /*
        //Task 2
        //on arrival of goal beep and reverse out and turn right by 90 degrees
        senseBeepReverse();
        
        if (colourGoal == "Red"){
            //Path plan back from a new route back to the starting position
            Node start2 = new Node(4,7);
            Node end2 = new Node(2,2);
            ArrayList<Node> path2 = findPath(start2, end2, task2ReturningArena);
            for(Node n : path2){
                System.out.println(n.xPos+" , "+n.yPos);
            }
            moveFromPath(path2);
        }else if(colourGoal == "Green"){
            //Path plan back from a new route back to the starting position
            Node start2 = new Node(4,7);
            Node end2 = new Node(2,2);
            ArrayList<Node> path2 = findPath(start2, end2, task2ReturningArena);
            for(Node n : path2){
                System.out.println(n.xPos+" , "+n.yPos);
            }
            moveFromPath(path2);
        }
        
        //while no Obstacle keeping moving, otherwise, singal starting position reached, and stop motors
        while(readImpact(impactSamples) != 1) {
            moveForward(1.0);
        }
        Sound.beep();
        motorLeft.stop();
        motorRight.stop();
        */
    }
}
