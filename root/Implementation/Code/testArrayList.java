import java.util.ArrayList;

/**
 * Created by Jack on 27/03/2017.
 */
public class testArrayList {

    private static class Node{
        public Node parent;
        public int xPos;
        public int yPos;
        public float g;
        public float h;
        public float f;

        public Node(int xPos, int yPos){
            this.xPos = xPos;
            this.yPos = yPos;
        }
    }

    ArrayList<Node> path = new ArrayList<Node>();

    public static void moveFromPath(ArrayList<Node> path){
        int counter = 0;
        for(int i = 1;i < path.size();i++){
            if(path.get(i).xPos == path.get(i-1).xPos){
                //vertical movement
                if(path.get(i).yPos == path.get(i-1).yPos + 1){
                    //move north
                    switch(counter%4){
                        case 0: //moveForward();break;
                        case 1: //turnLeft();moveForward();counter--;break;
                        case 2: //turnLeft();turnLeft();moveForward();counter-=2;break;
                        case 3: //turnRight();moveForward();counter++;break;
                    }
                }
                else if(path.get(i).yPos == path.get(i-1).yPos - 1){
                    //move south
                    switch(counter%4){
                        case 0: //turnLeft();turnLeft();moveForward();counter+=2;break;
                        case 1: //turnRight();moveForward();counter++;break;
                        case 2: //moveForward();break;
                        case 3: //turnLeft();moveForward();counter--;break;
                    }
                }
            }
            else if(path.get(i).yPos==path.get(i-1).yPos){
                //horizontal movement
                if(path.get(i).xPos == path.get(i-1).xPos + 1){
                    //move east
                    switch(counter%4){
                        case 0: //turnLeft();turnLeft();moveForward();counter-=2;break;
                        case 1: //moveForward();break;
                        case 2: //turnLeft();moveForward();counter--;break;
                        case 3: //turnRight();moveForward();counter++;break;
                    }
                }
                else if(path.get(i).xPos == path.get(i-1).xPos - 1){
                    //move west
                    switch(counter%4){
                        case 0: //turnLeft();moveForward();counter--;break;
                        case 1: //turnLeft();turnLeft();moveForward();counter+=2;break;
                        case 2: //turnRight();moveForward();counter++;break;
                        case 3: //moveForward();break;
                    }
                }
            }
        }
    }
}
