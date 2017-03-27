import java.util.ArrayList;


public class Astar{


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





    public static int calcMan(Node a,Node b){
        return Math.abs(a.xPos-b.xPos)+Math.abs(a.yPos-b.yPos);
    }

    public static Node lowestInOpen(ArrayList<Node> a){
        if(a.isEmpty()){
            System.out.println("ArrayList is empty");
            return null;
        }
        else{
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
        }
        else if(grid[n.xPos][n.yPos]){
            return true;
        }
        else{
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

    public static void moveFromPath(ArrayList<Node> path){
        int counter = 0;
        Node current;
        Node next;
        for(int i = 0; i < path.size(); i++){
            if(i == path.size()-1){
                System.out.println("Journey finished");
            }
            else{
                current = path.get(i);
                next = path.get(i+1);
                if(current.xPos == next.xPos){
                    if(current.yPos + 1 == next.yPos){
                        //move north
                        if(counter%4==0){
                            //moveForward();
                        }
                        else if(counter%4==1){
                            //turnLeft();
                            //moveForward();
                        }
                        else if(counter%4==2){
                            //turnAround();
                            //moveForward();
                        }
                        else if(counter%4==3){
                            //turnRight();
                            //moveForward();
                        }
                    }
                    else if(current.yPos - 1 == next.yPos){
                        //move south
                        if(counter%4==0){
                            //turnAround();
                            //moveForward();
                        }
                        else if(counter%4==1){
                            //turnRight();
                            //moveForward();
                        }
                        else if(counter%4==2){
                            //moveForward();
                        }
                        else if(counter%4==3){
                            //turnLeft();
                            //moveForward();
                        }
                    }
                }
                else if(current.yPos==current.yPos){
                    if(current.xPos + 1 == next.xPos){
                        //move east
                        if(counter%4==0){
                            //turnRight();
                            //moveForward();
                        }
                        else if(counter%4==1){
                            //moveForward();
                        }
                        else if(counter%4==2){
                            //turnLeft();
                            //moveForward();
                        }
                        else if(counter%4==3){
                            //turnAround();
                            //moveForward();
                        }
                    }
                    else if(current.xPos - 1 == next.xPos){
                        //move west
                        if(counter%4==0){
                            //turnLeft();
                            //moveForward();
                        }
                        else if(counter%4==1){
                            //turnAround();
                            //moveForward();
                        }
                        else if(counter%4==2){
                            //turnRight
                            //moveForward();
                        }
                        else if(counter%4==3){
                            //moveForward();
                        }
                    }
                }
            }
        }
    }



















    public static void main(String[] args){
        Node one = new Node(0,0);
        Node two = new Node(0,3);
        boolean grid[][] = new boolean[][]{
                {true,false,false,true},
                {true,true,true,true},
                {true,true,true,true},
                {true,true,true,true},
        };
        for(Node n: findPath(one,two,grid)){
            System.out.println(n.xPos+" , "+n.yPos);
        }

    }



}
