package edu.illinois.mitra.demo.projectapp;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.Stack;
import edu.illinois.mitra.starl.comms.RobotMessage;
import edu.illinois.mitra.starl.functions.RandomLeaderElection;
import edu.illinois.mitra.starl.gvh.GlobalVarHolder;
import edu.illinois.mitra.starl.interfaces.LeaderElection;
import edu.illinois.mitra.starl.interfaces.LogicThread;
import edu.illinois.mitra.starl.objects.*;
import edu.illinois.mitra.starl.motion.*;
import edu.illinois.mitra.starl.motion.MotionParameters.COLAVOID_MODE_TYPE;

public class ProjectApp extends LogicThread {
    private static final boolean RANDOM_DESTINATION = false;
    public static final int ARRIVED_MSG = 22;
    private static final MotionParameters DEFAULT_PARAMETERS = MotionParameters.defaultParameters();
    private volatile MotionParameters param = DEFAULT_PARAMETERS;
    // this is an ArrayList of HashMap. Each HashMap element in the array will contain one set of waypoints
    final ArrayList<HashMap<String, ItemPosition>> destinations = new ArrayList<>();
    private int numSetsWaypoints = 4;
    int robotIndex;

    private Set<String> hasBeenLeader;

    // used to find path through obstacles
    Stack<ItemPosition> pathStack;
    RRTNode kdTree = new RRTNode();

    //obsList is a local map each robot has, when path planning, use this map
    ObstacleList obsList;

    //obEnvironment is the physical environment, used when calculating collisions
    ObstacleList obEnvironment;

    ItemPosition currentDestination, currentDestination1, preDestination;

    private LeaderElection le;
    //	private String leader = null;
    private boolean iamleader = false;

    private enum Stage {
        //        PICK, GO, DONE, ELECT, HOLD, MIDWAY
        START, ELECT, FINDPATH, HOLD, LEADER_MOVE, AVOID, DONE
    };

    //start in the elect stage
    private Stage stage = Stage.START;

    public ProjectApp(GlobalVarHolder gvh) {
        super(gvh);
        for(int i = 0; i< gvh.gps.getPositions().getNumPositions(); i++){
            if(gvh.gps.getPositions().getList().get(i).name == name){
                robotIndex = i;
                break;
            }

        }

        // instantiates each HashMap object in the array
        for(int i = 0; i < numSetsWaypoints; i++) {
            destinations.add(new HashMap<String, ItemPosition>());
        }
        le = new RandomLeaderElection(gvh);


        MotionParameters.Builder settings = new MotionParameters.Builder();
//		settings.ROBOT_RADIUS(400);
        settings.COLAVOID_MODE(COLAVOID_MODE_TYPE.USE_COLBACK);
        MotionParameters param = settings.build();
        gvh.plat.moat.setParameters(param);

        // this loop gets add each set of waypoints i to the hashmap at destinations(i)
        for(ItemPosition i : gvh.gps.getWaypointPositions()) {
            String setNumStr = i.getName().substring(0,1);
            int setNum = Integer.parseInt(setNumStr);
            destinations.get(setNum).put(i.getName(), i);
        }


        //point the environment to internal data, so that we can update it
        obEnvironment = gvh.gps.getObspointPositions();

        //download from environment here so that all the robots have their own copy of visible ObstacleList
        obsList = gvh.gps.getViews().elementAt(robotIndex) ;

        hasBeenLeader = new HashSet<String>();

        gvh.comms.addMsgListener(this, ARRIVED_MSG);
    }

    @Override
    public List<Object> callStarL() {
        int i = 0;
        while(true) {


            ///do not think we need this because the map never changes?
            obEnvironment.updateObs();

            obsList.updateObs();
            if((gvh.gps.getMyPosition().type == 0) || (gvh.gps.getMyPosition().type == 1)){

                switch(stage) {
                    case START:
//                        RobotMessage inform = new RobotMessage("ALL", name, ARRIVED_MSG, "START " + name);
//                        gvh.comms.addOutgoingMessage(inform);
                        System.out.print(name + " is in START.\n");
//                        le.cancel();
                        le.elect();
                        stage = Stage.ELECT;
                        break;

                    case ELECT:
                        System.out.print(name + " is in ELECT.\n");
                        if(le.getLeader() != null) {
                            if (!hasBeenLeader.contains(le.getLeader())) {
                                hasBeenLeader.add(le.getLeader());
                                System.out.print("leader is " + le.getLeader() + "\n");
                                ///What does this do???
//                            results[1] = le.getLeader();
                                if (le.getLeader().equals(name)) {
                                    stage = Stage.FINDPATH;
                                } else {
                                    stage = Stage.HOLD;
                                }
                            }
                            else if (hasBeenLeader.size() >= gvh.id.getParticipants().size()){
                                System.out.print("leader is " + le.getLeader() + "\n");
                                ///What does this do???
//                            results[1] = le.getLeader();
                                if (le.getLeader().equals(name)) {
                                    stage = Stage.FINDPATH;
                                } else {
                                    stage = Stage.HOLD;
                                }
                            }
                            else {
                                RobotMessage inform = new RobotMessage("ALL", name, ARRIVED_MSG, "Reelect");
                                gvh.comms.addOutgoingMessage(inform);
                                stage = Stage.START;
                            }
                        }
                        else {
                            stage = Stage.ELECT;
                        }
                        break;

                    case FINDPATH:
                        System.out.print(name + " is in FINDPATH.\n");
                        //check to see if all points have been visited, or whether we should move on to the next set
                        if(destinations.get(i).isEmpty()) {
                            if(i+1 >= numSetsWaypoints) {
                                stage = Stage.DONE;
                            }
                            else {
                                i++;
                                //reset leader counter
                                hasBeenLeader = new HashSet<String>();
                            }
                        }
                        else {
//                            RobotMessage informleader = new RobotMessage("ALL", name, 21, le.getLeader());
//                            gvh.comms.addOutgoingMessage(informleader);

//                            iamleader = le.getLeader().equals(name);
//                            iamleader = true;

                            currentDestination = getRandomElement(destinations.get(i));

                            ///get current position and make a RRTNode with it
                            RRTNode path = new RRTNode(gvh.gps.getPosition(name).x, gvh.gps.getPosition(name).y);
                            ///pathStack := make route from Node to Destination, avoiding obEnvironment(complete world map less moving robots)
                            //TODO add dynamic objects plus leader's path to obEnvironment
                            pathStack = path.findRoute(currentDestination, 5000, obEnvironment, 5000, 3000, (gvh.gps.getPosition(name)), (int) (gvh.gps.getPosition(name).radius*0.8));

                            ///stopNode is destination
                            kdTree = RRTNode.stopNode;

                            //wait when can not find path
                            // TODO wait for path to clear
                            if(pathStack == null){
                                stage = Stage.HOLD;
                            }
                            else{
                                preDestination = null;
                                stage = Stage.LEADER_MOVE;
                            }
                        }
                        break;


                    case LEADER_MOVE:
                        System.out.print(name + " is in LEADER_MOVE.\n");
                        if(!gvh.plat.moat.inMotion) {
                            if(pathStack == null){
                                stage = Stage.HOLD;
                                // if can not find a path, wait for obstacle map to change
                                break;
                            }
                            if(!pathStack.empty()){
                                //if did not reach last midway point, go back to path planning
                                if(preDestination != null){
                                    if((gvh.gps.getPosition(name).distanceTo(preDestination) > param.GOAL_RADIUS)){
                                        pathStack.clear();
                                        stage = Stage.FINDPATH;
                                        break;
                                    }
                                    preDestination = pathStack.peek();
                                }
                                else{
                                    preDestination = pathStack.peek();
                                }
                                ItemPosition goMidPoint = pathStack.pop();
                                gvh.plat.moat.goTo(goMidPoint, obsList);
                                stage = Stage.LEADER_MOVE;
                            }
                            else{
                                //not going to waypoint just a place? maybe???
                                if((gvh.gps.getPosition(name).distanceTo(currentDestination) > param.GOAL_RADIUS)) {
                                    pathStack.clear();
                                    RRTNode path = new RRTNode(gvh.gps.getPosition(name).x, gvh.gps.getPosition(name).y);
                                    pathStack = path.findRoute(currentDestination, 5000, obEnvironment, 5000, 3000, (gvh.gps.getPosition(name)), (int) (gvh.gps.getPosition(name).radius*0.8));
                                }
                                else{
                                    if(currentDestination != null) {
                                        destinations.get(i).remove(currentDestination.getName());
                                        //inform other node that the waypoint was cleared
                                        System.out.print(name + " sending Cleared message\n");
                                        System.out.print("Cleared waypoint " + currentDestination.getName() + "\n");
                                        RobotMessage clearedInform = new RobotMessage("ALL", name, ARRIVED_MSG, "Cleared`" + i + "`" + currentDestination.getName());
                                        gvh.comms.addOutgoingMessage(clearedInform);
                                        stage = Stage.START;
                                    }
                                }
                            }
                        }
                        break;

//                    case GO:
//                        if(!gvh.plat.moat.inMotion) {
//                            //if stopped and dest is not null then remove dest from waypoint set
//                            if(currentDestination != null) {
//                                destinations.get(i).remove(currentDestination.getName());
//                            }
////                            RobotMessage inform = new RobotMessage("ALL", name, ARRIVED_MSG, currentDestination.getName());
////                            gvh.comms.addOutgoingMessage(inform);
//                            stage = Stage.ELECT;
//                        }
//
//                        break;
                    case HOLD:
                        System.out.print(name + " is in HOLD.\n");
                        //check to see if all points have been visited, or whether we should move on to the next set
                        if(destinations.get(i).isEmpty()) {
                            if(i+1 >= numSetsWaypoints) {
                                stage = Stage.DONE;
                            }
                            else {
                                i++;
                                //reset leader counter
                                hasBeenLeader = new HashSet<String>();
                            }
                        }
                        ///If close to leader move away, pick new dest by moving into Stage.PICK else stop
                        if(gvh.gps.getMyPosition().distanceTo(gvh.gps.getPosition(le.getLeader())) < 1000 ) {
                            stage = Stage.AVOID;
                        }
                        else {
                            gvh.plat.moat.motion_stop();
                        }
                        break;
                    case AVOID:
                        System.out.print(name + " is in AVOID.\n");
                        //go 1/8 of the way to currentDestination then stop and go to Stage.HOLD
                        currentDestination = gvh.gps.getPosition(le.getLeader());
                        currentDestination1 = new ItemPosition(currentDestination);
                        int newx, newy;
                        if(gvh.gps.getPosition(name).getX() < currentDestination1.getX()) {
                            newx = gvh.gps.getPosition(name).getX() - currentDestination1.getX()/4;
                        }
                        else {
                            newx = gvh.gps.getPosition(name).getX() + currentDestination1.getX()/4;
                        }
                        if(gvh.gps.getPosition(name).getY() < currentDestination1.getY()) {
                            newy = gvh.gps.getPosition(name).getY() - currentDestination1.getY()/4;
                        }
                        else {
                            newy = gvh.gps.getPosition(name).getY() + currentDestination1.getY()/4;
                        }
                        currentDestination1.setPos(newx, newy, (currentDestination1.getAngle()));
                        //				currentDestination1.setPos(currentDestination);
                        ///What does obsList do here?
                        gvh.plat.moat.goTo(currentDestination1, obsList);
                        stage = Stage.HOLD;
                        break;

                    case DONE:
                        System.out.print(name + " is in DONE.\n");
                        gvh.plat.moat.motion_stop();
                        return null;
                }
            }
            else {
                currentDestination = getRandomElement(destinations.get(i));
                gvh.plat.moat.goTo(currentDestination, obsList);
            }
            sleep(100);
        }
    }

    @Override
    protected void receive(RobotMessage m) {
//        String posName = m.getContents(0);
//        System.out.println("message test receive from :" + m.getFrom());
//        if(destinations.get(0).containsKey(posName))
//            destinations.get(0).remove(posName);
        System.out.print(name + " got message : " + m.getContents(0) + "\n");
        switch(stage) {
            case HOLD:
                if (m.getContents(0).equals("Cleared")) {
                    System.out.println(Integer.parseInt(m.getContents(1)) + " " + m.getContents(2));
                    //remove cleared waypoint from list
                    destinations.get(Integer.parseInt(m.getContents(1))).remove(m.getContents(2));
                    //return to START to get next waypoint
                    stage = Stage.START;
                }
                else if (m.getContents(0).equals("Reelect")){
                    System.out.println(name + " is going to START for reelection");
                    stage = Stage.START;
                }
                break;
        }
    }

       /* if(currentDestination.getName().equals(posName)) {
            gvh.plat.moat.cancel();
            stage = Stage.PICK;
        }*/

    private static final Random rand = new Random();

    @SuppressWarnings("unchecked")
    private <X, T> T getRandomElement(Map<X, T> map) {
        if(RANDOM_DESTINATION)
            return (T) map.values().toArray()[rand.nextInt(map.size())];
        else
            return (T) map.values().toArray()[0];
    }
}
