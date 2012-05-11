package Robot;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.OdometryMsg;

import Navigation.DijkstraGood;
import Planner.Planner;

public class StateLookingForBlocks extends RobotState {
	private boolean destComplete;

//	private static ArrayList<Point2D.Double> tmpPoints;
//	private static int tmpCount = 0;
	public StateLookingForBlocks(Robot ri) {
		super(ri);
//		tmpPoints = new ArrayList<Point2D.Double>();
//		tmpPoints.add(new Point2D.Double(.5, 0));
//		tmpPoints.add(new Point2D.Double(.5, .5));
//		tmpPoints.add(new Point2D.Double(0, .5));
//		tmpPoints.add(new Point2D.Double(0, 0));
	}

	enum State {
		INIT, MOVING
	}

	@Override
	public void perform() {
		// for safety!
		robot.stopMoving();
		robot.speaker.speak("block search engaged");

		State state = State.INIT;
		while (true) {
			switch (state) {
			case INIT:
				if (robot.vision.canSeeBlock()) {
					state = State.MOVING;
					break;
//					robot.setStateObject(new StateInitial(robot));
//					return;
				}

				// can pick a place depending on known information
				// if we haven't explored a certain location, then pick a place
				// we haven't explored
				// or we can pick the location of a mapped block not retrieved
				// yet
				// or we can pick a place that is close by randomly
//				waypoint = robot.navigationMain.pickNewPoint();
//				waypoint = tmpPoints.get(tmpCount);
		
//				robot.driveToLocation(robot.planner.getCurrentBlockPosition());

				Point2D.Double robotpos = robot.odom.getPosition();
				Point2D.Double curpos = robot.planner.getCurrentBlockPosition();
				robot.log.info("looking from " + robotpos);
				robot.log.info("looking to " + curpos);
				ArrayList<Point2D.Double> shortestPath = DijkstraGood.getMyDijkstra(
						robot.navigationMain.getAndPlotVisGraph(robotpos), robotpos, 
						curpos, robot.log);
				log.info("DIJKSTRA COMPLETE");
				if (shortestPath == null) {
					// there is no route to this block. get the next block.
					robot.log.info("NO ROUTE TO THIS BLOCK");
					robot.planner.markCurrentBlockDone();
					robot.planner.nextClosestBlock();
				} else {
					shortestPath.remove(0); // we should already be at the first waypoint
					if (shortestPath.size() == 0) {
						// draw a direct line from current pos to block
						shortestPath.add(curpos);
					}
					robot.driveToLocations(shortestPath);
					state = State.MOVING;
				}
				break;
			case MOVING:
				if (robot.vision.canSeeBlock()) {
					robot.log.info("Robot can see block; immediate transition to StateMovingToBlock");
					//robot.stopMoving();
					// TODO clearly long; we should get the block location from the vision system
					robot.setStateObject(new StateMovingToBlock(robot));
					return;
				} else if (robot.doneMoving()) {
					// TODO stopMoving; security measure
					robot.log.info("Done moving");
					robot.stopMoving();
					state = State.INIT;
					// normally the block retrival code calls this. but if we got here,
					// we didn't find a block. just go to the next one. it's ok.
					robot.planner.nextClosestBlock();
//					tmpCount += 1;
//					tmpCount = tmpCount % 4;
					
				} else { // cantSeeBlock and notDoneMoving
					// We have already sent a message
					// this.robot.waypointDriver.stopMoving();
					// state = State.INIT;
				}
				break;
			}
		}
	}

}