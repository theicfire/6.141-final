package Robot;

import java.awt.geom.Point2D;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.OdometryMsg;

public class StateLookingForBlocks extends RobotState {
	private boolean destComplete;

	public StateLookingForBlocks(Robot ri) {
		super(ri);
		// TODO Auto-generated constructor stub
	}

	enum State {
		INIT, MOVING
	}

	@Override
	public void perform() {
		// for safety!
		robot.stopMoving();

		Point2D.Double waypoint = null;
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
				waypoint = robot.navigationMain.pickNewPoint();
				robot.driveToLocation(waypoint);
				state = State.MOVING;
				break;
			case MOVING:
				if (robot.vision.canSeeBlock()) {
					robot.log.info("Robot can see block; stopping and going to StateMovingToBlock");
					robot.stopMoving();
					// TODO clearly long; we should get the block location from the vision system
					robot.setStateObject(new StateMovingToBlock(robot));
					return;
				} else if (robot.doneMoving()) {
					// TODO stopMoving; security measure
					robot.log.info("Done moving");
					robot.stopMoving();
					state = State.INIT;
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