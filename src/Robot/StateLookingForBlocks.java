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
		//robot.waypointDriver.stopMoving();

		Point2D.Double waypoint = null;
		State state = State.INIT;
		StateInitial exitState;

		loop: while (true) {

			switch (state) {
			case INIT:
				if (this.robot.canSeeBlock()) {
					exitState = new StateInitial(this.robot);
					break loop;
				}

				// can pick a place depending on known information
				// if we haven't explored a certain location, then pick a place
				// we haven't explored
				// or we can pick the location of a mapped block not retrieved
				// yet
				// or we can pick a place that is close by randomly
				waypoint = robot.navigationMain.pickNewPoint();

				state = State.MOVING;
				break;
			case MOVING:
				if (this.robot.canSeeBlock()) {
					exitState = new StateInitial(this.robot);
					break loop;
				} else if (this.robot.doneMoving()) {
//					TODO stopMoving; security measure
					state = State.INIT;
				} else { // cantSeeBlock and notDoneMoving
					this.robot.goToLocation(waypoint);
					//this.robot.waypointDriver.stopMoving();
//					state = State.INIT;
				}
				break;
			}
		}

		// TODO INSERT STOP MOVING
//		this.robot.waypointDriver.stopMoving();
		this.robot.setStateObject(exitState);
	}
	

}