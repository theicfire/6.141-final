package Robot;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.OdometryMsg;
import Navigation.DijkstraGood;
import Planner.Planner;

public class StateBumpLookingForBlocks extends RobotState {
	private boolean destComplete;
	public StateBumpLookingForBlocks(Robot ri) {
		super(ri);
	}

	enum State {
		MOVING
	}

	@Override
	public void perform() {
		// for safety!
		robot.stopMoving();
		State state = State.MOVING;
		int lastMessage = -1; // 0 for left, 1 for right, 2 for forward
		while (true) {
			switch (state) {
			case MOVING:
//				if (robot.vision.canSeeBlock()) {
				if (false) {
					robot.log.info("Robot can see block; stopping and going to StateMovingToBlock");
					robot.stopMoving();
					// TODO clearly long; we should get the block location from the vision system
					robot.setStateObject(new StateMovingToBlock(robot));
					return;
				} else if (robot.isLeftBump() || robot.isRightBump()) { //
					if (lastMessage != 0) {
						double rv = Math.random() > .5 ? .5 : -.5; // pick some random rotational velocity
						robot.sendMotorMessage(-.5, rv);
						lastMessage = 0;
					}
				    // block and move back
                    // block and rotate right
                    // done
//				} else if (robot.isRightBump()) { //
//					if (lastMessage != 1) {
//						robot.sendMotorMessage(-.5, .5);
//						lastMessage = 1;
//					}
				} else {
					robot.log.info("just go forward");
					if (lastMessage != 2) {
						robot.sendMotorMessage(.5, 0);
						lastMessage = 2;
					}
                }
				break;
			}
		}
	}
}
