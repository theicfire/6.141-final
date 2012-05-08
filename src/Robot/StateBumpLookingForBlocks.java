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
	public StateLookingForBlocks(Robot ri) {
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
		while (true) {
			switch (state) {
			case MOVING:
				if (robot.vision.canSeeBlock()) {
					robot.log.info("Robot can see block; stopping and going to StateMovingToBlock");
					robot.stopMoving();
					// TODO clearly long; we should get the block location from the vision system
					robot.setStateObject(new StateMovingToBlock(robot));
					return;
				} else if (doneMoving) { // 
                    if (leftBumpEngaged) {
                        // block and move back
                        // block and rotate right
                        // done
                    } else if (rightBumpEngaged) {
                        // block and move back
                        // block and rotate left
                        // done
                    } else {
                        // Move forward
                    }
				} else {
                    Thread.sleep(200);
                    // wait (block) until done moving or you see a block
                }
				break;
			}
		}
	}
}
