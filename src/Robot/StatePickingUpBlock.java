package Robot;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Point2D.Double;
import java.io.File;
import java.io.IOException;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.ros.message.lab5_msgs.GUIEraseMsg;
import org.ros.message.lab5_msgs.GUISegmentMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.node.parameter.ParameterTree;

import Controller.AngleController;
import Controller.Utility;
import Controller.Utility.Pose;
import Grasping.Arm;
import Planner.Planner;

public class StatePickingUpBlock extends RobotState {

	private Point2D.Double startPoint;
	public StatePickingUpBlock(Robot ri, Point2D.Double startPoint) {
		super(ri);
		this.startPoint = startPoint;
	}

	enum State {
		APPROACHING_BLOCK, MOVING_ARM, STORING_BLOCK, CHECK_HOLDING_PEN
	}

	@Override
	public void perform() {
		robot.stopMoving();
		
		State state = State.APPROACHING_BLOCK;
		boolean done = false;

		// robot.arm.prepareToPickup();
		
		// update position of the odometry
		// something like
		// TODO
		final double DIST_FROM_BLOCK_TO_ROBOT = .25; // totally arbitrary
		Point2D.Double curPoint = robot.odom.getPosition();
		double curTheta = robot.odom.getTheta();
		Point2D.Double blockPose = robot.planner.getCurrentBlockPosition();
		// end - start
		double odomAngle = Math.atan2(curPoint.y - startPoint.y, curPoint.x - startPoint.x);
		double realAngle = Math.atan2(blockPose.y - startPoint.y, blockPose.x - startPoint.x);
		double angleDiff = realAngle - odomAngle;
		double newTheta = curTheta + angleDiff;
		
		blockPose.x -= DIST_FROM_BLOCK_TO_ROBOT * Math.cos(newTheta );
		blockPose.y -= DIST_FROM_BLOCK_TO_ROBOT * Math.sin(newTheta );
		robot.odom.updatePosition((new Utility()).new Pose(blockPose,
				newTheta));
		robot.planner.markCurrentBlockDone();

		while (!done) {
			switch (state) {
			case APPROACHING_BLOCK: {
				robot.stopMoving();
				state = State.MOVING_ARM;
				break;
			}
			case MOVING_ARM: {
				if (! robot.vision.canSeeBlock()) {
					// TODO TODO TODO
//					robot.arm = new Arm(); // reset
//					robot.arm.closeGripper();
//					robot.armDriver.doMovement(robot.arm);
//					// TODO verify breakbeam is in place
//					robot.arm.raiseArm();
//
//					// blocking arm controller movement
//					robot.armDriver.doMovement(robot.arm);
//					robot.log.info("don't see block, putting arm up");
					// robot.failedToPickBlock(b);
					// exitState =
					// iDontKnowWhatShouldBeTheStateIfWeFail;
					done = true;
				} else if (robot.vision.canSeeBlock()) {
					robot.log.info("see block - lowering arm");
//					Utility.sleepFor5Seconds();
					robot.arm.openGripper();
					robot.armDriver.doMovement(robot.arm);
					robot.arm.lowerArm();
					// blocking arm controller movement, TODO replace with
					// non-blocking
					robot.armDriver.doMovement(robot.arm);
					state = State.STORING_BLOCK;
					robot.log.info("arm lowered");
				}
				break;
			}
			case STORING_BLOCK: {
				robot.log.info("storing block");
	
				robot.arm.closeGripper();
				robot.armDriver.doMovement(robot.arm);
				robot.arm.raiseArm();
				robot.armDriver.doMovement(robot.arm);
				robot.arm.openGripper();
				robot.armDriver.doMovement(robot.arm);

				robot.log.info("done storing block");
				done = true;
				robot.planner.nextClosestBlock();
				break;
			}
			case CHECK_HOLDING_PEN: {
				// wait a little bit for the block to be acknowledged
				// this state shouldn't really be necessary though...

				break;
			}
			} // end switch
		} // end while

		// state transition
		this.robot.setStateObject(new StateInitial(this.robot));

	}

}