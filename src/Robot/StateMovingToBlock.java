package Robot;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

import Controller.PositionController;
import Controller.ProportionalController;
import Controller.Utility;
import Robot.StateLookingForBlocks.State;

/**
 * Move towards any point (hopefully a point where a block exists) and stop when
 * you are within x meters of it (doesn't have to be perfect).
 * 
 * @author rss-student
 * 
 */
public class StateMovingToBlock extends RobotState {

	private static final double STANDOFF_ANGLE = 0.1;
	private static final double STANDOFF_DISTANCE = .75; // should 0.75
	private static final double STANDOFF_EPSILON = .02; // should 0.75

	public StateMovingToBlock(Robot ri) {
		super(ri);
	}

	enum State {
		INIT
	}

	@Override
	public void perform() {
		robot.stopMoving(); // safety
		State state = State.INIT;
		robot.arm.lowerArm();
		robot.arm.openGripper();
		robot.armDriver.doMovement(robot.arm);
		
		while (true) {
			switch (state) {
			case INIT:
				if (! robot.vision.canSeeBlock()) {
					robot.log.info("no block...:P");
					Utility.sleepFor20ms();
					break;
				}
				
				double dist = robot.vision.getBlockDistance();
				if (Math.abs(dist - STANDOFF_DISTANCE) < STANDOFF_EPSILON && 
						Math.abs(robot.vision.getBlockTheta()) < STANDOFF_ANGLE) {
					robot.log.info("In StateMovingBlock; going to statePickingUpblock");
					robot.setStateObject(new StatePickingUpBlock(robot));
//					robot.setStateObject(new StateMovingToBlock(robot));
					return;
				} else {
					if (Math.abs(robot.vision.getBlockTheta()) > STANDOFF_ANGLE) {
						// rotate to that angle
						robot.sendMotorMessage(0, robot.vision.getBlockTheta() * .3);
						Utility.sleepFor20ms();
					} else if (dist < STANDOFF_DISTANCE){
//						robot.driveBackward(robot.vision.getBlockTheta() / 3.0);
						robot.driveBackward(0);
					} else {
						robot.driveForward(0);
					}
					robot.log.info("distance to block is " + dist + " angle is " + Math.abs(robot.vision.getBlockTheta()));
										
//					robot.log.info("dist " + dist + " camvis: "
//							+ robot.vision.getBlockDistance() + " camang"
//							+ robot.vision.getBlockTheta());
//					robot.log.info("In StateMovingBlock; distance " + dist + " is not close enough yet");
					// like clicking a lot in starcraft (high APM w00t)
					Utility.sleepFor20ms();
				}
				break;
			}
		}
	}
}