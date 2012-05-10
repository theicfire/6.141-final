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
	private static final double STANDOFF_DISTANCE = 1.0; // should 0.75
	private static final double STANDOFF_EPSILON = .02; // should 0.75

	public StateMovingToBlock(Robot ri) {
		super(ri);
	}

	enum State {
		INIT
	}

	@Override
	public void perform() {
		robot.log.info("IN STATE MOVING TO BLOCK");
		robot.speaker.speak("i am getting that block");
		//robot.stopMoving(); // safety
		
		int noBlockTicks = 0;
		State state = State.INIT;			
		boolean lowered = false;
		Point2D.Double startPoint = robot.odom.getPosition();
		
		while (true) {
			switch (state) {
			case INIT:
				if (! robot.vision.canSeeBlock()) {
					robot.log.info("no block...:P");
					noBlockTicks += 1;
					// tuned to 20ms; do not change
					Utility.sleepFor20ms();
					// sweep left for 100 ticks (2 sec), then right for 200 ticks (4 sec), then 
					// give up
					if (noBlockTicks > 300) {
						// give up after 6 seconds
						robot.setStateObject(new StateLookingForBlocks(robot));
						return;
					} else if (noBlockTicks > 100) {
						robot.sendMotorMessage(0, 0.5);
					} else if (noBlockTicks > 0) {
						robot.sendMotorMessage(0, -0.5);
					}
					break;
				} else {
					noBlockTicks = 0;
				}
				
				double dist = robot.vision.getBlockDistance();
				
				if (lowered && robot.isBeamBroken()) {
					robot.log.info("In StateMovingBlock; going to StatePickingUpBlock");
					robot.setStateObject(new StatePickingUpBlock(robot, startPoint));
					return;
				} else {
					if (Math.abs(robot.vision.getBlockTheta()) > STANDOFF_ANGLE) {
						// rotate to that angle
						robot.sendMotorMessage(0, robot.vision.getBlockTheta() * 0.3);
						Utility.sleepFor20ms();
					} else if (!lowered && dist < STANDOFF_DISTANCE){
						robot.driveBackward(0);
					} else { // if the arm is lowered, we can assume the robot just goes forward
						if (!lowered) {
							robot.stopMoving(); // safety
							robot.arm.lowerArm();
							robot.arm.openGripper();
							robot.armDriver.doMovement(robot.arm);
							lowered = true;
						}
						robot.driveForward(0);
					}
					robot.log.info("distance to block is " + dist + " angle is " + Math.abs(robot.vision.getBlockTheta()));
//					robot.log.info("In StateMovingBlock; distance " + dist + " is not close enough yet");
					// like clicking a lot in starcraft (high APM w00t)
					Utility.sleepFor20ms();
				}
				break;
			}
		}
	}
}