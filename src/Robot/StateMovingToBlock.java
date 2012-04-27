package Robot;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

import Controller.PositionController;
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

	private final double STANDOFF_DISTANCE = 0.1;

	public StateMovingToBlock(Robot ri) {
		super(ri);
	}

	enum State {
		INIT
	}

	@Override
	public void perform() {
		robot.stopMoving(); //safety
		State state = State.INIT;

		while (true) {
			switch (state) {
			case INIT:	
				if (Utility.getMagnitude(robot.getCurrentLocation(), robot.getBlockLocation()) < STANDOFF_DISTANCE) {
					robot.setStateObject(new StatePickingUpBlock(robot));
					return;
				} else {
					robot.driveToLocation(robot.getBlockLocation());
					Utility.sleepForASecond();
					break;
				}
			}
		}
	}

}