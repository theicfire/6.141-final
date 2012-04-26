package Robot;

import java.awt.geom.Point2D;

import Robot.StateLookingForBlocks.State;

/**
 * Move towards any point (hopefully a point where a block exists) 
 * 	and stop when you are within x meters of it (doesn't have to be perfect). 
 * @author rss-student
 *
 */
public class StateMovingToBlock extends RobotState {

	private Point2D.Double blockLoc;
	public StateMovingToBlock(Robot ri, Point2D.Double blockLoc) {
		super(ri);
		this.blockLoc = blockLoc;
		// TODO Auto-generated constructor stub
	}

	enum State {
		INIT
	}
	
	@Override
	public void perform() {
		// for safety!
		robot.stopMoving();

		State state = State.INIT;

		while (true) {
			switch (state) {
			case INIT:
				boolean robotNear = false; // TODO
				if (robotNear) {
					robot.setStateObject(new StatePickingUpBlock(robot));
					return;
				}

				robot.goToLocation(blockLoc);
				break;
			}
		}
	}

}