package Robot;

import java.awt.geom.Point2D;

import Controller.Utility;

public class StateEnd extends RobotState {

	public StateEnd(Robot ri) {
		super(ri);
	}

	@Override
	public void perform() {
		robot.speaker.speak("well");
		Utility.sleepFor5Seconds();
		robot.speaker.speak("friday");
		// robot.arm.raiseArm();
		// robot.armDriver.doMovement(robot.arm);
		// this.robot.setStateObject(new StateLookingForBlocks(this.robot));
		while (true) {
			// victory dance
		}
	}
}