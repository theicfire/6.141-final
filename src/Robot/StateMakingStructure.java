package Robot;

import Controller.Utility;

public class StateMakingStructure extends RobotState {
	
	public StateMakingStructure(Robot ri) {
		super(ri);
	}

	enum State {
		INIT
	}

	@Override
	public void perform() {
		State state = State.INIT;
		robot.speaker.speak("initializing structure");
		boolean done = false;
		while (!done) {
			switch (state) {
				case INIT: {
					robot.log.info("entry ENTRY ENTRY making struct");
////					this.robot.stopMoving();
////					this.robot.driveForward(0);					
////					Utility.sleepFor250ms();
//					Utility.sleepFor250ms();
//					this.robot.stopMoving();
					Utility.sleepFor5Seconds();
					this.robot.doorDriver.openDoor();
					Utility.sleepFor5Seconds();
					this.robot.driveForward(0);
					Utility.sleepFor5Seconds();
					this.robot.stopMoving();
					// open the door
					done = true;
					robot.setStateObject(new StateEnd(robot));
					break;
				}
			} // end switch
		} // end while
	}
}