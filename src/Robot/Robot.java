package Robot;

import Grasping.Arm;
import Navigation.NavigationMain;
import VisualServoSolution.VisualServo;
import WaypointDriver.RosWaypointDriver;

public class Robot {
	Arm arm;
	// Wheels wheels;
	// BreakBeamSensor handBreakBeam;
	// SonarSensor sideSonars[4];

	RobotState robotState;
	NavigationMain navigationMain;
	RosWaypointDriver waypointDriver;
	VisualServo vision;
	
	public Robot(NavigationMain navigationMain, RosWaypointDriver waypointNav, VisualServo vision) {
		this.navigationMain = navigationMain;
		this.waypointDriver = waypointNav;
		this.vision = vision;
	}
	
	public void setStateObject(RobotState newRobotState) {
		robotState = newRobotState;
	}

	public RobotState getRobotState() {
		return robotState;
	}
}
