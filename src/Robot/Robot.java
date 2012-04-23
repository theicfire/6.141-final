package Robot;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.VisionMsg;
import org.ros.node.topic.Subscriber;

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
	Subscriber<VisionMsg> visionSub;
	
	boolean visionCanSeeBlock;
	
	public Robot(NavigationMain navigationMain, RosWaypointDriver waypointNav, Subscriber<VisionMsg> visionSub) {
		this.navigationMain = navigationMain;
		this.waypointDriver = waypointNav;
		this.visionSub = visionSub;
		this.visionSub.addMessageListener(new VisionMessageListener());
	}
	
	public class VisionMessageListener implements
	MessageListener<org.ros.message.rss_msgs.VisionMsg>  {
		public void onNewMessage(org.ros.message.rss_msgs.VisionMsg vm) {
			visionCanSeeBlock = vm.detectedBlock;
		}
	}
	
	public void setStateObject(RobotState newRobotState) {
		robotState = newRobotState;
	}

	public RobotState getRobotState() {
		return robotState;
	}
}
