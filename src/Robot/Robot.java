package Robot;

import java.awt.geom.Point2D;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.BreakBeamMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.message.rss_msgs.VisionMsg;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import Grasping.Arm;
import Navigation.NavigationMain;
import WaypointDriver.RosWaypointDriver;

public class Robot {
	Arm arm;
	// Wheels wheels;
	// BreakBeamSensor handBreakBeam;
	// SonarSensor sideSonars[4];

	private RobotState robotState;
	NavigationMain navigationMain;
	private Subscriber<VisionMsg> visionSub;
	private Subscriber<BreakBeamMsg> doneMovingSub;
	
	private boolean visionCanSeeBlock;
	private boolean doneMoving;
	private Publisher<OdometryMsg> waypointCommandPub;
	
	public Robot(NavigationMain navigationMain, Publisher<OdometryMsg> waypointNav, Subscriber<VisionMsg> visionSub,
			Subscriber<BreakBeamMsg> doneMovingSub) {
		this.navigationMain = navigationMain;
		this.visionSub = visionSub;
		this.doneMovingSub = doneMovingSub;
		this.visionSub.addMessageListener(new VisionMessageListener());
		this.doneMovingSub.addMessageListener(new DoneMovingListener());
		this.waypointCommandPub = waypointNav;
	}
	
	public class VisionMessageListener implements
	MessageListener<org.ros.message.rss_msgs.VisionMsg>  {
		public void onNewMessage(org.ros.message.rss_msgs.VisionMsg vm) {
			visionCanSeeBlock = vm.detectedBlock;
		}
	}
	
	public class DoneMovingListener implements
			MessageListener<org.ros.message.rss_msgs.BreakBeamMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.BreakBeamMsg bb) {
			doneMoving = bb.beamBroken; // probably always true
		}
	}
	
	public void setStateObject(RobotState newRobotState) {
		robotState = newRobotState;
	}

	public RobotState getRobotState() {
		return robotState;
	}
	
	public boolean doneMoving() {
		return doneMoving;
	}
	
	public boolean canSeeBlock() {
		return visionCanSeeBlock;
	}
	
	public void goToLocation(Point2D.Double loc) {
		OdometryMsg dest = new OdometryMsg();
		dest.x = loc.x;
		dest.y = loc.y;
		waypointCommandPub.publish(dest);
		doneMoving = false;
	}
}
