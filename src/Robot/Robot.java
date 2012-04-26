package Robot;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.BreakBeamMsg;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.message.rss_msgs.VisionMsg;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.Node;

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
	
	private VisionMsg visionMsg;
	private boolean doneMoving;
	private Publisher<OdometryMsg> waypointCommandPub;
	private Publisher<OdometryMsg> angleCommandPub;
	private Publisher<MotionMsg> movePub;
	private Publisher<BreakBeamMsg> stopPub;
	public Log log;
	
	public Robot(Node node) {
		this.log = node.getLog();
		this.navigationMain = new NavigationMain(node);
		this.visionSub = node.newSubscriber("rss/VisionMain", "rss_msgs/VisionMsg");
		this.doneMovingSub = node.newSubscriber("rss/waypointcomplete", "rss_msgs/BreakBeamMsg");
		this.waypointCommandPub = node.newPublisher("rss/waypointcommand", "rss_msgs/OdometryMsg");
		this.angleCommandPub = node.newPublisher("rss/anglecommand", "rss_msgs/OdometryMsg");
		this.stopPub = node.newPublisher("rss/stopcommand", "rss_msgs/BreakBeamMsg");
		this.movePub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		
		log.info("Waiting for movePub");
		while (movePub.getNumberOfSubscribers() == 0) {
			// block
		}
		log.info("Done waiting for movePub");
		this.visionSub.addMessageListener(new VisionMessageListener());
		this.doneMovingSub.addMessageListener(new DoneMovingListener());
	}
	
	public class VisionMessageListener implements
	MessageListener<org.ros.message.rss_msgs.VisionMsg>  {
		public void onNewMessage(org.ros.message.rss_msgs.VisionMsg vm) {
			visionMsg = vm;
		}
	}
	
	public class DoneMovingListener implements
			MessageListener<org.ros.message.rss_msgs.BreakBeamMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.BreakBeamMsg bb) {
			log.info("Done moving message received");
			doneMoving = bb.beamBroken; // probably always true
		}
	}
	
	public void setStateObject(RobotState newRobotState) {
		log.info("Setting new state" + newRobotState.getClass());
		robotState = newRobotState;
	}

	public RobotState getRobotState() {
		return robotState;
	}
	
	public boolean doneMoving() {
		return doneMoving;
	}
	
	public void stopMoving() {
		BreakBeamMsg stop = new BreakBeamMsg();
		stop.beamBroken = true;
		stopPub.publish(stop);
	}
	
	public boolean canSeeBlock() {
		return visionMsg.detectedBlock;
	}
	
	public double getBlockTheta() {
		if (canSeeBlock()) {
			return visionMsg.theta;
		}
		return -1;
	}
	
	public double getBlockDistance() {
		if (canSeeBlock()) {
			return visionMsg.distance;
		}
		return -1;
	}
	
	public void goToLocation(Point2D.Double loc) {
		log.info("go to location" + loc);
		OdometryMsg dest = new OdometryMsg();
		dest.x = loc.x;
		dest.y = loc.y;
		waypointCommandPub.publish(dest);
		doneMoving = false;
	}
	
	public void rotateToLocation(Point2D.Double loc) {
		log.info("go to location" + loc);
		OdometryMsg dest = new OdometryMsg();
		dest.x = loc.x;
		dest.y = loc.y;
		angleCommandPub.publish(dest);
		doneMoving = false;
	}
	
	public void rotateToTheta(double theta) {
		// make an arbitrary point that is theta away
		Point2D.Double p = new Point2D.Double(Math.cos(theta), Math.sin(theta)); // TODO sign ok?
		rotateToLocation(p);
	}
	
	public void sendMotorMessage(double translationalVelocity, double rotationalVelocity) {
		log.info("sendMotorMessage" + translationalVelocity + " " + rotationalVelocity);
		MotionMsg stopMsg = new MotionMsg();
		stopMsg.rotationalVelocity = rotationalVelocity;
		stopMsg.translationalVelocity = translationalVelocity;
		movePub.publish(stopMsg);
	}
}
