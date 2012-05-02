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

import Controller.Utility;
import Grasping.Arm;
import Grasping.RosArmDriver;
import Localization.Localizer;
import Navigation.NavigationMain;
import Planner.Planner;
import VisualServoSolution.VisionMsgWrapper;

public class Robot {
	Arm arm;
	RosArmDriver armDriver;
	// Wheels wheels;
	// BreakBeamSensor handBreakBeam;
	// SonarSensor sideSonars[4];
	
	private RobotState robotState;
	Planner planner;
	NavigationMain navigationMain;
	Localizer odom;
	VisionMsgWrapper vision;
	private boolean doneMoving;

	private Subscriber<VisionMsg> visionSub;
	private Subscriber<BreakBeamMsg> doneMovingSub;
	private Subscriber<BreakBeamMsg> breakBeamSub;
	private boolean beamBroken;

	private Publisher<OdometryMsg> waypointCommandPub;
	private Publisher<OdometryMsg> angleCommandPub;
	private Publisher<MotionMsg> movePub;
	private Publisher<BreakBeamMsg> stopPub;
	public Log log;

	public Robot(Node node) {
		this.log = node.getLog();
		this.navigationMain = new NavigationMain(node);
		this.arm = new Arm();
		this.armDriver = new RosArmDriver(node);
		
		this.visionSub = node.newSubscriber("rss/VisionMain", "rss_msgs/VisionMsg");

		this.doneMovingSub = node.newSubscriber("rss/waypointcomplete", "rss_msgs/BreakBeamMsg");
		this.breakBeamSub = node.newSubscriber("rss/BreakBeam", "rss_msgs/BreakBeamMsg");
		this.waypointCommandPub = node.newPublisher("rss/waypointcommand","rss_msgs/OdometryMsg");
		this.angleCommandPub = node.newPublisher("rss/anglecommand", "rss_msgs/OdometryMsg");

		this.stopPub = node.newPublisher("rss/stopcommand",
				"rss_msgs/BreakBeamMsg");
		this.movePub = node.newPublisher("rss/waypointMovecommand", "rss_msgs/MotionMsg");

		log.info("Waiting for movePub");
		
		log.info("Done waiting for movePub");
		log.info("Done waiting for waypointCommand");
		while (waypointCommandPub.getNumberOfSubscribers() == 0) {
			// block
		}
		log.info("Done waiting for waypointCommand");
		log.info("~~~~DONE ALL WAITING IN ROBOT~~~~");
		
		this.odom = new Localizer(node, true);
		this.planner = new Planner(odom, log);
		this.vision = new VisionMsgWrapper();
		this.visionSub.addMessageListener(new VisionMessageListener());
		this.doneMovingSub.addMessageListener(new DoneMovingListener());
		this.breakBeamSub.addMessageListener(new BreakBeamListener());
	}

	public class VisionMessageListener implements
			MessageListener<org.ros.message.rss_msgs.VisionMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.VisionMsg vm) {
			vision = new VisionMsgWrapper(vm);
		}
	}

	public class DoneMovingListener implements
			MessageListener<org.ros.message.rss_msgs.BreakBeamMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.BreakBeamMsg bb) {
			log.info("Done moving message received");
			doneMoving = bb.beamBroken; // probably always true
		}
	}
	
	public class BreakBeamListener implements
			MessageListener<org.ros.message.rss_msgs.BreakBeamMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.BreakBeamMsg bb) {
			beamBroken = bb.beamBroken; // probably always true
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

	public void driveToLocation(Point2D.Double loc) {
		log.info("go to location" + loc);
		OdometryMsg dest = new OdometryMsg();
		dest.x = loc.x;
		dest.y = loc.y;
		waypointCommandPub.publish(dest);
		doneMoving = false;
	}

	public void rotateToLocation(Point2D.Double loc) {
		log.info("setting angle based on point2d" + loc);
		OdometryMsg dest = new OdometryMsg();
		dest.x = loc.x;
		dest.y = loc.y;		
		angleCommandPub.publish(dest);
		doneMoving = false;
	}

	public void rotateToTheta(double theta) {
		log.info("rotating by " + theta);
		OdometryMsg dest = new OdometryMsg();
		dest.x = 0;
		dest.y = 0;
		dest.theta = theta;
		angleCommandPub.publish(dest);
		doneMoving = false;
	}
	
	public void sendMotorMessage(double translationalVelocity, double rotationalVelocity) {
		MotionMsg motionMsg = new MotionMsg();
		motionMsg.rotationalVelocity = rotationalVelocity;
		motionMsg.translationalVelocity = translationalVelocity;
		movePub.publish(motionMsg);
	}
	
	public Double getCurrentLocation() {
		return this.odom.getPosition();
	}

	// TODO move these somewhere else? I don't really like this...
	public void driveForward(double atAngle) {
		OdometryMsg om = new OdometryMsg();
		om.type = "forward";
		om.theta = atAngle;
		this.waypointCommandPub.publish(om);
	}
	
	public void driveBackward(double atAngle) {
		OdometryMsg om = new OdometryMsg();
		om.type = "backward";
		om.theta = atAngle;
		this.waypointCommandPub.publish(om);
	}
	
	public boolean isBeamBroken() {
		return beamBroken;
	}

	// NO MORE METHODS HERE!!!! NO MORE METHODS HERE!!!!
	// instead of doing robot.resetArm()
	// put the methods where they logically belong in the hierarchy
	// eg., robot.arm.reset()
	// instead of writing robot.getBlockDistance()
	// use robot.vision.getBlockDistance()
}
