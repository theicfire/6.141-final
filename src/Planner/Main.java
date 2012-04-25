package Planner;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.message.rss_msgs.BreakBeamMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.message.rss_msgs.VisionMsg;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import Navigation.NavigationMain;
import Robot.StateInitial;
import Robot.Robot;
import VisualServoSolution.VisionMain;
import WaypointDriver.RosWaypointDriver;

public class Main implements NodeMain {

	// ENTRY POINT IS HERE
	@Override
	public void onStart(Node node) {
		Planner p = new Planner();

		// start thread for vision
		// start thread for sonars
		// start thread for odometer? maybe not
		// start thread for bump sensors
		Subscriber<VisionMsg> vs = node.newSubscriber("rss/VisionMain",
				"rss_msgs/VisionMsg");
		Subscriber<BreakBeamMsg> doneMove = node.newSubscriber("rss/waypointcomplete", "rss_msgs/BreakBeamMsg");
		Publisher<OdometryMsg> om = node.newPublisher("rss/waypointcommand", "rss_msgs/OdometryMsg");
		Robot robot = new Robot(new NavigationMain(node), om, vs, doneMove);
		StateInitial startState = new StateInitial(robot);
		robot.setStateObject(startState);

		while (true) {
			// make states represent atomic actions
			robot.getRobotState().perform();
		}
	}

	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

}
