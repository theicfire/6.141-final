package Planner;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import Robot.StateInitial;
import Robot.Robot;

public class Main implements NodeMain {

	// ENTRY POINT IS HERE
	@Override
	public void onStart(Node node) {		
		Robot robot = new Robot(node);
		StateInitial startState = new StateInitial(robot);
		robot.setStateObject(startState);

		robot.log.info("Start performing");
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
