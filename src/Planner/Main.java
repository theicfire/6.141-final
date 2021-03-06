package Planner;

import java.awt.geom.Point2D;
import java.io.IOException;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;

import Controller.Utility;
import Controller.Utility.Pose;
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
		
		boolean actuallyWork = true;
		if (! actuallyWork) {
			robot.log.info("skip perform method flag set - JUST KIDDING");
		}		
		while (actuallyWork) {
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
