package Robot;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.File;
import java.io.IOException;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.ros.message.lab5_msgs.GUIEraseMsg;
import org.ros.message.lab5_msgs.GUISegmentMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.node.parameter.ParameterTree;

import Controller.Utility;
import Navigation.PolygonObstacle;

public class StateInitial extends RobotState {

	public StateInitial(Robot ri) {
		super(ri);
	}

	@Override
	public void perform() {
		// robot.arm.raiseArm();
		// robot.armDriver.doMovement(robot.arm);
		// this.robot.setStateObject(new StateLookingForBlocks(this.robot));

		// if we are in c-space, get out
		PolygonObstacle[] cSpaceObstacles = this.robot.navigationMain.cspace.getObstacles();
		boolean didCSpaceEscape = false;
		while (isInCSpace(robot.odom.getPosition(), cSpaceObstacles) && robot.planner.getVisualServoStartPoint() != null) {
			didCSpaceEscape = true;
			robot.log.info("i am in cspace and driving back to start location: " + robot.planner.getVisualServoStartPoint());
			//robot.driveToLocation(robot.planner.getVisualServoStartPoint());
			robot.driveBackward(0);
			Utility.sleepFor250ms();
		} 
		if (didCSpaceEscape) {
			robot.stopMoving();
			Utility.sleepFor5Seconds();
			robot.planner.nextClosestBlock();
		}
		robot.planner.setVisualServoStartPoint(null);
		
		if (robot.planner.blocksStored >= 7) {
			robot.setStateObject(new StateMakingStructure(robot));
		} else {
			robot.setStateObject(new StateLookingForBlocks(robot));
		}
	}
	
	private boolean isInCSpace(Point2D.Double currentPosition, PolygonObstacle[] cSpaceObstacles) {
		for (PolygonObstacle o: cSpaceObstacles) {
//			robot.log.info("closed?: " + o.closed);
//			robot.log.info("curpos " + currentPosition);
			if (o.contains(currentPosition)) {
				robot.log.info("I am inside obstacle " + o);
				return true;
			}
		}
		return false;
	}
}