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

public class StateInitial extends RobotState {

	public StateInitial(Robot ri) {
		super(ri);
	}

	@Override
	public void perform() {
		// robot.arm.raiseArm();
		// robot.armDriver.doMovement(robot.arm);
		// this.robot.setStateObject(new StateLookingForBlocks(this.robot));

		if (robot.planner.blocksStored >= 7) {
			robot.setStateObject(new StateMakingStructure(robot));
		} else {
			robot.setStateObject(new StateLookingForBlocks(robot));
		}
	}
}