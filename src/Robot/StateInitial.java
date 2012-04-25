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
		// TODO Auto-generated constructor stub
	}

	@Override
	public void perform() {	
		this.robot.setStateObject(new StateLookingForBlocks(this.robot));
	}

}