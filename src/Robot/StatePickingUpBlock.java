package Robot;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Point2D.Double;
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

import Controller.AngleController;
import Grasping.Arm;

public class StatePickingUpBlock extends RobotState {

	public StatePickingUpBlock(Robot ri) {
		super(ri);
	}

	enum State {
		ROTATING,
		APPROACHING_BLOCK,
		MOVING_ARM,
		STORING_BLOCK,
		CHECK_HOLDING_PEN
	}
	@Override
	public void perform() {	
		
//		AngleController pc = new AngleController(1, 0, robot.);
//	       pc.setDesired(0);
	       
	       State state = State.ROTATING;
	       boolean done = false;
           
	       robot.arm.prepareToPickup(); // blocking
	       
           while (!done) {
	           switch (state) {
	           case APPROACHING_BLOCK: {
	                   robot.stopMoving();
	                   state = State.MOVING_ARM;
	               }	              
	               break;
	           case MOVING_ARM:
	               if (robot.vision.canSeeBlock() == false) {
	                   robot.arm = new Arm(); // reset
	                   // blocking arm controller movement
	                   robot.armDriver.doMovement(robot.arm);
//	                   robot.failedToPickBlock(b);
//	                   exitState =
//	                       iDontKnowWhatShouldBeTheStateIfWeFail;
	                   done = true;
	               } else if (robot.vision.canSeeBlock() == true) {
	            	   robot.arm.step();
	            	   // blocking arm controller movement
	            	   robot.armDriver.doMovement(robot.arm);
	               }
	               break;

	           case STORING_BLOCK:
	               
	               // do arm movements here
	               
	               break;
	               
	           case CHECK_HOLDING_PEN:
	               // wait a little bit for the block to be acknowledged
	               // this state shouldn't really be necessary though...
	               
	               break;
	               
	           
	           }

	       }
	       
	   }
		// state transition
		this.robot.setStateObject(new StateLookingForBlocks(this.robot));

	}

}