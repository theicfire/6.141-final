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
//		this.robot.setStateObject(new StateLookingForBlocks(this.robot));
		
//		AngleController pc = new AngleController(1, 0, robot.odom);
//	       pc.setDesired(0);
	       
//	       State state = State.ROTATING;
//	       boolean done = false;
//
//	       while (!done) {
//	           switch (state) {
//	           case ROTATING:
////	               if (cantSeeBlock || cantReachBlock ) {
//	        	   if (false) {
////	                   robot.failedToPickBlock(b);
////	                   exitState =
////	                       iDontKnowWhatShouldBeTheStateIfWeFail;
////	                   done = true;
////	               } else if (alreadyFacingBlock) {
////	                   state = State.APPROACHING_BLOCK;
//	               } else {
//	            	   Point2D.Double p = new Point2D.Double(0, 0); // TODO
//	            	   robot.rotateToLocation(p);
//	               }
//	               break;
//	           case APPROACHING_BLOCK:
//
//	               if (cantSeeBlock || cantReachBlock ) {
//	                   robot.failedToPickBlock(b);
//	                   exitState =
//	                       iDontKnowWhatShouldBeTheStateIfWeFail;
//	                   done = true;
//	               } else if (noLongerFacingBlock) {
//	                   stopMoving;
//	                   state = State.ROTATING;
//	               } else if (atRightDistanceToBlock) {
//	                   stopMoving();
//	                   robot.arm.prepareToPickup(); // blocking
//	                   state = State.MOVING_ARM;
//	               }
//	               
//	               // move towards the pose
//	               // from robot will perform the grab
//	               
//	               
//	               break;
//	           case MOVING_ARM:
//	               if (cantSeeBlock || cantReachBlock) {
//	                   robot.resetArm();
//	                   robot.failedToPickBlock(b);
//	                   exitState =
//	                       iDontKnowWhatShouldBeTheStateIfWeFail;
//	                   done = true;
//	               } else if (armNotDoneMoving) {
//	               // we perhaps need to use a controller here
//	                   angles = armController.Step();
//	                   arm.SetAngles(angles);
//	               }
//	               break;
//
//	           case STORING_BLOCK:
//	               
//	               // do arm movements here
//	               
//	               break;
//	               
//	           case CHECK_HOLDING_PEN:
//	               // wait a little bit for the block to be acknowledged
//	               // this state shouldn't really be necessary though...
//	               
//	               break;
//	               
//	           
//	           }
//
//	       }
	       
//	   }
	}

}