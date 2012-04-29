package VisualServoSolution;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

import org.ros.message.rss_msgs.VisionMsg;

public class VisionMsgWrapper extends VisionMsg {
	private final VisionMsg visionMsg;

	public VisionMsgWrapper(VisionMsg vm) {
		visionMsg = vm;
	}

	public VisionMsgWrapper() {
		visionMsg = null;
	}

	public boolean canSeeBlock() {
		if (visionMsg == null)
			return false;
		return visionMsg.detectedBlock;
	}

	public double getBlockTheta() {
		if (visionMsg != null && canSeeBlock()) {
			return visionMsg.theta;
		}
		return -1;
	}

	public double getBlockDistance() {
		if (visionMsg != null && canSeeBlock()) {
			return visionMsg.distance;
		}
		return -1;
	}

	// DANGER DANGER DANGER VERY BAD DOES NOT WORK
	// getBlockDistance() and getBlockTheta() are NOT 
	// calibrated in any sense...
//	public Double getAbsBlockLocation(Point2D.Double ourLocation) {
//		Double us = ourLocation;
//		// TODO currently the angles are inverted
//		return new Double(
//				us.x + (getBlockDistance()) * Math.cos(-getBlockTheta()), 
//				us.y + (getBlockDistance()) * Math.sin(-getBlockTheta()));
//	}

}
