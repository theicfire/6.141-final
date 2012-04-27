package VisualServoSolution;

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

}
