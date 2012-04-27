package Grasping;

public interface ArmGymnastics {
	// non blocking
	public void lowerArm();
	public void raiseArm();
	public void setGripAngle(double radians);
	public void closeGripper();
	public void closeOnBlock();
	public void openGripper();
	
	// blocking command
	public boolean step();
}
