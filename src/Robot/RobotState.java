package Robot;

public abstract class RobotState {
   Robot robot;
   public abstract void perform(); // may change the state object of the robot
   
   protected RobotState(Robot ri) {
       this.robot = ri;
   }
}