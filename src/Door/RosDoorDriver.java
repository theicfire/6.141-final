package Door;

import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.apache.commons.logging.Log;
import org.ros.message.rss_msgs.ArmMsg;

import Controller.Utility;

public class RosDoorDriver {
	private final Publisher<ArmMsg> doorPub;
	private long CLOSE_PWM = 828;
	private long OPEN_PWM = 1300;
	private static Log log;
	public RosDoorDriver(Node node) {
		log = node.getLog();
		doorPub = node.newPublisher("command/Door", "rss_msgs/ArmMsg");
				
		log.info("Waiting for subscribers FOR DOOR");
			while (doorPub.getNumberOfSubscribers() == 0) {
			 //log.info("num sub FOR DOOR is 0");
		}
		log.info("More than 0 subscribers");

		sendDoorPWM(828);
		openDoor();
		log.info("Door is ready");
	}

	public void sendDoorPWM(long pwm) {
		ArmMsg msg = new ArmMsg();
		long[] pwms = {pwm, pwm, pwm};
		msg.pwms = pwms;
//		log.info("arm publishing " + Arrays.toString(msg.pwms));
		doorPub.publish(msg);
	}
	
	public void openDoor() {
		sendDoorPWM(OPEN_PWM);
	}
	
	public void closeDoor() {
		sendDoorPWM(CLOSE_PWM);
	}
}
