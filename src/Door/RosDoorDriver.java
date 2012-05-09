package Door;

import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.apache.commons.logging.Log;
import org.ros.message.rss_msgs.ArmMsg;

public class RosDoorDriver {
	private final Publisher<ArmMsg> doorPub;
	private static Log log;
	public RosDoorDriver(Node node) {
		log = node.getLog();
		doorPub = node.newPublisher("command/Door", "rss_msgs/ArmMsg");
		
		log.info("Waiting for subscribers");
		while (doorPub.getNumberOfSubscribers() == 0) {
			// log.info("num sub is 0");
		}
		log.info("More than 0 subscribers");
		
		log.info("Door is ready");
	}

	public void sendDoorPWM() {
		ArmMsg msg = new ArmMsg();
		long[] pwms = {500, 500, 500};
		msg.pwms = pwms;
//		log.info("arm publishing " + Arrays.toString(msg.pwms));
		doorPub.publish(msg);
	}
}
