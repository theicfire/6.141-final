package Sound;

import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.apache.commons.logging.Log;
import org.ros.message.rss_msgs.ArmMsg;

import Controller.Utility;

public class RosSpeak {
	private final Publisher<org.ros.message.std_msgs.String> soundPub;
	private static Log log;
	public RosSpeak(Node node) {
		log = node.getLog();
		soundPub = node.newPublisher("command/sound", "std_msgs/String");
				
		String[] args = {"hello"};
		log.info("Waiting for subscribers FOR SOUND");
			while (soundPub.getNumberOfSubscribers() == 0) {
//			 log.info("num sub FOR SOUND is 0");
		}
		log.info("More than 0 subscribers");
	}

	public void speak(String text) {
		org.ros.message.std_msgs.String msg = new org.ros.message.std_msgs.String();
		msg.data = text;
		soundPub.publish(msg);
	}
}
