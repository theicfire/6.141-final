package VisualServoSolution;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.VisionMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class VisionMain implements NodeMain {
	Node globalNode;
	Publisher<org.ros.message.sensor_msgs.Image> vidPub;
	Subscriber<org.ros.message.sensor_msgs.Image> rawVidSub;
	Log log;

	boolean useNew = true;
	BlobTracking2 bt2;
	BlobTracking bt;
	private Publisher<VisionMsg> blobDetectedPub;
	public double centroidX;
	public double centroidY;
	public double range;
	public boolean targetDetected;
	private static final int IMAGE_WIDTH = 160;
	private static final int IMAGE_HEIGHT = 120;

//	private double target_hue_level = 0.66; // (Solution)
//	private double hue_threshold = 0.03; // (Solution)
//	private double saturation_level = 0.8; // (Solution)
	

	private double target_hue_level = 0.66; // (Solution)
	private double hue_threshold = 0.02; // (Solution)
	private double saturation_level = 0.8; // (Solution)
	// // Units are fraction of total number of pixels detected in blob //
	// (Solution)
	private double blob_size_threshold = 0.005; // (Solution)
	private double target_radius = 0.1; // (Solution)
	private double desired_fixation_distance = .5; // (Solution)
	private double translation_error_tolerance = .05;// (Solution)
	private double translation_velocity_gain = .75;// (Solution)
	private double translation_velocity_max = .75;// (Solution)
	private double rotation_error_tolerance = 0.2; // (Solution)
	private double rotation_velocity_gain = 0.15; // (Solution)
	private double rotation_velocity_max = 0.15; // (Solution)
	private boolean use_gaussian_blur = true;// (Solution)
	public double angle;


	@Override
	public void onStart(Node node) {
		// TODO Auto-generated method stub
		log = node.getLog();
		globalNode = node;


		log.info("added video sub");
		
		if (!useNew) {
		bt = new BlobTracking(IMAGE_WIDTH, IMAGE_HEIGHT, log);
		bt.targetHueLevel = target_hue_level;//(Solution)
		bt.hueThreshold = hue_threshold;//(Solution)
		bt.saturationLevel = saturation_level;//(Solution)
		bt.blobSizeThreshold = blob_size_threshold;//(Solution)
		bt.targetRadius = target_radius;//(Solution)
		bt.desiredFixationDistance = desired_fixation_distance;//(Solution)
		bt.translationErrorTolerance = translation_error_tolerance;//(Solution)
		bt.translationVelocityGain = translation_velocity_gain;//(Solution)
		bt.translationVelocityMax = translation_velocity_max;//(Solution)
		bt.rotationErrorTolerance = rotation_error_tolerance;//(Solution)
		bt.rotationVelocityGain = rotation_velocity_gain;//(Solution)
		bt.rotationVelocityMax = rotation_velocity_max;//(Solution)
		bt.useGaussianBlur = use_gaussian_blur;//(Solution)
		} else {
			bt2 = new BlobTracking2(IMAGE_WIDTH, IMAGE_HEIGHT, log);
			bt2.targetHueLevel = target_hue_level;//(Solution)
			bt2.hueThreshold = hue_threshold;//(Solution)
			bt2.saturationLevel = saturation_level;//(Solution)
			bt2.blobSizeThreshold = blob_size_threshold;//(Solution)
			bt2.targetRadius = target_radius;//(Solution)
			bt2.desiredFixationDistance = desired_fixation_distance;//(Solution)
			bt2.translationErrorTolerance = translation_error_tolerance;//(Solution)
			bt2.translationVelocityGain = translation_velocity_gain;//(Solution)
			bt2.translationVelocityMax = translation_velocity_max;//(Solution)
			bt2.rotationErrorTolerance = rotation_error_tolerance;//(Solution)
			bt2.rotationVelocityGain = rotation_velocity_gain;//(Solution)
			bt2.rotationVelocityMax = rotation_velocity_max;//(Solution)
			bt2.useGaussianBlur = use_gaussian_blur;//(Solution)
			
		}
		
		vidPub = node.newPublisher("/rss/blobVideo", "sensor_msgs/Image");
		Subscriber<org.ros.message.sensor_msgs.Image> rawVidSub = node.newSubscriber("rss/video", "sensor_msgs/Image");
		rawVidSub.addMessageListener(new InterpRawVid());
		rawVidSub = node.newSubscriber("rss/video", "sensor_msgs/Image");
		blobDetectedPub = node.newPublisher("/rss/VisionMain", "rss_msgs/VisionMsg");

		while (true) {
			VisionMsg vm = new VisionMsg();
			vm.detectedBlock = targetDetected;
			vm.distance = range;
			// important!!
			vm.theta = -angle;
			this.blobDetectedPub.publish(vm);
			
			try {
				Thread.sleep(30);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

	public class InterpRawVid implements
			MessageListener<org.ros.message.sensor_msgs.Image> {

		@Override
		public void onNewMessage(org.ros.message.sensor_msgs.Image src) {
			// TODO Auto-generated method stub

			Image dest = new Image(src.data, (int) src.width, (int) src.height);
//			log.info("hue" + dest.getPixel(50, 50).getHue());
//			log.info("sat" + dest.getPixel(50, 50).getSaturation());
//			log.info("bri" + dest.getPixel(50, 50).getBrightness());
			Image otherSrc = new Image(src.data, (int) src.width,
					(int) src.height);
			//log.info("bt + " + bt);
		
			if (!useNew) {
			bt.apply(otherSrc, dest);

//			for (int i = 50; i < 55; i++) {
//				for (int j = 50; j < 55; j++) {
//					dest.setPixel(i, j, new Image.Pixel(255, 0, 0));
//				}
//			}

			targetDetected = bt.targetDetected;
			centroidX = bt.centroidX;
			centroidY = bt.centroidY;
			range = bt.targetRange;
			} else {
				bt2.apply(otherSrc, dest);
				targetDetected = bt2.targetDetected;
				centroidX = bt2.centroidX;
				centroidY = bt2.centroidY;
				range = bt2.targetRange;
			}
			angle = (0.0171693 + 0.00987868 * centroidX) - Math.PI / 4;

//			log.info("range, angle" + range + ", " + angle);

			org.ros.message.sensor_msgs.Image pubImage = new org.ros.message.sensor_msgs.Image();
			pubImage.width = IMAGE_WIDTH;
			pubImage.height = IMAGE_HEIGHT;
			pubImage.encoding = "rgb8";
			pubImage.is_bigendian = 0;
			pubImage.step = IMAGE_WIDTH * 3;

			pubImage.data = dest.toArray();
			vidPub.publish(pubImage);
		}
	}
}
