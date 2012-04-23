package VisualServoSolution;

import java.util.concurrent.ArrayBlockingQueue;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import VisualServoSolution.BlobTracking;
import VisualServoSolution.Image;
import VisualServoSolution.VisionGUI;
import VisualServoSolution.Image.Pixel;

/**
 * 
 * @author previous TA's, prentice, vona
 * 
 */
public class VisualServo implements NodeMain, Runnable {

	private static final int width = 160;

	private static final int height = 120;

	Publisher<MotionMsg> publisher; // (Solution)

	/**
	 * <p>
	 * The blob tracker.
	 * </p>
	 **/
	private BlobTracking blobTrack = null;

	private double target_hue_level = 0.475; // (Solution)
	private double hue_threshold = 0.08; // (Solution)
	private double saturation_level = 0.5; // (Solution)
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

	private VisionGUI gui;
	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(
			1);

	protected boolean firstUpdate = true;

	public Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;

	/**
	 * <p>
	 * Create a new VisualServo object.
	 * </p>
	 */
	public VisualServo() {

		setInitialParams();

		gui = new VisionGUI();
	}

	protected void setInitialParams() {

	}

	/**
	 * <p>
	 * Handle a CameraMessage. Perform blob tracking and servo robot towards
	 * target.
	 * </p>
	 * 
	 * @param a
	 *            received camera message
	 */
	public void handle(byte[] rawImage) {

		visionImage.offer(rawImage);
	}

	@Override
	public void run() {
		while (true) {

			Image src = null;
			try {
				src = new Image(visionImage.take(), width, height);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			Pixel p = src.getPixel(width / 2, height / 2);

			Image dest = new Image(src.toArray(), width, height);
			blobTrack.apply(src, dest);

			// update newly formed vision message
			gui.setVisionImage(dest.toArray(), width, height);

			// Begin Student Code

			// publish velocity messages to move the robot towards the target
			MotionMsg msg = new MotionMsg(); // (Solution)
			msg.translationalVelocity = blobTrack.translationVelocityCommand; // (Solution)
			msg.rotationalVelocity = blobTrack.rotationVelocityCommand; // (Solution)
			publisher.publish(msg); // (Solution)

			// End Student Code
		}
	}

	/**
	 * <p>
	 * Run the VisualServo process
	 * </p>
	 * 
	 * @param optional
	 *            command-line argument containing hostname
	 */
	@Override
	public void onStart(Node node) {
		blobTrack = new BlobTracking(width, height, node.getLog());

		// Begin Student Code

		// set parameters on blobTrack as you desire

		blobTrack.targetHueLevel = target_hue_level;// (Solution)
		blobTrack.hueThreshold = hue_threshold;// (Solution)
		blobTrack.saturationLevel = saturation_level;// (Solution)
		blobTrack.blobSizeThreshold = blob_size_threshold;// (Solution)
		blobTrack.targetRadius = target_radius;// (Solution)
		blobTrack.desiredFixationDistance = desired_fixation_distance;// (Solution)
		blobTrack.translationErrorTolerance = translation_error_tolerance;// (Solution)
		blobTrack.translationVelocityGain = translation_velocity_gain;// (Solution)
		blobTrack.translationVelocityMax = translation_velocity_max;// (Solution)
		blobTrack.rotationErrorTolerance = rotation_error_tolerance;// (Solution)
		blobTrack.rotationVelocityGain = rotation_velocity_gain;// (Solution)
		blobTrack.rotationVelocityMax = rotation_velocity_max;// (Solution)
		blobTrack.useGaussianBlur = use_gaussian_blur;// (Solution)

		System.err.println("  target hue level: " + blobTrack.targetHueLevel); // (Solution)
		System.err.println("  hue threshold: " + blobTrack.hueThreshold); // (Solution)
		System.err.println("  saturation level: " + blobTrack.saturationLevel); // (Solution)
		System.err.println("  blob size threshold: " + // (Solution)
				blobTrack.blobSizeThreshold); // (Solution)
		System.err.println("  target radius: " + blobTrack.targetRadius); // (Solution)
		System.err.println("  desired fixation distance: " + // (Solution)
				blobTrack.desiredFixationDistance); // (Solution)
		System.err.println("  translation error tolerance: " + // (Solution)
				blobTrack.translationErrorTolerance); // (Solution)
		System.err.println("  translation velocity gain: " + // (Solution)
				blobTrack.translationVelocityGain); // (Solution)
		System.err.println("  translation velocity max: " + // (Solution)
				blobTrack.translationVelocityMax); // (Solution)
		System.err.println("  rotation error tolerance: " + // (Solution)
				blobTrack.rotationErrorTolerance); // (Solution)
		System.err.println("  rotation velocity gain: " + // (Solution)
				blobTrack.rotationVelocityGain); // (Solution)
		System.err.println("  rotation velocity max: " + // (Solution)
				blobTrack.rotationVelocityMax); // (Solution)
		System.err.println("  use gaussian blur: " + blobTrack.useGaussianBlur); // (Solution)

		// initialize the ROS publication to command/Motors

		publisher = node.newPublisher("command/Motors", "rss_msgs/MotionMsg"); // (Solution)

		// End Student Code

		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
		vidSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(org.ros.message.sensor_msgs.Image message) {
				byte[] rgbData = Image.RGB2BGR(message.data,
						(int) message.width, (int) message.height);
				assert ((int) message.width == width);
				assert ((int) message.height == height);
				handle(rgbData);
			}
		});

		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(
					org.ros.message.rss_msgs.OdometryMsg message) {
				if (firstUpdate) {
					firstUpdate = false;
					gui.resetWorldToView(message.x, message.y);
				}
				gui.setRobotPose(message.x, message.y, message.theta);
			}
		});
		Thread runningStuff = new Thread(this);
		runningStuff.start();
	}

	@Override
	public void onShutdown(Node node) {
		if (node != null) {
			node.shutdown();
		}
	}

	@Override
	public void onShutdownComplete(Node node) {
	}

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("rss/visualservo");
	}

	public boolean canSeeBlock() {
		// TODO Auto-generated method stub
		return false;
	}
}
