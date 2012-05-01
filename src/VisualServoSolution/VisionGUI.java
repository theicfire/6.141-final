package VisualServoSolution;

import org.ros.message.MessageListener;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;
import org.ros.namespace.GraphName;
import org.ros.message.sensor_msgs.*;
import org.ros.node.topic.Publisher;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.awt.image.*;
import javax.swing.*;
import javax.swing.event.*;

import VisualServoSolution.Image;
import VisualServoSolution.VisionGUI;
import com.googlecode.javacv.CanvasFrame;
import com.googlecode.javacv.OpenCVFrameGrabber;
import com.googlecode.javacv.cpp.opencv_core.CvMemStorage;
import com.googlecode.javacv.cpp.opencv_core.IplImage;

import java.util.*;
import java.lang.reflect.*;

/**
 * <p>GUI for driving robot, display of robot pose, and vision images for
 * RSS-I.</p>
 *
 * <p>Can either be run as a stand-alone program with the provided {@link
 * #main} (recommended), or you can make an instance of it from within your own
 * program.</p>
 *
 * <h2>Coordinate Grid</h2>
 *
 * <p>A coordinate grid is displayed representing the robot world frame.  The X
 * axis is horizontal and positive to the right.  The Y axis is vertical and
 * positive up.  The X and Y axis lines, when in view, are displayed in a
 * highlighted color.  Other grid lines are plotted at 1m intervals.</p>
 *
 * <h2>Panning and Zooming</h2>
 *
 * <p>The graphics display is interactively navigable with the mouse.</p>
 *
 * <p>To pan: click and drag (left button).</p>
 *
 * <p>To zoom:
 * <ul>
 * <li>click and drag (left button) while holding CTRL</li>
 * <li>click and with any other button</li>
 * <li>rotate mouse wheel</li>
 * </ul></p>
 *
 * <h2>Pose Display</h2>
 *
 * <p>The most recently plotted pose will always be displayed in a highlighted
 * color.  A fraction of all the older poses (controlled by {@link
 * #poseSaveInterval}) will also be plotted, in a lighter color, to provide
 * you with a representation of the history of the robot's motion.</p>
 *
 * <h2>Stand-Alone Operation</h2>
 *
 * <p>When run stand-alone, connects to the Carmen daemons directly and
 * periodically plots the robot pose.  Vision images are displayed in the
 * upper-left corner as they are received. See the documentation for {@link
 * #main} for the command line arguments and other details for stand-alone
 * operation.</p>
 *
 * <p>In stand-alone mode you may clear all prior drawing like this:
 * <pre>
 *   (new GUIEraseMessage(VisionGUI.ERASE_POSE)).publish();
 *   (new GUIEraseMessage(VisionGUI.ERASE_POSE_HISTORY)).publish();
 *   (new GUIEraseMessage(VisionGUI.ERASE_VISION_IMAGE)).publish();
 * </pre></p>
 *
 * <h2>Running from Within Your Own Program</h2>
 *
 * <p>When run from within your own program, this class <b>does not</b>
 * automatically plot anything.  If you wish to plot the robot pose, you will
 * need to call {@link #setRobotPose} as you receive odometry messages from
 * Carmen.  Subscribe to the appropriate Carmen messages and call the other
 * drawing methods as you desire to plot other data.</p>
 *
 * <p>Note that it is generally preferable design your code to run the GUI in
 * stand-alone mode so that you can run it on a different machine than the one
 * which runs your robot's operational code.</p>
 *
 * <h3>Drawing Data Manually</h3>
 *
 * <p>When run from within your own program, you will need to write code to
 * manually draw things in the GUI.</p>
 *
 * <p>Drawing example:
 * <pre>
 *   VisionGUI vg = new VisionGUI();
 *
 *   public void handle (OdometryMessage message) {
 *     vg.setRobotPose(message.x, message.y, message.theta);
 *   }
 *
 *   public void handle (VisionImageMessage message) {
 *     vg.setVisionImage(message.image, message.width, message.height);
 *   }
 * </pre></p>
 *
 * <p>To clear all prior drawing:
 * <pre>
 *   vg.erasePoseHistory();
 *   vg.erasePose();
 *   vg.eraseVisionImage();
 * </pre></p>
 *   
 * <h2>Driving the Robot</h2>
 *
 * <p>If {@link #maxTV} and {@link #maxRV} are both greater than zero then you
 * can drive the robot around using the keyboard:
 * <ul>
 * <li>space - stop</li>
 * <li>i - fwd</li>
 * <li>, - rev</li>
 * <li>u - fwd turning CCW</li>
 * <li>o - fwd turning CW</li>
 * <li>m - rev turning CW</li>
 * <li>. - rev turning CCW</li>
 * <li>j - turn CCW</li>
 * <li>l - turn CW</li>
 * </ul></p>
 *
 * <p>TBD:
 * <ul>
 * <li>sliders for max robot velocities</li>
 * <li>checkboxes to enable drawing each kind of item</li>
 * <li>read robot dims from Carmen (requires more detail from Carmen)</li>
 * </ul></p>
 *
 * @author vona
 **/
public class VisionGUI extends JPanel implements NodeMain {
	
	Publisher<MotionMsg> publisher;

	/**
	 * <p> Node for ROS communication </p>
	 */
	public Node node;
	
	/**
	 * <p>The application name.</p>
	 **/
	public static final String APPNAME = "VisionGUI";

	/**
	 * <p>Erase all object types.</p>
	 **/
	public static final int ERASE_ALL = ~0;

	/**
	 * <p>Needed as JPanel is seralizable</p>
	 **/
	static final long serialVersionUID = 42;

	/**
	 * <p>Bitfield constant for {@link GUIEraseMessage}.</p>
	 **/
	public static final int ERASE_POSE = 1<<0;

	/**
	 * <p>Bitfield constant for {@link GUIEraseMessage}.</p>
	 **/
	public static final int ERASE_POSE_HISTORY = 1<<1;

	/**
	 * <p>Bitfield constant for {@link GUIEraseMessage}.</p>
	 **/
	public static final int ERASE_VISION_IMAGE = 1<<2;

	/**
	 * <p>Coordinate grid line spacing in meters.</p>
	 **/
	public static final double GRID_SPACING = 1.0;

	/**
	 * <p>Line width of the lines making up the coordinate grid in pixels.</p>
	 **/
	public static final float GRID_LINE_WIDTH = 1.0f;

	/**
	 * <p>Line width of axes lines in pixels.</p>
	 **/
	public static final float AXES_LINE_WIDTH = 1.5f;

	/**
	 * <p>Line width of the lines making up a {@link VisionGUI.Pose} in
	 * pixels.</p>
	 **/
	public static final float POSE_LINE_WIDTH = 1.5f;

	/**
	 * <p>Default color for the coordinate system grid.</p>
	 **/
	public static final Color GRID_COLOR = Color.LIGHT_GRAY;

	/**
	 * <p>Default color for the axes.</p>
	 **/
	public static final Color AXES_COLOR = Color.BLUE;

	/**
	 * <p>Default color for historical {@link VisionGUI.Pose}s.</p>
	 **/
	public static final Color OLD_POSE_COLOR = Color.GRAY;

	/**
	 * <p>Default color for the newest {@link VisionGUI.Pose}.</p>
	 **/
	public static final Color NEW_POSE_COLOR = Color.GREEN;

	/**
	 * <p>Factor by which to accelerate pan if SHIFT is down.</p>
	 **/
	public static final double PAN_ACCEL_FACTOR = 4.0;

	/**
	 * <p>Factor by which to accelerate zoom if SHIFT is down.</p>
	 **/
	public static final double ZOOM_ACCEL_FACTOR = 4.0;

	/**
	 * <p>Conversion factor of pixels to zoom.</p>
	 **/
	public static final double ZOOM_PER_PIXEL = 0.01;

	/**
	 * <p>Mouse wheel clicks to drag delta.</p>
	 **/
	public static final double WHEEL_TO_DELTA = 5.0;

	/**
	 * <p>Default width.</p>
	 **/
	public static final int DEFAULT_WIDTH = 800;

	/**
	 * <p>Default canvas height.</p>
	 **/
	public static final int DEFAULT_HEIGHT = DEFAULT_WIDTH;

	/**
	 * <p>Default {@link #scale}.</p>
	 **/
	public static final double DEFAULT_SCALE = DEFAULT_WIDTH/10.0;

	/**
	 * <p>Default {@link #cx}.</p>
	 **/
	public static final double DEFAULT_CX = 0.0;

	/**
	 * <p>Default {@link #cy}.</p>
	 **/
	public static final double DEFAULT_CY = 0.0;

	/**
	 * <p>Default number of odometry updates to wait between saving robot
	 * pose.</p>
	 **/
	public static final int DEFAULT_POSE_SAVE_INTERVAL = 10;

	/**
	 * <p>Whether to render double-buffered.</p>
	 **/
	public static final boolean DOUBLE_BUFFERED = true;

	/**
	 * <p>Whether to use <code>RenderingHints.VALUE_ANTIALIAS_ON</code> and
	 * <code>RenderingHints.VALUE_TEXT_ANTIALIAS_ON</code>.</p>
	 **/
	public static final boolean ANTIALIASING = true;

	/**
	 * <p>The rendering quality to use.</p>
	 **/
	public static final Object RENDERING_QUALITY =
		RenderingHints.VALUE_RENDER_QUALITY;

	/**
	 * <p>The interpolation to use.</p>
	 **/
	public static final Object INTERPOLATION =
		RenderingHints.VALUE_INTERPOLATION_BICUBIC;

	/**
	 * <p>Whether to use
	 * <code>RenderingHints.VALUE_FRACTIONALMETRICS_ON</code>.</p>
	 **/
	public static final boolean FRACTIONALMETRICS = true;

	/**
	 * <p>Whether to use <code>RenderingHints.VALUE_STROKE_NORMALIZE</code>.</p>
	 **/
	public static final boolean STROKE_NORMALIZATION = true;

	/**
	 * <p>Width of the body (Y direction) in meters.</p>
	 **/
	public static final double BODY_WIDTH = 0.38;

	/**
	 * <p>Length of the body (X direction) in meters.</p>
	 **/
	public static final double BODY_LENGTH = 0.38;

	/**
	 * <p>Width of the wheel (Y direction) in meters.</p>
	 **/
	public static final double WHEEL_WIDTH = 0.02;

	/**
	 * <p>Length of the wheel (X direction) in meters.</p>
	 **/
	public static final double WHEEL_LENGTH = 0.12;

	/**
	 * <p>Y displacement of the left wheel rect relative to robot ctr (m).</p>
	 **/
	public static final double WHEEL_DISPLACEMENT = 0.21;

	/**
	 * <p>X displacement of the body rect relative to robot ctr (m).</p>
	 **/
	public static final double BODY_DISPLACEMENT = -0.09;

	/**
	 * <p>Wheel shape centered on origin in world frame (m).</p>
	 **/
	public static final Shape WHEEL_SHAPE =
		new RoundRectangle2D.Double(
				-WHEEL_LENGTH/2.0, -WHEEL_WIDTH/2.0,
				WHEEL_LENGTH, WHEEL_WIDTH,
				0.005, 0.005);

	/**
	 * <p>Body shape centered on origin in world frame (m).</p>
	 **/
	public static final Shape BODY_SHAPE =
		new Rectangle2D.Double(
				-BODY_LENGTH/2.0, -BODY_WIDTH/2.0,
				BODY_LENGTH, BODY_WIDTH);

	/**
	 * <p>Aggregate shape of the robot at pose (0, 0, 0).</p>
	 **/
	public static final GeneralPath ROBOT_SHAPE = new GeneralPath();

	static {
		AffineTransform t = new AffineTransform();

		t.setToIdentity();
		t.translate(BODY_DISPLACEMENT, 0.0);
		ROBOT_SHAPE.append(t.createTransformedShape(BODY_SHAPE), false);

		t.setToIdentity();
		t.translate(0.0, WHEEL_DISPLACEMENT);
		ROBOT_SHAPE.append(t.createTransformedShape(WHEEL_SHAPE), false);

		t.setToIdentity();
		t.translate(0.0, -WHEEL_DISPLACEMENT);
		ROBOT_SHAPE.append(t.createTransformedShape(WHEEL_SHAPE), false);

		ROBOT_SHAPE.append(new Line2D.Double(0.0, 0.0, 0.01, 0.0), false);
		ROBOT_SHAPE.append(new Line2D.Double(0.0, 0.0, 0.0, 0.01), false);
	}

	/**
	 * <p>Scale that takes one world meter to one pixel.</p>
	 **/
	protected double scale = DEFAULT_SCALE;

	/**
	 * <p>View center x world coord in meters.</p>
	 **/
	protected double cx = DEFAULT_CX;

	/**
	 * <p>View center y world coord in meters.</p>
	 **/
	protected double cy = DEFAULT_CY;

        protected boolean firstUpdate = true;

	/**
	 * <p>Whether to sacrifice rendering quality for speed.</p>
	 **/
	protected boolean renderFastest = false;

	/**
	 * <p>Whether to paint the grid.</p>
	 **/
	protected boolean gridEnabled = true;

	/**
	 * <p>Whether to paint the current robot pose.</p>
	 **/
	protected boolean currentPoseEnabled = true;

	/**
	 * <p>Whether to paint the historical robot poses.</p>
	 **/
	protected boolean historicalPosesEnabled = true;

	/**
	 * <p>Whether to paint the vision image.</p>
	 **/
	protected boolean visionImageEnabled = true;

	/**
	 * <p>The frame containing this GUI.</p>
	 **/
	protected JFrame frame;

	/**
	 * <p>Transform that takes points in world coordinates (meters) to points
	 * in view coordinates (pixels).</p>
	 **/
	protected AffineTransform worldToView = new AffineTransform();

	/**
	 * <p>Total time it took in milliseconds to render the last frame.</p>
	 **/
	protected double lastFrameTime = 0.0;

	/**
	 * <p>Frame time in ms above which to force fast rendering.</p>
	 **/
	public static final double FORCE_FAST_RENDER_THRESHOLD = 300.0;

	/**
	 * <p>Frame time in ms below which to un-force fast rendering.</p>
	 **/
	public static final double UN_FORCE_FAST_RENDER_THRESHOLD = 5.0;

    public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
    public Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	/**
	 * <p>A paintable graphical object.</p>
	 **/
	protected abstract class Glyph {

		/**
		 * <p>Paint this glyph.</p>
		 *
		 * @param g2d the graphics context
		 **/
		public abstract void paint(Graphics2D g2d);
	}

	/**
	 * <p>A robot pose.</p>
	 **/
	protected class Pose extends Glyph {

		/**
		 * <p>The robot shape in the specified pose in world frame.</p>
		 **/
		Shape robot;

		/**
		 * <p>Create a new unset pose.</p>
		 **/
		Pose() {
			robot = null;
		}

		/**
		 * <p>Create a new set pose.</p>
		 **/
		Pose(double x, double y, double theta) {
			set(x, y, theta);
		}

		/**
		 * <p>(Re)Set this pose.</p>
		 **/
		void set(double x, double y, double theta) {

			AffineTransform t = new AffineTransform();
			t.setToIdentity();
			t.translate(x, y);
			t.rotate(theta);

			robot = t.createTransformedShape(ROBOT_SHAPE);
		}

		/**
		 * <p>Unset this pose.</p>
		 **/
		void unset() {
			robot = null;
		}

		/**
		 * <p>Paints the pose.</p>
		 *
		 * <p>Assumes line width and color are already set.</p>
		 *
		 * @param g2d the graphics context
		 **/
		public void paint(Graphics2D g2d) {
			if (robot != null)
				g2d.draw(robot);
		}
	}

	/**
	 * <p>Displays images from the robot's camera.</p>
	 **/
	protected class VisionImage extends Glyph {

		/**
		 * <p>The pixel buffer for the displayed image.</p>
		 **/
		int packedImage[] = null;

		/**
		 * <p>Java image animation machinery.</p>
		 **/
		MemoryImageSource source = null;

		/**
		 * <p>The actual image we paint, null if none.</p>
		 **/
		java.awt.Image image = null;

		/**
		 * <p>Currently displayed image width or -1 if none.</p>
		 **/
		int width = -1;

		/**
		 * <p>Currently displayed image height or -1 if none.</p>
		 **/
		int height = -1;

		/**
		 * <p>Create a new vision image, initially not displaying anything.</p>
		 **/
		VisionImage() {
			unset();
		}

		/**
		 * <p>Set to display an image.</p>
		 *
		 * @param unpackedImage the unpacked RGB image (a copy is made)
		 * @param width image width
		 * @param height image height 
		 **/
		void set(byte[] unpackedImage, int width, int height) {

			if ((unpackedImage == null) || (width <= 0) || (height <= 0)) {
				unset();
				return;
			}

			boolean reConsedPacked = false;
			if ((packedImage == null) || 
					(this.width != width) || (this.height != height)) {
				packedImage = new int[width*height];
				reConsedPacked = true;
			}

			int srcIndex = 0;
			int destIndex = 0;

			for (int y = 0; y < height; y++) {
				for (int x = 0; x < width; x++) {
					int red = unpackedImage[srcIndex++] & 0xff;
					int green = unpackedImage[srcIndex++] & 0xff;
					int blue = unpackedImage[srcIndex++] & 0xff;
					packedImage[destIndex++] =
						(0xff << 24) | (red << 16) | (green << 8) | blue;
				}
			}

			if (reConsedPacked || (image == null) ||
					(this.width != width) || (this.height != height)) {

				source = new MemoryImageSource(width, height, packedImage, 0, width);
				source.setAnimated(true);

				image = createImage(source);

			} else {
				source.newPixels();
			}

			this.width = width;
			this.height = height;
		}

		/**
		 * <p>Disable display.</p>
		 **/
		void unset() {
			image = null;
			width = -1;
			height = -1;
		}

		/**
		 * <p>Paints the image, if any.</p>
		 *
		 * <p>Assumes g2d is in view coordinates.</p>
		 *
		 * @param g2d the graphics context
		 **/
		public void paint(Graphics2D g2d) {

			if (image == null)
				return;   

			g2d.drawImage(image, 0, 0, VisionGUI.this);
		}
	}

	/**
	 * <p>The current (i.e. most recent) robot pose.</p>
	 **/
	protected Pose currentPose = new Pose();

	/**
	 * <p>All the historical {@link VisionGUI.Pose}s.</p>
	 **/
	protected java.util.List<Pose> historicalPoses =
		Collections.synchronizedList(new ArrayList<Pose>());

	/**
	 * <p>The one {@link VisionGUI.VisionImage}.</p>
	 **/
	protected VisionImage visionImage = new VisionImage();

	/**
	 * <p>How many poses to skip before saving a history pose.</p>
	 *
	 * <p>Zero indicates that every pose is saved.  Negative indicates that no
	 * poses are saved.</p>
	 **/
	protected int poseSaveInterval;

	/**
	 * <p>Synchronization object for {@link #poseSaveInterval} and {@link
	 * #posesToGoBeforeSave}.</p>
	 **/
	protected Object poseSaveLock = new Object();

	/**
	 * <p>How many poses we have left to go before saving one (if enabled).</p>
	 **/
	protected int posesToGoBeforeSave = 0;

	/**
	 * <p>Current maximum translation velocity (m/s).</p>
	 *
	 * <p>Zero or negative disables driving.</p>
	 **/
	protected double maxTV = 0.0;

	/**
	 * <p>Current maximum rotational velocity (rad/s).</p>
	 *
	 * <p>Zero or negative disables driving.</p>
	 **/
	protected double maxRV = 0.0;

	/**
	 * <p>Current translation velocity command (m/s).</p>
	 **/
	protected double tv = 0.0;

	/**
	 * <p>Current maximum rotation velocity (rad/s).</p>
	 **/
	protected double rv = 0.0;

	/**
	 * <p>Consruct a new VisionGUI.</p>
	 *
	 * <p>The new viewer will automatically appear in a new window.</p>
	 *
	 * @param poseSaveInterval the number of robot pose updates to skip between
	 * saving a persistent snapshot of the pose.  Zero indicates that every pose
	 * should be saved.  Negative indicates that <i>no</i> poses should be saved.
	 * @param maxTV max translation velocity in m/s.  Setting this less than or
	 * equal to 0.0 disables driving.  See also {@link #setMaxTV}.
	 * @param maxRV max translation velocity in m/s.  Setting this less than or
	 * equal to 0.0 disables driving.  See also {@link #setMaxRV}.
	 **/
	public VisionGUI(int poseSaveInterval, double maxTV, double maxRV) {

		this.poseSaveInterval = poseSaveInterval;

		this.maxTV = maxTV;
		this.maxRV = maxRV;

		resetWorldToView(DEFAULT_CX, DEFAULT_CY, DEFAULT_SCALE);

		//do this here, not in invokeLater, so we can configure frame in
		//instanceMain()
		frame = new JFrame(getAppName());

		SwingUtilities.invokeLater(new Runnable() {
			public void run() {

				setBackground(Color.WHITE);
				setPreferredSize(new Dimension(DEFAULT_WIDTH, DEFAULT_HEIGHT));
				setOpaque(true);
				setDoubleBuffered(true);

				Container contentPane = frame.getContentPane();
				contentPane.setBackground(Color.WHITE);
				contentPane.setLayout(new BorderLayout());
				contentPane.add(VisionGUI.this, "Center");

				frame.pack(); //do this before working on focus!

				addKeyListener(keyListener);

				setFocusable(true);
				requestFocusInWindow();

				frame.setLocationByPlatform(true);
				frame.setVisible(true);

				setMaxTV(0.50);
				setMaxRV(0.10);
			}
		});
	}

	/**
	 * <p>Covers {@link #VisionGUI(int, double, double)}, disables driving.</p>
	 **/
	public VisionGUI(int poseSaveInterval) {
		this(poseSaveInterval, 0.0, 0.0);
	}

	/**
	 * <p>Covers {@link #VisionGUI(int)}, always uses {@link
	 * #DEFAULT_POSE_SAVE_INTERVAL}.</p>
	 **/
	public VisionGUI() {
		this(DEFAULT_POSE_SAVE_INTERVAL);
	}

	/**
	 * <p>Clamp v to [-|m|, |m|].</p>
	 *
	 * @param v the value to clamp
	 * @param m the absolute max
	 * @return the clamped value
	 **/
	public static double clamp(double v, double m) {
		m = Math.abs(m);
		if (v > m)
			v = m;
		if (v < -m)
			v = -m;
		return v;
	}

	/**
	 * <p>Get the frame containing this GUI.</p>
	 *
	 * @return the frame containing this GUI
	 **/
	public JFrame getFrame() {
		return frame;
	}

	/**
	 * <p>Get the title for the GUI frame.</p>
	 *
	 * <p>Default impl returns {@link #APPNAME}.</p>
	 *
	 * @return the title for the GUI frame
	 **/
	public String getAppName() {
		return APPNAME;
	}

	/**
	 * <p>(Re)Set the max translation velocity for driving.</p>
	 *
	 * @param maxTV the max translation velocity in m/s.  Less than or equal to
	 * zero disables driving.
	 **/
	public void setMaxTV(double maxTV) {
		synchronized(keyListener) {

			this.maxTV = maxTV;


		}
	}

	/**
	 * <p>(Re)Set the max rotation velocity for driving.</p>
	 *
	 * @param maxRV the max rotation velocity in rad/s.  Less than or equal to
	 * zero disables driving.
	 **/
	public void setMaxRV(double maxRV) {
		synchronized(keyListener) {

			this.maxRV = maxRV;


		}
	}

	/**
	 * <p>Set a new {@link #poseSaveInterval}.</p>
	 *
	 * @param poseSaveInterval the number of robot pose updates to skip between
	 * saving a persistent snapshot of the pose.  Zero indicates that every pose
	 * should be saved.  Negative indicates that <i>no</i> poses should be saved.
	 **/
	public void setPoseSaveInterval(int poseSaveInterval) {
		synchronized (poseSaveLock) {
			this.poseSaveInterval = poseSaveInterval;
			posesToGoBeforeSave = 0;
		}
	}

	/**
	 * <p>Set the robot pose to display.</p>
	 *
	 * <p>Note: if you are running the GUI in stand-alone mode, this will be
	 * automatically called when Carmen odometry messages are recieved.</p>
	 *
	 * <p>If {@link #poseSaveInterval} is non-negative, then it indicates how
	 * many calls to this method are skipped between a historical pose is
	 * saved.</p>
	 *
	 * @param x the robot center x position in world frame (m)
	 * @param y the robot center y position in world frame (m)
	 * @param theta the ccw orientation of the robot frame x-axis wrt the world
	 * frame x-axis (rad)
	 **/
	public void setRobotPose(double x, double y, double theta) {

		synchronized (currentPose) {
			currentPose.set(x, y, theta);
		}

		boolean addToHistory = false;
		synchronized (poseSaveLock) {
			if (poseSaveInterval >= 0) {

				if (posesToGoBeforeSave == 0) {
					addToHistory = true;
					posesToGoBeforeSave = poseSaveInterval;
				} else {
					posesToGoBeforeSave--;
				}
			}
		}

		if (addToHistory)
			historicalPoses.add(new Pose(x, y, theta));

		repaint();
	}

	/**
	 * <p>Set the vision image for display.</p>
	 *
	 * <p>Note: if you are running the GUI in stand-alone mode, this will be
	 * automatically called when Carmen vision image messages are recieved.</p>
	 *
	 * @param image is an unpacked <code>width</code> by <code>height</code> RGB
	 * image
	 * @param width the image width
	 * @param height the image height
	 **/
	public void setVisionImage(byte[] image, int width, int height) {
		synchronized (visionImage) {
			visionImage.set(image, width, height);
		}
		repaint();
	}

	/**
	 * <p>Erase the current robot pose, if any.</p>
	 **/
	public void erasePose() {
		synchronized (currentPose) {
			currentPose.unset();
		}
		repaint();
	}

	/**
	 * <p>Erase all historical robot poses.</p>
	 **/
	public void erasePoseHistory() {
		historicalPoses.clear();
		repaint();
	}

	/**
	 * <p>Erase the vision image, if any.</p>
	 **/
	public void eraseVisionImage() {
		synchronized (visionImage) {
			visionImage.unset();
		}
	}

	/**
	 * <p>Set the line width in <code>g2d</code>.</p>
	 *
	 * @param g2d the Graphics2D in which to set the appearance
	 * @param lineWidth the line width (pixels)
	 **/
	protected void setLineWidth(Graphics2D g2d, float lineWidth) {
		g2d.setStroke(new BasicStroke((float) (lineWidth/scale)));
	}

	/**
	 * <p>Calls {@link #paintComponent(Graphics2D)}.</p>
	 *
	 * @param g the paint context
	 **/
	public void paintComponent(Graphics g) {
		paintComponent((Graphics2D) g);
	}

	/**
	 * <p>Calls superclass impl, then {@link #paintContents}.</p>
	 **/
	protected void paintComponent(Graphics2D g2d) {
		super.paintComponent(g2d);
		paintContents(g2d);
	}

	/**
	 * <p>Paint all the graphics.</p>
	 * 
	 * @param g2d the paint context
	 **/
	protected void paintContents(Graphics2D g2d) {

		double startTime = System.currentTimeMillis();

		if (lastFrameTime > FORCE_FAST_RENDER_THRESHOLD)
			renderFastest = true;

		if (lastFrameTime < UN_FORCE_FAST_RENDER_THRESHOLD)
			renderFastest = false;

		if (renderFastest) {

			g2d.setRenderingHint(RenderingHints.KEY_RENDERING,
					RenderingHints.VALUE_RENDER_SPEED);

			g2d.setRenderingHint(RenderingHints.KEY_INTERPOLATION, 
					RenderingHints.
					VALUE_INTERPOLATION_NEAREST_NEIGHBOR);

			g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
					RenderingHints.VALUE_ANTIALIAS_OFF);

			g2d.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,
					RenderingHints.VALUE_TEXT_ANTIALIAS_OFF);

			g2d.setRenderingHint(RenderingHints.KEY_FRACTIONALMETRICS,
					RenderingHints.VALUE_FRACTIONALMETRICS_OFF);

			g2d.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL,
					RenderingHints.VALUE_STROKE_PURE);
		} else {

			g2d.setRenderingHint(RenderingHints.KEY_RENDERING, RENDERING_QUALITY);
			g2d.setRenderingHint(RenderingHints.KEY_INTERPOLATION, INTERPOLATION);

			if (ANTIALIASING) {
				g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
						RenderingHints.VALUE_ANTIALIAS_ON);

				g2d.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,
						RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
			}

			if (FRACTIONALMETRICS)
				g2d.setRenderingHint(RenderingHints.KEY_FRACTIONALMETRICS,
						RenderingHints.VALUE_FRACTIONALMETRICS_ON);

			if (STROKE_NORMALIZATION)
				g2d.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL,
						RenderingHints.VALUE_STROKE_NORMALIZE);

		}

		AffineTransform transformWas = g2d.getTransform();

		synchronized(worldToView) {
			g2d.transform(worldToView);

			paintInWorldUnderGridHook(g2d);

			if (gridEnabled)
				paintGrid(g2d);

			paintInWorldOverGridUnderPosesHook(g2d);

			if (currentPoseEnabled)
				paintCurrentPose(g2d);

			if (historicalPosesEnabled)
				paintPoseHistory(g2d);

			paintInWorldOverPosesHook(g2d);
		}

		g2d.setTransform(transformWas);

		paintInViewUnderVisionImageHook(g2d);

		if (visionImageEnabled)
			paintVisionImage(g2d);

		paintInViewOverVisionImageHook(g2d);

		lastFrameTime = System.currentTimeMillis() - startTime;

		//    System.err.println("frame time = " + lastFrameTime);
	}

	/**
	 * <p>Hook to add painting in world frame under the coordinate grid.</p>
	 **/
	protected void paintInWorldUnderGridHook(Graphics2D g2d) {
	}

	/**
	 * <p>Hook to add painting in world frame over the coordinate grid but under
	 * the robot poses.</p>
	 **/
	protected void paintInWorldOverGridUnderPosesHook(Graphics2D g2d) {
	}

	/**
	 * <p>Hook to add painting in world frame over the robot poses.</p>
	 **/
	protected void paintInWorldOverPosesHook(Graphics2D g2d) {
	}

	/**
	 * <p>Hook to add painting in view frame under the vision image.</p>
	 **/
	protected void paintInViewUnderVisionImageHook(Graphics2D g2d) {
	}

	/**
	 * <p>Hook to add painting in view frame over the vision image.</p>
	 **/
	protected void paintInViewOverVisionImageHook(Graphics2D g2d) {
	}

	/**
	 * <p>Paint the coordinate grid.</p>
	 *
	 * @param g2d the graphics context
	 **/
	protected void paintGrid(Graphics2D g2d) {

		setLineWidth(g2d, GRID_LINE_WIDTH);
		g2d.setColor(GRID_COLOR);

		//current viewport dimensions in meters
		double width = ((double)getWidth())/scale;
		double height = ((double)getHeight())/scale;

		//current viewport bounds in meters
		double xMin = cx - width/2.0;
		double xMax = cx + width/2.0;

		double yMin = cy - height/2.0;
		double yMax = cy + height/2.0;

		Line2D.Double l = new Line2D.Double();

		//draw vertical lines

		l.y1 = yMin;
		l.y2 = yMax;

		for (double x = Math.ceil(xMin/GRID_SPACING)*GRID_SPACING;
		x <= xMax;
		x += GRID_SPACING) {
			l.x1 = x;
			l.x2 = x;
			g2d.draw(l);
		}

		//draw horizontal lines

		l.x1 = xMin;
		l.x2 = xMax;

		for (double y = Math.ceil(yMin/GRID_SPACING)*GRID_SPACING;
		y <= yMax;
		y += GRID_SPACING) {
			l.y1 = y;
			l.y2 = y;
			g2d.draw(l);
		}

		//draw axes, maybe
		setLineWidth(g2d, AXES_LINE_WIDTH);
		g2d.setColor(AXES_COLOR);

		if ((xMin < 0.0) && (xMax > 0.0)) {
			l.y1 = yMin;
			l.y2 = yMax;
			l.x1 = 0.0;
			l.x2 = 0.0;
			g2d.draw(l);
		}

		if ((yMin < 0.0) && (yMax > 0.0)) {
			l.y1 = 0.0;
			l.y2 = 0.0;
			l.x1 = xMin;
			l.x2 = xMax;
			g2d.draw(l);
		}
	}

	/**
	 * <p>Paint {@link #currentPose}.</p>
	 *
	 * @param g2d the graphics context
	 **/
	protected void paintCurrentPose(Graphics2D g2d) {

		//avoid NPE on init
		if (currentPose == null)
			return;

		setLineWidth(g2d, POSE_LINE_WIDTH);
		g2d.setColor(NEW_POSE_COLOR);

		synchronized (currentPose) {
			currentPose.paint(g2d);
		}
	}

	/**
	 * <p>Paint all {@link #historicalPoses}.</p>
	 *
	 * @param g2d the graphics context
	 **/
	protected void paintPoseHistory(Graphics2D g2d) {

		//avoid NPE on init
		if (historicalPoses == null)
			return;

		setLineWidth(g2d, POSE_LINE_WIDTH);
		g2d.setColor(OLD_POSE_COLOR);

		synchronized (historicalPoses) {
			for (Iterator it = historicalPoses.iterator(); it.hasNext(); )
				((Pose) it.next()).paint(g2d);
		}
	}

	/**
	 * <p>Paint {@link #visionImage}.</p>
	 *
	 * @param g2d the graphics context
	 **/
	protected void paintVisionImage(Graphics2D g2d) {

		//avoid NPE on init
		if (visionImage == null)
			return;

		synchronized (visionImage) {
			visionImage.paint(g2d);
		}
	}

	/**
	 * <p>Updates {@link #worldToView} to be centered at <code>(cx, cy)</code>
	 * with the given scale.</p>
	 *
	 * @param cx the center x world coordinate in meters
	 * @param cy the center y world coordinate in meters
	 * @param scale scale that takes one world unit to one device unit
	 **/
	public void resetWorldToView(double cx, double cy, double scale) {

		this.cx = cx;
		this.cy = cy;
		this.scale = scale;

		double vWidth = (double) getWidth();
		double vHeight = (double) getHeight();

		if (vWidth == 0.0)
			vWidth = DEFAULT_WIDTH;

		if (vHeight == 0.0)
			vHeight = DEFAULT_HEIGHT;

		double tx = vWidth/2.0-scale*cx;
		double ty = vHeight/2.0+scale*cy;

		//    System.out.println("translation: (" + tx + ", " + ty + ")");

		synchronized (worldToView) {
			worldToView.setTransform(scale, 0.0, 0.0, -scale, tx, ty);
		}

		repaint();
	}

	/**
	 * <p>Updates {@link #worldToView} to be centered at <code>(cx,
	 * cy)</code>.</p>
	 *
	 * @param cx the center x world coordinate in meters
	 * @param cy the center y world coordinate in meters
	 **/
	public void resetWorldToView(double cx, double cy) {
		resetWorldToView(cx, cy, scale);
	}

	/**
	 * <p>Updates {@link #worldToView} with the given scale.</p>
	 *
	 * @param scale scale that takes one world unit to one device unit
	 **/
	public void zoomWorldToView(double scale) {
		resetWorldToView(cx, cy, scale);
	}

	/**
	 * <p>Gets the world coordinates from a view point.</p>
	 *
	 * @param point the point in view coords
	 *
	 * @return the point in world coords
	 **/
	public Point2D.Double viewToWorld(Point2D.Double point) {
		synchronized (worldToView) {
			try {
				worldToView.inverseTransform(point, point);
				return point;
			} catch (NoninvertibleTransformException ex) {
				System.err.println("warning: " + ex.getStackTrace());
				return null;
			}
		}
	}

	/**
	 * <p>Implements mouse navigation.</p>
	 *
	 * <p>See class header doc for details.</p>
	 **/
	protected MouseInputListener mouseInputListener =
		new MouseInputAdapter() {

		{
			addMouseMotionListener(this);
			addMouseListener(this);
			addMouseWheelListener(new MouseWheelListener() {

				public void mouseWheelMoved(MouseWheelEvent e) {

					int modifiers = e.getModifiersEx();
					boolean accel = isAccel(modifiers);

					zoomCenter.setLocation(e.getPoint());
					viewToWorld(zoomCenter);

					doZoom(deltaToZoom(WHEEL_TO_DELTA*e.getWheelRotation(), accel),
							modifiers);
				}
			});
		}

		/**
		 * <p>Set up for a mouse click or drag.</p>
		 **/
		 public void mousePressed(MouseEvent e) {

			 Point2D point = e.getPoint();

			 prevDragPoint.setLocation(point);

			 startDragPoint.setLocation(point);
			 viewToWorld(startDragPoint);

			 zoomCenter.setLocation(startDragPoint);

			 dragInProgress = false;
		 }

		 /**
		  * <p>End a mouse click or drag.</p>
		  **/
		 public void mouseReleased(MouseEvent e) {
			 if (!dragInProgress) {
				 mouseMoved(e);
				 handleClick(e);
			 } else {
				 dragInProgress = false;
			 }
		 }

		 /**
		  * <p>Handle non-drag motion.</p> 
		  *
		  * <p>Currently ignored.</p>
		  **/
		 public void mouseMoved(MouseEvent e) {

			 if (dragInProgress)
				 return;

		 }

		 /**
		  * <p>Handle mouse drag.</p>
		  *
		  * <p>Currently just implements navigation.</p>
		  **/
		 public void mouseDragged(MouseEvent e) {

			 dragInProgress = true;

			 if (prevDragPoint == null)
				 return;

			 currentDragPoint.setLocation(e.getPoint());
			 int modifiers = e.getModifiersEx();
			 boolean accel = isAccel(modifiers);

			 //      System.out.println(e.getModifiersExText(modifiers));

			 double dx = currentDragPoint.getX() - prevDragPoint.getX();
			 double dy = currentDragPoint.getY() - prevDragPoint.getY();

			 prevDragPoint.setLocation(currentDragPoint);

			 viewToWorld(currentDragPoint);

			 if (isPan(modifiers))
				 doPan(deltaToPanX(dx, accel), deltaToPanY(dy, accel));

			 if (isZoom(modifiers))
				 doZoom(deltaToZoom(dy, accel), modifiers);
		 }

		 /**
		  * <p>Handle a mouse click.</p>
		  *
		  * <p>Currently ignored.</p>
		  **/
		 public void handleClick(MouseEvent e) {
		 }

		 /**
		  * <p>Check whether <code>modifiers</code> indicates a zoom.</p>
		  *
		  * <p>Current impl just looks for CTRL down or button 2 or 3.</p>
		  *
		  * @param modifiers the extended modifiers
		  *
		  * @return true iff <code>modifiers</code> indicates a zoom
		  **/
		 public boolean isZoom(int modifiers) {
			 return 
			 ((modifiers & InputEvent.CTRL_DOWN_MASK) != 0) ||
			 ((modifiers & InputEvent.BUTTON2_DOWN_MASK) != 0) ||
			 ((modifiers & InputEvent.BUTTON3_DOWN_MASK) != 0);
		 }

		 /**
		  * <p>Check whether <code>modifiers</code> indicates a pan.</p>
		  *
		  * <p>Current impl just returns <code>!{@link #isZoom}</code>.</p>
		  *
		  * @param modifiers the extended modifiers
		  *
		  * @return true iff <code>modifiers</code> indicates a pan
		  **/
		 public boolean isPan(int modifiers) {
			 return !isZoom(modifiers);
		 }

		 /**
		  * <p>Check whether <code>modifiers</code> indicates an accelerated
		  * navigation.</p>
		  *
		  * <p>Current impl just looks for SHIFT down.</p>
		  *
		  * @param modifiers the extended modifiers
		  *
		  * @return true iff <code>modifiers</code> indicates an accelerated
		  * navigation
		  **/
		 public boolean isAccel(int modifiers) {
			 return (modifiers & InputEvent.SHIFT_DOWN_MASK) != 0;
		 }

		 /**
		  * <p>Compute a zoom amount from a drag delta.</p>
		  *
		  * @param dy delta y, in viewpoint coordinates
		  * @param accel whether to multiply the return value by {@link
		  * #ZOOM_ACCEL_FACTOR}
		  *
		  * @return the new view scale
		  **/
		 public double deltaToZoom(double dy, boolean accel) {

			 double factor = Math.abs(dy)*ZOOM_PER_PIXEL;

			 if (accel)
				 factor *= ZOOM_ACCEL_FACTOR;

			 factor += 1.0;

			 if (dy < 0.0)
				 factor = 1.0/factor;

			 double newScale = scale;

			 //    log.debug("factor: " + factor + ", newScale: " + newScale);

			 newScale *= factor;

			 return newScale;
		 }

		 /**
		  * <p>Compute pan x from delta x, in viewpoint coordinates.</p>
		  *
		  * @param dx delta x, in viewpoint coordinates
		  * @param accel whether to multiply the return value by {@link
		  * #PAN_ACCEL_FACTOR}
		  *
		  * @return the amount to pan x, in viewpoint coordinates
		  **/
		 public double deltaToPanX(double dx, boolean accel) {
			 return dx*((accel) ? PAN_ACCEL_FACTOR : 1.0);
		 }

		 /**
		  * <p>Compute pan y from delta y, in viewpoint coordinates.</p>
		  *
		  * @param dy delta y, in viewpoint coordinates
		  * @param accel whether to multiply the return value by {@link
		  * #PAN_ACCEL_FACTOR}
		  *
		  * @return the amount to pan y, in viewpoint coordinates
		  **/
		 public double deltaToPanY(double dy, boolean accel) {
			 return dy*((accel) ? PAN_ACCEL_FACTOR : 1.0);
		 }

		 /**
		  * <p>Perform a zoom.</p>
		  *
		  * <p>Zooms about {@link #zoomCenter} by default, about the view center if
		  * ALT is down.</p>
		  *
		  * @param newScale the new view scale
		  * @param modifiers the extended modifiers
		  **/
		 public void doZoom(double newScale, int modifiers) {

			 if ((modifiers & InputEvent.ALT_DOWN_MASK) != 0) {
				 //scale about current view center
				 resetWorldToView(cx, cy, newScale);
				 return;
			 }

			 if (zoomCenter == null)
				 return;

			 //scale about zoomCenter

			 double scale = VisionGUI.this.scale;
			 double ds = newScale - scale;

			 double x = (scale*cx+ds*zoomCenter.x)/newScale;
			 double y = (scale*cy+ds*zoomCenter.y)/newScale;

			 resetWorldToView(x, y, newScale);
		 }

		 /**
		  * <p>Perform a pan in {@link #view}.</p>
		  *
		  * @param dx the x translation, in viewpoint coordinates
		  * @param dy the y translation, in viewpoint coordinates
		  **/
		 public void doPan(double dx, double dy) {
			 resetWorldToView(cx-dx/scale, cy+dy/scale);
		 }

		 /**
		  * <p>The previous point of the current drag.</p>
		  **/
		 protected Point2D.Double prevDragPoint = new Point2D.Double();

		 /**
		  * <p>The current drag point.</p>
		  **/
		 protected Point2D.Double currentDragPoint = new Point2D.Double();

		 /**
		  * <p>The start point of the current drag in world coords.</p>
		  **/
		 protected Point2D.Double startDragPoint = new Point2D.Double();

		 /**
		  * <p>The current zoom center in world coords.</p>
		  **/
		 protected Point2D.Double zoomCenter = new Point2D.Double();

		 /**
		  * <p>Whether a drag is currently ongoing.</p>
		  **/
		 protected boolean dragInProgress = false;
	};

	/**
	 * <p>Drives robot iff both {@link #maxTV} and {@link #maxRV} are greater
	 * than zero.</p>
	 *
	 * <p>See class header doc for details.</p>
	 **/
	protected KeyListener keyListener = new KeyListener() {

		public void keyPressed(KeyEvent e) {
		}

		public void keyReleased(KeyEvent e) {
		}

		public synchronized void keyTyped(KeyEvent e) {
			
			if ((maxTV <= 0.0) || (maxRV <= 0.0))
				return;

			tv = 0.0;
			rv = 0.0;

			switch (e.getKeyChar()) {
			case ' ':
				break;
			case 'i':
				tv = maxTV;				
				break;
			case 'u':
				tv = maxTV;
				rv = maxRV;
				break;
			case 'o':
				tv = maxTV;
				rv = -maxRV;
				break;
			case 'j':
				rv = maxRV;
				break;
			case 'l':
				rv = -maxRV;
				break;
			case ',':
				tv = -maxTV;
				break;
			case 'm':
				tv = -maxTV;
				rv = -maxRV;
				break;
			case '.':
				tv = -maxTV;
				rv = maxRV;
				break;
			case 'q':
				System.exit(-1);
				break;
			}

			System.out.printf("setting velocity: %f %f\n", tv, rv);

			MotionMsg msg= new MotionMsg();
			msg.translationalVelocity = tv;
			msg.rotationalVelocity = rv;
			publisher.publish(msg);
		}
	};

	/**
	 * <p>Extends default impl to set white background.</p>
	 **/
	public JToolTip createToolTip() {
		JToolTip toolTip = super.createToolTip();
		toolTip.setBackground(Color.WHITE);
		return toolTip;
	}

	/**
	 * <p>Handle size changes.</p>
	 **/
	protected ComponentListener componentListener =
		new ComponentAdapter() {

		{
			addComponentListener(this);
		}

		/**
		 * <p>Calls {@link #resetWorldToView}, then {@link #repaint}.</p>
		 **/
		public void componentResized(ComponentEvent e) {
			resetWorldToView(cx, cy, scale);
		}
	};

	/**
	 * <p>See {@link #instanceMain}.</p>
	 **/
	@Override
	public void onStart(Node node) {
		this.node = node;
		
		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
		vidSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			    public void onNewMessage(org.ros.message.sensor_msgs.Image message) {
			    byte[] rgbData = Image.RGB2BGR(message.data,  (int)message.width, (int)message.height);
			    setVisionImage(rgbData, (int)message.width, (int)message.height);
			}
		    }
		    );

		odoSub = node.newSubscriber("/rss/odometryAdjusted", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(
					  new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
					      @Override
						  public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
						  if ( firstUpdate ) {
						      firstUpdate = false;
						      resetWorldToView(message.x, message.y);
						  }
						  setRobotPose(message.x, message.y, message.theta);
					      }
					  }
					  );
		
		publisher = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
	}

	/**
	 * <p>Draw some things to test the graphics capabilities.</p>
	 **/
	protected void testGraphics() {

		try {

			setRobotPose(0.0, 0.0, 0.0);
			setRobotPose(0.0, -1.0, Math.PI/4.0);

			//      for (int i = 0; i < 5000; i++)
			//        setRobotPose(Math.random(), Math.random(),
			//                     (Math.random()-0.5)*2.0*Math.PI);

			byte[] testImage = new byte[256*256*3];

			int index = 0;
			for (int r = 0; r < 256; r++) {
				for (int c = 0; c < 256; c++) {
					byte val = (byte) c;
					testImage[index++] = val;
					testImage[index++] = val;
					testImage[index++] = val;
				}
			}

			setVisionImage(testImage, 256, 256);

			Thread.sleep(1000);

			setVisionImage(testImage, 100, 100);

			Thread.sleep(1000);

			eraseVisionImage();

			testGraphicsHook();

			for (;;) 
				Thread.sleep(1000);

		} catch (InterruptedException e) {
			//ignore
		}
	}

	/**
	 * <p>Hook to append to the end of {@link #testGraphics}.</p>
	 **/
	protected void testGraphicsHook() throws InterruptedException {
	}

	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub
		if(node != null) {
			node.shutdown();
		}
	}

    @Override
	public void onShutdownComplete(Node node) {
    }

    @Override
	public GraphName getDefaultNodeName() {
	return new GraphName("rss/visiongui");
    }

}
