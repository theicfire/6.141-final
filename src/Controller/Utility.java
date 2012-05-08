package Controller;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.nio.ByteBuffer;
import java.util.ArrayList;

import org.apache.commons.logging.Log;
import org.ros.message.rss_msgs.OdometryMsg;

import Challenge.GrandChallengeMap;
import Controller.Utility.ConfidencePose;
import VisualServoSolution.HomographySrcDstPoint;
import VisualServoSolution.Image;
import VisualServoSolution.Image.Pixel;

import com.googlecode.javacv.cpp.opencv_calib3d;
import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.CvMat;
import com.googlecode.javacv.cpp.opencv_core.IplImage;

public class Utility {
	public static double getMagnitude(Point2D.Double a, Point2D.Double b) {
		return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
	}

	public static double getAngle(Point2D.Double a, Point2D.Double b) {
		return Math.atan2(b.y - a.y, b.x - a.x);
	}

	public static void sleepFor20ms() {
		try {
			Thread.sleep(20);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void sleepFor250ms() {
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void sleepFor5Seconds() {
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public class ConfidencePose extends Pose {
		private double _confidence;
		
		public ConfidencePose(double x, double y, double theta, double confidence) {
			super(x, y, theta);
			_confidence = confidence;
		}
		
		public double getConfidence() {
			return _confidence;
		}

		public void setConfidence(double confidence) {
			_confidence = confidence;
		}

		public ConfidencePose add(Pose other) {
			return (new Utility()).new ConfidencePose(this.getX() + other.getX(), 
					this.getY() + other.getY(),
					this.getTheta() + other.getTheta(),
					this.getConfidence());			
		}
	}
	
	public class Pose {
		public double _x, _y, _theta;

		public Pose(double x, double y, double theta) {
			setX(x);
			setY(y);
			setTheta(theta);
		}

		public Pose(Double point, double theta) {
			setX(point.x);
			setY(point.y);
			setTheta(theta);
		}

		public double getY() {
			return _y;
		}

		public void setY(double _y) {
			this._y = _y;
		}

		public double getTheta() {
			return _theta;
		}

		public void setTheta(double _theta) {
			this._theta = _theta;
		}

		public double getX() {
			return _x;
		}

		public void setX(double _x) {
			this._x = _x;
		}

		public Double getPoint() {
			return new Double(this._x, this._y);
		}

		public OdometryMsg getOdomMsg() {
			OdometryMsg om = new OdometryMsg();
			om.x = getX();
			om.y = getY();
			om.theta = getTheta();
			return om;
		}

		@Override
		public String toString() {
			return "Pose [_x=" + _x + ", _y=" + _y + ", _theta=" + _theta + "]";
		}

	}

	public static double inRangeNegPiToPi(double radians) {
		if (radians > Math.PI) {
			do {
				radians -= 2 * Math.PI;
			} while (radians > Math.PI);
		} else if (radians < -Math.PI) {
			do {
				radians += 2 * Math.PI;
			} while (radians < -Math.PI);
		}
		return radians;
	}

	public static class Pair<E1, E2> {
		public final E1 first;
		public final E2 second;

		public Pair(E1 first, E2 second) {
			this.first = first;
			this.second = second;
		}
	}

	public static Image rgbIplImgToRgbImg(IplImage rgbIpl,
			org.apache.commons.logging.Log log) {
		ByteBuffer iplBytePointer = rgbIpl.getByteBuffer();
		return new Image(iplBytePointer.array(), rgbIpl.width(),
				rgbIpl.height(), rgbIpl.widthStep());
	}

	public static org.ros.message.sensor_msgs.Image
	rgbIplImgToRgbImgMsg(
			IplImage rgbIpl) {

		int width = rgbIpl.width();
		int height = rgbIpl.height();
		org.ros.message.sensor_msgs.Image dest =
				new org.ros.message.sensor_msgs.Image();
		dest.width = width;
		dest.height = height;
		dest.encoding = "rgb8";
		dest.is_bigendian = 0;
		dest.step = width * 3;

		dest.data = new byte[width*height*3];

		// widthStep is the number of bytes per line in src
		ByteBuffer byteBuff = rgbIpl.getByteBuffer();
	
		int widthStep = rgbIpl.widthStep();
		int i = 0;
		int r, g, b;
		for (int y = 0; y < height; y++) {
			int yTimesWStep = y * widthStep;
			for (int x = 0; x < width; x++) {
				int threeX = 3 * x;
				// log.info("" + yTimesWStep + " " + threeX + " ");
				r = byteBuff.get(yTimesWStep + threeX + 0) & 0xff;
				g = byteBuff.get(yTimesWStep + threeX + 1) & 0xff;
				b = byteBuff.get(yTimesWStep + threeX + 2) & 0xff;				
				dest.data[i++] = (byte) r;
				dest.data[i++] = (byte) g;
				dest.data[i++] = (byte) b;
			}
		}
		return dest;
	}

	public static org.ros.message.sensor_msgs.Image
	monoIplImgToRgbImgMsg(
			IplImage monoIpl, Log log) {

		int width = monoIpl.width();
		int height = monoIpl.height();
		org.ros.message.sensor_msgs.Image dest =
				new org.ros.message.sensor_msgs.Image();
		dest.width = width;
		dest.height = height;
		dest.encoding = "rgb8";
		dest.is_bigendian = 0;
		dest.step = width * 3;

		dest.data = new byte[width*height*3];

		// widthStep is the number of bytes per line in src
		ByteBuffer byteBuff = monoIpl.getByteBuffer();
	
		int widthStep = monoIpl.widthStep();
		int i = 0;
		for (int y = 0; y < height; y++) {
			int yTimesWStep = y * widthStep;
			for (int x = 0; x < width; x++) {
//				int threeX = 3 * x;
				// log.info("" + yTimesWStep + " " + threeX + " ");
				byte val = (byte) (byteBuff.get(yTimesWStep + x) & 0xff);
				dest.data[i++] = val;
				dest.data[i++] = val;
				dest.data[i++] = val;
			}
		}
		return dest;
	}

	void copyFromRgbImgMsgToRgbIplImg(
			org.ros.message.sensor_msgs.Image src,
			IplImage dest) {

//		final int numChannels = 3;
//		IplImage rgbIpl = IplImage.create((int) rgbImg.width,
//				(int) rgbImg.height, opencv_core.IPL_DEPTH_8U, numChannels);
		ByteBuffer iplBytePointer = dest.getByteBuffer();
		int widthStep = dest.widthStep();

		int srcIndex = 0;
		byte[] srcArray = src.data;

		for (int y = 0; y < src.height; ++y) {
			for (int x = 0; x < src.width; ++x) {
				byte r = (byte) (srcArray[srcIndex++] & 0xff);
				byte g = (byte) (srcArray[srcIndex++] & 0xff);
				byte b = (byte) (srcArray[srcIndex++] & 0xff);
				iplBytePointer.put(y * widthStep + x * 3 + 0, b);
				iplBytePointer.put(y * widthStep + x * 3 + 1, g);
				iplBytePointer.put(y * widthStep + x * 3 + 2, r);
			}
		}
	}
	
	public static IplImage rgbImgMsgToRgbIplImg(
			org.ros.message.sensor_msgs.Image rgbImg,
			org.apache.commons.logging.Log log) {
		final int numChannels = 3;
		IplImage rgbIpl = IplImage.create((int) rgbImg.width,
				(int) rgbImg.height, opencv_core.IPL_DEPTH_8U, numChannels);
		ByteBuffer iplBytePointer = rgbIpl.getByteBuffer();
		int widthStep = rgbIpl.widthStep();

		int srcIndex = 0;
		byte[] src = rgbImg.data;

		for (int y = 0; y < rgbImg.height; ++y) {
			for (int x = 0; x < rgbImg.width; ++x) {
				byte r = (byte) (src[srcIndex++] & 0xff);
				byte g = (byte) (src[srcIndex++] & 0xff);
				byte b = (byte) (src[srcIndex++] & 0xff);
				iplBytePointer.put(y * widthStep + x * 3 + 0, b);
				iplBytePointer.put(y * widthStep + x * 3 + 1, g);
				iplBytePointer.put(y * widthStep + x * 3 + 2, r);
			}
		}
		return rgbIpl;
	}

	public static IplImage rgbImgToRgbIplImg(VisualServoSolution.Image rgbImg,
			org.apache.commons.logging.Log log) {
		final int numChannels = 3;
		IplImage rgbIpl = IplImage.create(rgbImg.getWidth(),
				rgbImg.getHeight(), opencv_core.IPL_DEPTH_8U, numChannels);
		ByteBuffer iplBytePointer = rgbIpl.getByteBuffer();
		int width = rgbImg.getWidth();
		int height = rgbImg.getHeight();
		int widthStep = rgbIpl.widthStep();
		int i = 0;
		for (int y = 0; y < height; ++y) {
			for (int x = 0; x < width; ++x) {
				Image.Pixel p = rgbImg.pixels.get(i);
				iplBytePointer.put(y * widthStep + x * 3 + 0,
						(byte) p.getBlue());
				iplBytePointer.put(y * widthStep + x * 3 + 1,
						(byte) p.getGreen());
				iplBytePointer
						.put(y * widthStep + x * 3 + 2, (byte) p.getRed());
				++i;
			}
		}
		return rgbIpl;
	}

	public static void copyFromRgbIplImgToRgbImg(IplImage rgbIpl, Image dest,
			Log log) {
		// widthStep is the number of bytes per line in src
		int r, g, b;
		ByteBuffer byteBuff = rgbIpl.getByteBuffer();
		int width = rgbIpl.width();
		int height = rgbIpl.height();
		int widthStep = rgbIpl.widthStep();
		// log.info("" + width + " " + height + " " + widthStep + " ");
		for (int y = 0; y < height; y++) {
			int yTimesWStep = y * widthStep;
			for (int x = 0; x < width; x++) {
				int threeX = 3 * x;
				// log.info("" + yTimesWStep + " " + threeX + " ");
				r = byteBuff.get(yTimesWStep + threeX + 0) & 0xff;
				g = byteBuff.get(yTimesWStep + threeX + 1) & 0xff;
				b = byteBuff.get(yTimesWStep + threeX + 2) & 0xff;
				dest.setPixel(x, y, new Pixel(r, g, b));
			}
		}
	}

	public static CvMat computeHomography(ArrayList<HomographySrcDstPoint> correspondences) {
		CvMat matSrc = CvMat.create(correspondences.size(),2);
		CvMat matDst = CvMat.create(correspondences.size(),2);
		
		int numCorresp = correspondences.size();
		for(int i = 0; i < numCorresp; ++i) {
			//Add this point to matSrc and matDst
			HomographySrcDstPoint p = correspondences.get(i);
			matSrc.put(i,0,p.src_x);
			matSrc.put(i,1,p.src_y);
			matDst.put(i,0,p.dst_x);
			matDst.put(i,1,p.dst_y);
		}
		
		double ransacReprojThreshold = 3;
		CvMat mask = null;
		CvMat matH2 = opencv_core.cvCreateMat(3,3,opencv_core.CV_32FC1);
		int result2 = opencv_calib3d.cvFindHomography(matSrc,matDst,matH2,
			opencv_calib3d.CV_RANSAC, ransacReprojThreshold, mask);

		return matH2;
    }
	
	public static GrandChallengeMap getChallengeMap() {
		GrandChallengeMap map = new GrandChallengeMap();
		try {
			String mapfilename = "/home/rss-student/RSS-I-group/Challenge/src/construction_map_2012.txt";
			map = GrandChallengeMap.parseFile(mapfilename);
		} catch (Exception e) {
			throw new RuntimeException(
					"DIE DIE DIE DIE DIE DIE couldn't load map");
		}
		return map;
	}


	// standard position means that the vector originates at
	// the origin
	// returns radian angle between v1 and v2 by
	// sweeping v1 into v2 according to righthand rule
	// result is in range [0,2*pi]
	public static double signedAngleBetweenVectorsInStandardPosition(
			Point2D.Double v1,
			Point2D.Double v2) {
		double a = Math.atan2(v1.y,v1.x);
		double b = Math.atan2(v2.y,v2.x);
		double diff = b - a;
		if (a < b) {
			return diff;
		} else {
			return 2*Math.PI + diff;
		}
	}

	
	// assumes that shapes are specified
	// in counter clockwise order
	public static ArrayList<Point2D.Double> calcOutwardNormals(
			ArrayList<Point2D.Double> shape) {

		int numVertices = shape.size();
		ArrayList<Point2D.Double> normals = new ArrayList<Point2D.Double>(
				numVertices);
		Point2D.Double v1 = new Point2D.Double();
		Point2D.Double v2 = new Point2D.Double();
		Point2D.Double p0 = shape.get(0);
		Point2D.Double p1 = shape.get(1);
		for (int i = 0; i < numVertices; ++i) {
			Point2D.Double p2 = shape.get((i + 2) % numVertices);
			v1.x = p0.x - p1.x;
			v1.y = p0.y - p1.y;
			v2.x = p2.x - p1.x;
			v2.y = p2.y - p1.y;

			double angle = Utility.signedAngleBetweenVectorsInStandardPosition(
					v1, v2);

			// dx = x2-x1 and dy=y2-y1,
			// then the normals are (-dy, dx) and (dy, -dx)
			Point2D.Double n = new Point2D.Double(v1.y, -v1.x);
			double dot = n.x * v2.x + n.y * v2.y;
			if (angle < Math.PI) {
				if (dot < 0) {
					n.y = -n.y;
					n.x = -n.x;
				}
			} else {
				if (dot > 0) {
					n.y = -n.y;
					n.x = -n.x;
				}
			}

			double invLen = 1.0 / Math.sqrt(n.x * n.x + n.y * n.y);
			n.x *= invLen;
			n.y *= invLen;

			normals.add(n);

			p0 = p1;
			p1 = p2;
		}

		return normals;
	}
	
}
