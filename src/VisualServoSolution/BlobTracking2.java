package VisualServoSolution;

import java.awt.Point;
import java.util.ArrayList;

import org.apache.commons.logging.Log;

import com.googlecode.javacpp.Loader;
import com.googlecode.javacpp.Pointer;
import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_imgproc;
import com.googlecode.javacv.cpp.opencv_core.CvContour;
import com.googlecode.javacv.cpp.opencv_core.CvMemStorage;
import com.googlecode.javacv.cpp.opencv_core.CvPoint;
import com.googlecode.javacv.cpp.opencv_core.CvSeq;
import com.googlecode.javacv.cpp.opencv_core.CvSize;
import com.googlecode.javacv.cpp.opencv_core.IplImage;


import Controller.Utility;
import VisualServoSolution.ConnectedComponents;
import VisualServoSolution.Image;
import VisualServoSolution.Image.Pixel;

/**
 * BlobTracking performs image processing and tracking for the VisualServo
 * module.  BlobTracking filters raw pixels from an image and classifies blobs,
 * generating a higher-level vision image.
 *
 * @author previous TA's, prentice
 */
public class BlobTracking2 {

	protected int stepCounter = 0;
	protected double lastStepTime = 0.0;

	public int width;
	public int height;
	public int numChannels = 3;

	protected float histogram[][]; //(Solution)
	// (Solution)
	protected int blobMask[] = null; //(Solution)
	protected int blobPixelMask[] = null; //(Solution)
	protected int imageFiltered[] = null; //(Solution)
	protected int imageConnected[] = null; //(Solution)
	protected float imageHsb[] = null; //(Solution)
	// (Solution)
	public double targetHueLevel; // (Solution)
	public double targetRadius; // (Solution)
	public double hueThreshold; // (Solution)
	public double saturationLevel; // (Solution)
	public double blobSizeThreshold; // (Solution)
	public double desiredFixationDistance; // (Solution)
	public double translationErrorTolerance; // (Solution)
	public double rotationErrorTolerance; // (Solution)
	public boolean useGaussianBlur; // (Solution)
	public double translationVelocityGain; // (Solution)
	public double translationVelocityMax; // (Solution)
	public double rotationVelocityGain; // (Solution)
	public double rotationVelocityMax; // (Solution)
	// (Solution)
	/** //(Solution)
	 * <p>Focal plane distance, experimentally determined, in pixels.</p> //(Solution)
	 **/ //(Solution)
	public double focalPlaneDistance = 107.0; //(Solution)
	// (Solution)
	protected ConnectedComponents connComp = new ConnectedComponents(); //(Solution)
	// (Solution)
	public double translationVelocityCommand = 0.0; // (Solution)
	public double rotationVelocityCommand = 0.0; // (Solution)

	// Variables used for velocity controller that are available to calling
	// process.  Visual results are valid only if targetDetected==true; motor
	// velocities should do something sane in this case.
	public boolean targetDetected = false; // set in blobPresent()
	public double centroidX = 0.0; // set in blobPresent()
	public double centroidY = 0.0; // set in blobPresent()
	public double targetArea = 0.0; // set in blobPresent()
	public double targetRange = 0.0; // set in blobFix()
	public double targetBearing = 0.0; // set in blobFix()
	public Log log;

	BlockMapper blockMapper;
	Blob lastBlob;
	IplImage imgHSV;
	IplImage imgBlobMaskBlue;
	IplImage imgBlobMaskGreen;
	IplImage imgBlobMaskRedLo; // only Lo hue values
	IplImage imgBlobMaskRed; // both Lo and Hi values
	IplImage imgBlobMaskYellow;
	
	CvMemStorage cvStorage;

	// blurring properties and edge detection
	static int LOW_THRESHOLD = 200;
	static int RATIO = 5;
	static int KERNEL_SIZE = 5;
	// blob thresholds
	int MIN_BLOB_AREA = 200;
	int MAX_BLOB_AREA = 1024;


	/**
	 * <p>Create a BlobTracking object</p>
	 *
	 * @param image width
	 * @param image height
	 */
	public BlobTracking2(int width, int height, org.apache.commons.logging.Log log) {

		this.log = log;
		this.width = width;
		this.height = height;

		histogram = new float[width][3]; // (Solution)
		// (Solution)
		// initialize image structures and masks // (Solution)
		blobMask = new int[width * height]; // (Solution)
		blobPixelMask = new int[width * height]; // (Solution)
		imageConnected = new int[width * height]; // (Solution)
		imageHsb = new float[width * height * numChannels]; // (Solution)
		
		CvSize size = new CvSize();
		size.width(width);
		size.height(height);
		imgHSV = opencv_core.cvCreateImage(
				size,opencv_core.IPL_DEPTH_8U,3);
		imgBlobMaskBlue = opencv_core.cvCreateImage(
				size,opencv_core.IPL_DEPTH_8U,1);
		imgBlobMaskGreen = opencv_core.cvCreateImage(
				size,opencv_core.IPL_DEPTH_8U,1);
		imgBlobMaskRedLo = opencv_core.cvCreateImage(
				size,opencv_core.IPL_DEPTH_8U,1);
		imgBlobMaskRed = opencv_core.cvCreateImage(
				size,opencv_core.IPL_DEPTH_8U,1);
		imgBlobMaskYellow = opencv_core.cvCreateImage(
				size,opencv_core.IPL_DEPTH_8U,1);

		cvStorage = opencv_core.cvCreateMemStorage(0);
		
		blockMapper = new BlockMapper(width,height);
	}	
	//(Solution)
	//(Solution)	
	/** // (Solution)
	 * <p>Apply connected components analysis to pick out the largest blob. Then // (Solution)
	 * build stats on this blob.</p> // (Solution)
	 **/ // (Solution)
	protected void blobPresent(int[] threshIm, int[] connIm, int[] blobIm) { // (Solution)
		// (Solution)
		try {
		connComp.doLabel(threshIm, connIm, width, height); // (Solution)
		} catch (Exception e) {
			log.info("OH NO; EXCEPTION!!!");
			log.info(e.getStackTrace());
//			log.info("labeling; sizes are " + threshIm.length + " " + connIm.length);
//			log.info("and width/height are " + width + " " + height);
		}
		// (Solution)
		int colorMax = connComp.getColorMax(); // (Solution)
		int countMax = connComp.getCountMax(); // (Solution)
		//XXX System.out.println("Count max -- num pixels is :  " + countMax);// (Solution)
		//XXX System.out.println("Fraction of num pixels is  " + ((float)countMax) / (width*height));// (Solution)
		// (Solution)
		if (countMax > blobSizeThreshold * height * width) { // (Solution)
			int sx = 0; // (Solution)
			int sy = 0; // (Solution)
			targetArea = countMax; // (Solution)
			int destIndex = 0; // (Solution)
			for (int y = 0; y < height; y++) { // (Solution)
				for (int x = 0; x < width; x++) { // (Solution)
					if (connIm[destIndex] == colorMax) { // (Solution)
						sx += x; // (Solution)
						sy += y; // (Solution)
						blobIm[destIndex++] = 255; // (Solution)
					} else { // (Solution)
						blobIm[destIndex++] = 0; // (Solution)
					} // (Solution)
				} // (Solution)
			} // (Solution)
			centroidX = sx / (double) countMax; // (Solution)
			centroidY = sy / (double) countMax; // (Solution)
			targetDetected = true; // (Solution)
		}  // (Solution)
		else { // (Solution)
			targetDetected = false; // (Solution)
		} // (Solution)
	} // (Solution)
	// (Solution)
	// (Solution)
	/** //(Solution)
	 * <p>Compute range and bearing.</p> //(Solution)
	 * //(Solution)
	 * <p>Note: these are very rough measurments, sufficient for task.</p> //(Solution)
	 * //(Solution)
	 * <p>Using pinhole camera assumption (Horn, Ch2).</p> //(Solution)
	 * //(Solution)
	 * <p>For a point in the world (x,y,z) projected on to image plane IP as //(Solution)
	 * (xp,yp,fp), where fp is the focal length, by simple geometry we know that: //(Solution)
	 * <pre> //(Solution)
	 * xp/fp = x/z, yp/fp=y/z, or fp=xp*z/x //(Solution)
	 * </pre></p> //(Solution)
	 *  //(Solution)
	 * <p>Experimentally, we determine that:<ul> //(Solution)
	 * <li>0.25m = Area of 7200, or radius of 42 pixels</li> //(Solution)
	 * <li>0.5m = Area of  2500,  or radius of 28 pixels</li> //(Solution)
	 * <li>1.0m = Area of  1150,  or radius of 19 pixels</li> //(Solution)
	 * </ul></p> //(Solution)
	 * //(Solution)
	 * <p>We can estimate fp ~= Rpixel*Depth/Rblob, where Rblob ~= .13m , so fp //(Solution)
	 * ~= 107 pixels if we use the 0.5m estimate.</p> //(Solution)
	 * //(Solution)
	 * <p>Now given a area, we can estimate the depth as: Depth = //(Solution)
	 * fp*Rblob/Rpixel.</p> //(Solution)
	 * //(Solution)
	 * <p>We can also estimate the bearing using fp as: //(Solution)
	 * <pre>bearing = atan2(centroidX,fp) //(Solution)
	 * </pre></p> //(Solution)
	 **/ //(Solution)
	private void blobFix() { //(Solution)
		double deltaX = centroidX - width / 2.0; //(Solution)
		targetRange = //(Solution)
			focalPlaneDistance * targetRadius / Math.sqrt(targetArea / Math.PI); //(Solution)
		targetBearing = Math.atan2(deltaX, focalPlaneDistance); //(Solution)
	} //(Solution)
	//(Solution)
	/**  //(Solution)
	 * <p>Compute the translational velocity command using a //(Solution)
	 * P-controller on current translation error.</p> //(Solution)
	 *  //(Solution)
	 * @return translational velocity command //(Solution)
	 */ //(Solution)
	protected void computeTranslationVelocityCommand() { //(Solution)
		double translationError = targetRange - desiredFixationDistance; //(Solution)
		if (Math.abs(translationError) < translationErrorTolerance) //(Solution)
			translationVelocityCommand = 0.0; //(Solution)
		else //(Solution)
			translationVelocityCommand = // (Solution)
				Math.max(-translationVelocityMax,  //(Solution)
						Math.min(translationVelocityMax, //(Solution)
								translationError * translationVelocityGain)); //(Solution)
	} //(Solution)
	//(Solution)
	/** //(Solution)
	 * <p>Compute the rotational velocity command using a P-controller //(Solution)
	 * on the current rotation error, {@link #targetBearing}.<\p>  //(Solution)
	 *  //(Solution)
	 * @return rotation velocity command //(Solution)
	 */ //(Solution)
	protected void computeRotationVelocityCommand() { //(Solution)
		double rotationError = targetBearing; //(Solution)
		if (Math.abs(rotationError) < rotationErrorTolerance) //(Solution)
			rotationVelocityCommand = 0.0; //(Solution)
		else //(Solution)
			rotationVelocityCommand = // (Solution)
				Math.max(-rotationVelocityMax, //(Solution)
						Math.min(rotationVelocityMax, //(Solution)
								-rotationError * rotationVelocityGain)); //(Solution)
	} //(Solution)
	//(Solution)
	/**
	 * <p>Computes frame rate of vision processing</p>
	 */
	private void stepTiming() {
		double currTime = System.currentTimeMillis();
		stepCounter++;
		// if it's been a second, compute frames-per-second
		if (currTime - lastStepTime > 1000.0) {
			double fps = (double) stepCounter * 1000.0
			/ (currTime - lastStepTime);
			//System.err.println("FPS: " + fps);
			stepCounter = 0;
			lastStepTime = currTime;
		}
	}
	// (Solution)

	/** // (Solution)
	 * <p>Highlight the blob in the given image, while setting all // (Solution)
	 * background pixels to grayscale<\p> // (Solution)
	 * // (Solution)
	 * @param source image // (Solution)
	 * @param destination image // (Solution)
	 */ // (Solution)
	protected void markBlob(Image src, Image dest) { // (Solution)
		int maskIndex = 0; //(Solution)
		//(Solution)
		for (int y = 0; y < height; y++) { //(Solution)
			for (int x = 0; x < width; x++) { //(Solution)
				//(Solution)
				Pixel pix = src.getPixel(x, y); //(Solution)
				//(Solution)
				if (targetDetected && blobMask[maskIndex++] > 0) { //(Solution)
					dest.setPixel(x, y, new Pixel(0, 255, 0)); // (Solution)
				} else { //(Solution)
					int av = (pix.getRed() + pix.getGreen() + pix.getBlue()) / 3; //(Solution)
					dest.setPixel(x, y, new Pixel(av, av, av));//(Solution)
				} //(Solution)
			} //(Solution)
		} //(Solution)
	} // (Solution)
	//(Solution)
	
	/** //(Solution)
	 * <p>BallPixel pixel-level classifier. Threshold HSB image //(Solution)
	 * based on hue distance to target hue and saturation level, //(Solution)
	 * and build binary mask.</p> //(Solution)
	 *  //(Solution)
	 * @param source image (float) //(Solution)
	 * @param dest image (int) //(Solution)
	 **/ //(Solution)
	protected void blobPixel(Image src, int[] mask) { //(Solution)
		//(Solution)
		int maskIndex = 0; //(Solution)
		//(Solution)
		// use to accumulate for avg hue and saturation (Solution)
		double avg_h = 0.0; // (Solution)
		double avg_s = 0.0; // (Solution)
		// (Solution)
		for (int y = 0; y < height; y++) { // (Solution)
			for (int x = 0; x < width; x++) { // (Solution)
				Pixel pix = src.getPixel(x, y);  // (Solution)
				// (Solution)
				avg_h += pix.getHue(); // (Solution)
				avg_s += pix.getSaturation(); // (Solution)
				// (Solution)
				double hdist = Math.abs(pix.getHue() - targetHueLevel); // (Solution)
				// handle colorspace wraparound (Solution)
				if (hdist > 0.5) { // (Solution)
					hdist = 1.0 - hdist; // (Solution)
				} // (Solution)
				// (Solution)
				// classify pixel based on saturation level (Solution)
				// and hue distance (Solution)
				if (pix.getSaturation() > saturationLevel && hdist < hueThreshold) { // (Solution)
					mask[maskIndex++] = 255; // (Solution)
				} else { // (Solution)
					mask[maskIndex++] = 0; // (Solution)
				} // (Solution)
			} // (Solution)
		} // (Solution)
		// (Solution)
		// avg_h /= width * height; // (Solution)
		// avg_s /= width * height; // (Solution)
		// System.err.println("Total Avgerage Hue, Sat: "+avg_h+" "+avg_s); // (Solution)
	} // (Solution)
	

	
	
	
	
	/**
	 * <p>Segment out a blob from the src image (if a good candidate exists).</p>
	 *
	 * <p><code>dest</code> is a packed RGB image for a java image drawing
	 * routine. If it's not null, the blob is highlighted.</p>
	 *
	 * @param src the source RGB image, not packed
	 * @param dest the destination RGB image, packed, may be null
	 */
	synchronized
	public void apply(Image src, Image dest) {
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < 20; j++) {
				dest.setPixel(i,j,new Image.Pixel(255,0,0));
			}
		}

		stepTiming();  // monitors the frame rate

		// Begin Student Code

		// convert the image to ipl
		IplImage rgbIpl = Utility.rgbImgToRgbIplImg(src, log);
		int blueLower = 100;
		int blueUpper = 150;
		int blueSatLower = 150; // 160
		int blueValLower = 15; //30
		
		int greenLower = 50;
		int greenUpper = 80;
		int greenSatLower = 160;
		int greenValLower = 40;
		
		int redLoLower = 0;
		int redLoUpper = 10;
		int redLoSatLower = 160;
		int redLoValLower = 75;
		int redHiLower = 175;
		int redHiUpper = 180;
		int redHiSatLower = 160;
		int redHiValLower = 75;
		
		int yellowLower = 22;
		int yellowUpper = 35;
		int yellowSatLower = 170;
		int yellowValLower = 83;
//		if (lastBlob != null) {
//			switch (lastBlob.color) {
//			
//			case BLUE:
//				ArrayList<Blob> blobListBlue = this.getBlobList(
//						imgBlobMaskBlue,Blob.BlobColor.BLUE,
//						MIN_BLOB_AREA, MAX_BLOB_AREA,
//						LOW_THRESHOLD, RATIO, KERNEL_SIZE);
//				if (blobListBlue.size() > 0) {
//					// pick the blob that is closest to the last blob
//					double closeEnough = 300;
//					double closestDistSqr = Double.POSITIVE_INFINITY;
//					for (Blob blob: blobListBlue) {
//						if (blob.color == lastBlob.color) {
//							double deltaX = lastBlob.px_x - blob.px_x;
//							double deltaY = lastBlob.px_y - blob.px_y;
//							double distSqr = deltaX*deltaX + deltaY*deltaY;
//							if (distSqr < closeEnough && distSqr < closestDistSqr) {
////									log.info("distSqr: " + distSqr);
//								pickedBlob = blob;
//								closestDistSqr = distSqr;
//							}
//						}
//					}
//
//					if (pickedBlob == null) {
//						pickedBlob = blobList.get(0);
//					}
//				} else {
//					// no blue blobs, pick new color
//				}
//				
//				
//				
//				
//				
//				break;
//			
//			
//			}
//			
//		}
		
		
		

		ImageAnalyzer.getBlobs(rgbIpl,
				imgHSV,
				blueLower,
				blueUpper,
				blueSatLower,
				blueValLower,
				imgBlobMaskBlue);
		ImageAnalyzer.getBlobs(rgbIpl,
				imgHSV,
				greenLower,
				greenUpper,
				greenSatLower,
				greenValLower,
				imgBlobMaskGreen);
		ImageAnalyzer.getBlobs(rgbIpl,
				imgHSV,
				redLoLower,
				redLoUpper,
				redLoSatLower,
				redLoValLower,
				imgBlobMaskRedLo);
		ImageAnalyzer.getBlobs(rgbIpl,
				imgHSV,
				redHiLower,
				redHiUpper,
				redHiSatLower,
				redHiValLower,
				imgBlobMaskRed);
		ImageAnalyzer.getBlobs(rgbIpl,
				imgHSV,
				yellowLower,
				yellowUpper,
				yellowSatLower,
				yellowValLower,
				imgBlobMaskYellow);
		opencv_core.cvSet(rgbIpl,opencv_core.cvScalar(255,255,0,0),imgBlobMaskBlue);
		opencv_core.cvSet(rgbIpl,opencv_core.cvScalar(255,0,0,0),imgBlobMaskGreen);
		opencv_core.cvOr(imgBlobMaskRedLo,imgBlobMaskRed,imgBlobMaskRed, null);
//		opencv_core.cvSet(rgbIpl,opencv_core.cvScalar(0,255,0,0),imgBlobMaskRedLo);
		opencv_core.cvSet(rgbIpl,opencv_core.cvScalar(0,255,0,0),imgBlobMaskRed);
		opencv_core.cvSet(rgbIpl,opencv_core.cvScalar(0,255,255,0),imgBlobMaskYellow);
//		log.info("~~~~~~~~~~~~~~~~~here");
		Utility.copyFromRgbIplImgToRgbImg(rgbIpl,dest,log);

		
		
		
//		ArrayList<Blob> blobList =
//		this.blockMapper.getBlobList(src,log);
		ArrayList<Blob> blobListBlue = this.getBlobList(
				imgBlobMaskBlue,Blob.BlobColor.BLUE,
				MIN_BLOB_AREA, MAX_BLOB_AREA,
				LOW_THRESHOLD, RATIO, KERNEL_SIZE);
		ArrayList<Blob> blobListGreen = this.getBlobList(
				imgBlobMaskGreen,Blob.BlobColor.GREEN,
				MIN_BLOB_AREA, MAX_BLOB_AREA,
				LOW_THRESHOLD, RATIO, KERNEL_SIZE);
		ArrayList<Blob> blobList = this.getBlobList(
				imgBlobMaskRed,Blob.BlobColor.RED,
				MIN_BLOB_AREA, MAX_BLOB_AREA,
				LOW_THRESHOLD, RATIO, KERNEL_SIZE);
		ArrayList<Blob> blobListYellow = this.getBlobList(
				imgBlobMaskYellow,Blob.BlobColor.YELLOW,
				MIN_BLOB_AREA, MAX_BLOB_AREA,
				LOW_THRESHOLD, RATIO, KERNEL_SIZE);
		Blob pickedBlob = null;

//		log.info("numBlobs: " + blobList.size());
		
		if (blobList.size() > 0) {
			if (lastBlob != null) {
				// pick the blob that is closest to the last blob
				double closeEnough = 300;
				double closestDistSqr = Double.POSITIVE_INFINITY;
				for (Blob blob: blobList) {
					if (blob.color == lastBlob.color) {
						double deltaX = lastBlob.px_x - blob.px_x;
						double deltaY = lastBlob.px_y - blob.px_y;
						double distSqr = deltaX*deltaX + deltaY*deltaY;
						if (distSqr < closeEnough && distSqr < closestDistSqr) {
//							log.info("distSqr: " + distSqr);
							pickedBlob = blob;
							closestDistSqr = distSqr;
						}
					}
				}
			}

			if (pickedBlob == null) {
				pickedBlob = blobList.get(0);
			}
		}

		lastBlob = pickedBlob;

		if (pickedBlob != null) {
//			log.info("blob: "
//					+ pickedBlob.px_x + " "
//					+ pickedBlob.px_y + " "
//					+ pickedBlob.area + " ");

			// draw a marker for the selected block
			int halfSquareWidth = 2;
			int yStart = Math.max(0, pickedBlob.px_y-halfSquareWidth);
			int yEnd = Math.min(dest.getHeight(),pickedBlob.px_y+halfSquareWidth);
			int xStart = Math.max(0, pickedBlob.px_x-halfSquareWidth);
			int xEnd = Math.min(dest.getWidth(),pickedBlob.px_x+halfSquareWidth);
			for (int i = yStart; i < yEnd; i++) {
				for (int j = xStart; j < xEnd; j++) {
					dest.setPixel(j,i,new Pixel(255,0,255));
				}
			}	
			
			centroidX = pickedBlob.px_x;
			centroidY = pickedBlob.px_y;
			targetArea = pickedBlob.area;
			
			blobFix();
			computeTranslationVelocityCommand();
			computeRotationVelocityCommand();
		} else {
			translationVelocityCommand = 0.0;
			rotationVelocityCommand = 0.0;
		}
		
		
		//averageRGB(src); // (Solution)
//		averageHSB(src); // (Solution)
//		if (useGaussianBlur) // (Solution)
//		    GaussianBlur.apply(src.toArray(), src.toArray(), width, height); // (Solution)	       
//		blobPixel(src, blobPixelMask);  //(Solution)
//		blobPresent(blobPixelMask, imageConnected, blobMask); //(Solution)
//		if (targetDetected) { // (Solution)
//			blobFix(); // (Solution)
//			computeTranslationVelocityCommand(); // (Solution)
//			computeRotationVelocityCommand(); // (Solution)
//			//XXX System.err.println("Bearing (Deg): " + (targetBearing*180.0/Math.PI)); // (Solution)
//			//XXX System.err.println("Range (M): " + targetRange); // (Solution)
//		} else { // (Solution)
//			//XXX System.err.println("no target"); // (Solution)
//			translationVelocityCommand = 0.0; // (Solution)
//			rotationVelocityCommand = 0.0; // (Solution)
//		} // (Solution)
		
		
		// (Solution)
		//XXX System.err.println("Tracking Velocity: " + // (Solution)
		//XXX		translationVelocityCommand + "m/s, " + // (Solution)
		//XXX		rotationVelocityCommand + "rad/s"); // (Solution)
		// For a start, just copy src to dest. // (Solution)
//		if (dest != null) {
//			for (int y = 0; y < height; y++) {
//				for (int x = 0; x < width; x++) {
//					Pixel pix = src.getPixel(x, y);
//					dest.setPixel(x, y, pix);
//				}
//			}
////			Histogram.getHistogram(src, dest, true); // (Solution)			
////			markBlob(src, dest); // (Solution)
//		} // (Solution)
		// End Student Code
		
		
	}
	
	ArrayList<Blob> getBlobList(IplImage imgMask,
			Blob.BlobColor blobColor,
			int minBlobArea, int maxBlobArea,
			int lowThreshold, int ratio,
			int kernel_size) {
		ArrayList<Blob> blobList =
				new ArrayList<Blob>();

		ArrayList<Utility.Pair<ArrayList<Point>,Double>>
		blobContours =
			discretizeBlobs(imgMask, minBlobArea,
					maxBlobArea, lowThreshold,
					ratio, kernel_size);
		
		for (Utility.Pair<ArrayList<Point>, Double> pair: blobContours) {
			int centerX = 0;
			int centerY = 0;
			ArrayList<Point> ctr = pair.first;
			for (Point p: ctr) {
				centerX += p.x;
				centerY += p.y;
			}
			centerX /= ctr.size();
			centerY /= ctr.size();
			Blob blob = new Blob(centerX,centerY,
					pair.second,blobColor);
			blobList.add(blob);
		}
		
		return blobList;
	}
	
	ArrayList<Utility.Pair<
	ArrayList<Point>, Double>> discretizeBlobs(
			IplImage imgMask,
			int minBlobArea, int maxBlobArea,
			int lowThreshold, int ratio,
			int kernel_size) {
        opencv_imgproc.cvCanny(imgMask, imgMask,
        		lowThreshold,lowThreshold*ratio,
        		kernel_size);
//		opencv_imgproc.cvDilate(imgMask,
//				imgMask, null, 1);

		// find contours
		CvSeq first_contour = new CvSeq();
		CvSeq contour = first_contour;
		int header_size = Loader.sizeof(CvContour.class);
		opencv_imgproc.cvFindContours(imgMask,
			cvStorage,first_contour,header_size,
//			opencv_imgproc.CV_RETR_LIST,
			opencv_imgproc.CV_RETR_TREE,
			opencv_imgproc.CV_CHAIN_APPROX_SIMPLE);

		ArrayList<Utility.Pair<ArrayList<Point>,Double>> shapes =
			new ArrayList<Utility.Pair<ArrayList<Point>,Double>>();
		while (contour != null && !contour.isNull()) {
			int numPoints = contour.total();
			if (numPoints >= 6) {
				double epsilon = 0.03;
				epsilon *= opencv_imgproc.cvContourPerimeter(contour);
				CvSeq seqPoints2 = contour;
				CvSeq seqPoints = opencv_imgproc.cvApproxPoly(seqPoints2,
						Loader.sizeof(CvContour.class),cvStorage,
						opencv_imgproc.CV_POLY_APPROX_DP, epsilon,
						1);
				int numSeqPts = seqPoints.total();
				if (numSeqPts >= 4) {
					double area = opencv_imgproc.cvContourArea(
							seqPoints, opencv_core.CV_WHOLE_SEQ, 0);
					if (area >= minBlobArea && area <= maxBlobArea) {
						ArrayList<Point> points = new ArrayList<Point>();
						for (int i = 0; i < numSeqPts; ++i) {
							Pointer ptrPoint = opencv_core.cvGetSeqElem(seqPoints,i);
							CvPoint point = new CvPoint(ptrPoint);
			                points.add(new Point(point.x(),point.y()));
						}
						Utility.Pair<ArrayList<Point>,Double> shapeAndAreaPair =
							new Utility.Pair<ArrayList<Point>,Double>(points, area);
						shapes.add(shapeAndAreaPair);
					}
				}
			}
			
			contour = contour.h_next();
		}

		opencv_core.cvClearMemStorage(cvStorage);
		return shapes;
	}
	
}
