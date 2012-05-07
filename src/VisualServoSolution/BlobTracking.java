package VisualServoSolution;

import org.apache.commons.logging.Log;

import VisualServoSolution.ConnectedComponents;
import VisualServoSolution.GaussianBlur;
import VisualServoSolution.Image;
import VisualServoSolution.Image.Pixel;

/**
 * BlobTracking performs image processing and tracking for the VisualServo
 * module.  BlobTracking filters raw pixels from an image and classifies blobs,
 * generating a higher-level vision image.
 *
 * @author previous TA's, prentice
 */
public class BlobTracking {

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
	private static int countExceptions = 0;

	/**
	 * <p>Create a BlobTracking object</p>
	 *
	 * @param image width
	 * @param image height
	 */
	public BlobTracking(int width, int height, Log log) {

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
			if (countExceptions == 0) {
				log.info("OH NO; EXCEPTION!!!");
				log.info(e.getStackTrace());
//				log.info("labeling; sizes are " + threshIm.length + " " + connIm.length);
//				log.info("and width/height are " + width + " " + height);
				countExceptions += 1;
			}						
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
	/** //(Solution)
	 * <p>Print average RGB over center 1/4 of image.</p> //(Solution)
	 * //(Solution)
	 * @param source image //(Solution)
	 **/ //(Solution)
	public void averageRGB(Image image){//(Solution)
		double totalRed = 0; //(Solution)
		double totalGreen = 0; //(Solution)
		double totalBlue = 0; //(Solution)
		int numPixels = 0; //(Solution)
		//(Solution)
		for (int y = 0; y < height; y++) { //(Solution)
			for (int x = 0; x < width; x++) { //(Solution)
				Pixel pix = image.getPixel(x, y);   // (Solution)
				// if pixel is in central 1/4 of image, add to total (Solution)
				if (x > 3.0 * width / 8.0 && x < 5.0 * width / 8.0 //(Solution)
						&& y > 3.0 * height / 8.0 && y < 5.0 * height / 8.0) { //(Solution)
					totalRed += pix.getRed(); //(Solution)
					totalGreen += pix.getGreen(); //(Solution)
					totalBlue += pix.getBlue(); //(Solution)
					numPixels++; //(Solution)
				} //(Solution)
			} //(Solution)
		} //(Solution)
		//XXX System.err.println("AverageRGB:  R:" + totalRed/numPixels + " G:" //(Solution)
		//XXX		+ totalGreen/numPixels + " B:" + totalBlue/numPixels); //(Solution)
	} // (Solution)

	/** //(Solution)
	 * <p>Print average HSB over center 1/4 of image.</p> //(Solution)
	 * //(Solution)
	 * @param source image (as float image) //(Solution)
	 **/ //(Solution)
	protected void averageHSB(Image image) { //(Solution)
		float totalHue = 0; //(Solution)
		float totalSat = 0; //(Solution)
		float totalBri = 0; //(Solution)
		int numPixels = 0;  //(Solution)
		//(Solution)
		for (int y = 0; y < height; y++) { //(Solution)
			for (int x = 0; x < width; x++) { //(Solution)
				
				Pixel pix = image.getPixel(0, 0);
//				try {
//					pix = image.getPixel(x, y);  // (Solution)
//				} catch(Exception e) {
//					log.info("exception on " + x + " " + y + 
//							" width, height " + image.getWidth() + " " + image.getHeight());
//				}
				if (x > 3.0 * width / 8.0 && x < 5.0 * width / 8.0 //(Solution)
						&& y > 3.0 * height / 8.0 && y < 5.0 * height / 8.0) { //(Solution)
					totalHue += pix.getHue(); //(Solution)
					totalSat += pix.getSaturation(); //(Solution)
					totalBri += pix.getBrightness(); //(Solution)
					numPixels++; //(Solution)
				} //(Solution)
			} //(Solution)
		} //(Solution)
		//XXX System.err.println("AverageHSB: H:" + totalHue/numPixels + " S:" //(Solution)
		//XXX		+ totalSat/numPixels + " B:" + totalBri/numPixels); //(Solution)
	} //(Solution)
	
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
	public void apply(Image src, Image dest) {
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < 20; j++) {
				dest.setPixel(i,j,new Image.Pixel(255,0,0));
			}
		}

		stepTiming();  // monitors the frame rate

		// Begin Student Code
		
		//averageRGB(src); // (Solution)
		averageHSB(src); // (Solution)

		if (useGaussianBlur) // (Solution)
		    GaussianBlur.apply(src.toArray(), src.toArray(), width, height); // (Solution)	       

		blobPixel(src, blobPixelMask);  //(Solution)
		blobPresent(blobPixelMask, imageConnected, blobMask); //(Solution)
		if (targetDetected) { // (Solution)
			blobFix(); // (Solution)
			computeTranslationVelocityCommand(); // (Solution)
			computeRotationVelocityCommand(); // (Solution)
			//XXX System.err.println("Bearing (Deg): " + (targetBearing*180.0/Math.PI)); // (Solution)
			//XXX System.err.println("Range (M): " + targetRange); // (Solution)
		} else { // (Solution)
			//XXX System.err.println("no target"); // (Solution)
			translationVelocityCommand = 0.0; // (Solution)
			rotationVelocityCommand = 0.0; // (Solution)
		} // (Solution)
		// (Solution)
		//XXX System.err.println("Tracking Velocity: " + // (Solution)
		//XXX		translationVelocityCommand + "m/s, " + // (Solution)
		//XXX		rotationVelocityCommand + "rad/s"); // (Solution)
		// For a start, just copy src to dest. // (Solution)
		if (dest != null) { // (Solution)
			for (int y = 0; y < height; y++) { // (Solution)
				for (int x = 0; x < width; x++) { // (Solution)
					Pixel pix = src.getPixel(x, y); // (Solution)
					dest.setPixel(x, y, pix); // (Solution)
				} // (Solution)
			} // (Solution)
			// (Solution)
//			Histogram.getHistogram(src, dest, true); // (Solution)			
			markBlob(src, dest); // (Solution)
			// (Solution)
		} // (Solution)
		// End Student Code
		
		
	}
	
	
}
