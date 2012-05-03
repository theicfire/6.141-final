package VisualServoSolution;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.ros.message.rosgraph_msgs.Log;

import Controller.Utility;

import com.googlecode.javacv.cpp.opencv_core.CvMat;
import com.googlecode.javacv.cpp.opencv_core.IplImage;


public class BlockMapper  {

	static class Block {
		double x;
		double y;

		int centerPx_x;
		int centerPx_y;

	}
	
//	FakeUtility.Robot robot;

	ArrayList<Block> knownBlocks;
	
	final int hueRedLowerBound = 0; // red
    final int hueRedUpperBound = 25; // red
//	final int hueGreenLowerBound = 55; // green
//    final int hueGreenUpperBound = 62; // green
	final int minBlobArea = 256;
	final int maxBlobArea = 1024;
	
	ImageAnalyzer imgAnal = null;

	public BlockMapper(int width, int height) {
		imgAnal = new ImageAnalyzer(width,height);
		knownBlocks = new ArrayList<Block>();
	}

	ArrayList<Blob> getBlobList(Image srcRgb, org.apache.commons.logging.Log log) {
		// convert Image to format usable by openCV
		IplImage iplRgb = Utility.rgbImgToRgbIplImg(srcRgb,log);

		ArrayList<Blob> blobList = new ArrayList<Blob>();

		// green blobs
		ArrayList<Utility.Pair<ArrayList<Point>,Double>> blobContours =
			imgAnal.detectBlobs(iplRgb,
				hueRedLowerBound,hueRedUpperBound,
				minBlobArea,maxBlobArea);
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
					pair.second,Blob.BlobColor.RED);
			blobList.add(blob);
		}
		
		return blobList;
	}
	
	Point2D.Double applyHomography(CvMat h, double x, double y) {
		double z_ = x*h.get(2,0) + y*h.get(2,1) + h.get(2,2);
		double invz_ = 1/z_;
		double x_ = x*h.get(0,0) + y*h.get(0,1) + h.get(0,2);
		double y_ = x*h.get(1,0) + y*h.get(1,1) + h.get(1,2);
		return new Point2D.Double(x_*invz_,y_*invz_);
	}
	
	// given an image, map any blocks you can see
	// and remove any blocks you no longer see
//	void foo(Image rgb) {
//		FakeUtility.Pose currPose = robot.localizer.getPose();
//
//		ArrayList<Blob> blobList = this.getBlobList(rgb);
//		disappeared = new set;
//		bList = new List<Block>;
//		ppList = new List<Point2D>;
//
//		for (Block block: knownBlocks) {
//			CvMat hFloorToPixel = camera.floorToPixelHomography();
//			opencv_core.
//			int arg0;
//			hFloorToPixel.get(arg0, arg1);//get(arg0);
//			pp_xyz =
//				block.location.x*inv.getCol(0) +
//				block.location.y*inv.getCol(1) +
//				inv.getCol(2);
//			//.apply(block.location);
//			double inv_z = 1.0/pp_xyz;
//			Point2D.Double pp = new Point2D.Double(pp_xyz.x*inv_z,pp_xyz.y*inv_z);//pixel projection of block onto the camera;
//
//			if (pp.x >= 0 && pp.x < cameraImage.width
//					&& pp.y >= 0 && pp.y < cameraImage.height) {
//				Blob matched = null;
//				for (Blob blob: blobList) {
//					if (blob.center is close enough to pp) {
//						updateLocationOf(b,pp);
//						matched = blob;
//					}
//				}
//				if (matched != null) {
//					blobList.remove(matched);
//					associate(matched,block);
//				}
//				else {
//					disappeared.add(b);
//				}
//			}
//		}
//
//		
//		for (Blob blob: blobList) {
//			Pair<Double,Double> angleAndDist =
//				getBlockLocationRelativeToRobot(blob.area,blob.px_x,blob.px_y);
//			//then the block location is...
//			double sumTheta = currPose.theta + angleAndDist.first;
//			newX = Math.cos(sumTheta) + currPose.x;
//			newY = Math.sin(sumTheta) + currPose.y;
//			blobLocation = rightHandRotate(unitX,sumTheta)+ currentPose.xy;
//
//			blob = new Blob(blobLocation,blobCenter)
//			blobList.add(blob);
//		}
//		
		
		
		
//	}
	
//	public class InterpRawVid implements
//			MessageListener<org.ros.message.sensor_msgs.Image> {
//
//		@Override
//		public void onNewMessage(org.ros.message.sensor_msgs.Image src) {
//			// TODO Auto-generated method stub
//
//			Image dest = new Image(src.data, (int) src.width, (int) src.height);
////			log.info("hue" + dest.getPixel(50, 50).getHue());
////			log.info("sat" + dest.getPixel(50, 50).getSaturation());
////			log.info("bri" + dest.getPixel(50, 50).getBrightness());
//			Image otherSrc = new Image(src.data, (int) src.width,
//					(int) src.height);
//			//log.info("bt + " + bt);
//			bt.apply(otherSrc, dest);
//			
//			extractBlocks();
//
////			for (int i = 50; i < 55; i++) {
////				for (int j = 50; j < 55; j++) {
////					dest.setPixel(i, j, new Image.Pixel(255, 0, 0));
////				}
////			}
//
//			targetDetected = bt.targetDetected;
//			centroidX = bt.centroidX;
//			centroidY = bt.centroidY;
//			range = bt.targetRange;
//			angle = (0.0171693 + 0.00987868 * centroidX) - Math.PI / 4;
//
////			log.info("range, angle" + range + ", " + angle);
//
//			org.ros.message.sensor_msgs.Image pubImage = new org.ros.message.sensor_msgs.Image();
//			pubImage.width = IMAGE_WIDTH;
//			pubImage.height = IMAGE_HEIGHT;
//			pubImage.encoding = "rgb8";
//			pubImage.is_bigendian = 0;
//			pubImage.step = IMAGE_WIDTH * 3;
//
//			pubImage.data = dest.toArray();
//			vidPub.publish(pubImage);
//		}
//	}
}
