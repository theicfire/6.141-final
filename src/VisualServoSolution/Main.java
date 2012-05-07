//import java.awt.Container;
//import java.awt.Point;
//import java.awt.geom.Point2D;
//import java.awt.geom.Point2D.Double;
//import java.nio.ByteBuffer;
//import java.util.ArrayList;
//import java.util.Random;
//
//import com.googlecode.javacpp.BytePointer;
//import com.googlecode.javacpp.Loader;
//import com.googlecode.javacpp.Pointer;
//import com.googlecode.javacv.CanvasFrame;
//import com.googlecode.javacv.cpp.opencv_core;
//import com.googlecode.javacv.cpp.opencv_highgui;
//import com.googlecode.javacv.cpp.opencv_imgproc;
//import com.googlecode.javacv.cpp.opencv_core.CvArr;
//import com.googlecode.javacv.cpp.opencv_core.CvContour;
//import com.googlecode.javacv.cpp.opencv_core.CvMat;
//import com.googlecode.javacv.cpp.opencv_core.CvMemStorage;
//import com.googlecode.javacv.cpp.opencv_core.CvPoint;
//import com.googlecode.javacv.cpp.opencv_core.CvSeq;
//import com.googlecode.javacv.cpp.opencv_core.CvSize;
//import com.googlecode.javacv.cpp.opencv_core.IplImage;
//import com.googlecode.javacv.cpp.opencv_highgui.CvTrackbarCallback;
//
//public class Main {
//
//	static IplImage srcImg; // 3 channels
//	static IplImage imgHsv; // 3 channels
//	static IplImage imgH; // 3 channels
//	static IplImage imgS; // 3 channels
//	static IplImage imgV; // 3 channels
////	static IplImage imgHsvBlurred; // 3 channels
//	static IplImage imgThreshold; // 1 channel
//	static IplImage imgDetectedEdges; // 1 channel
//	static IplImage imgDilated; // 1 channel
//	static IplImage imgContours; // 1 channel
//	static IplImage imgContoursOverlayed; // 3 channels
//	
//	
////	static IplImage dst;
////	static CvMat src_gray;
//
//	static CanvasFrame canvas0;// = new CanvasFrame("Canvas0");
//	static CanvasFrame canvas1;// = new CanvasFrame("Canvas2");
//	static CanvasFrame canvas2;// = new CanvasFrame("Original Image");
//	static CanvasFrame canvas3;// = new CanvasFrame("Canvas3");
//	static CanvasFrame canvas4;// = new CanvasFrame("Canvas4: HSV");
//	static CanvasFrame canvas5;
//	static CanvasFrame canvas6;
//	static CanvasFrame canvas7;
//	static CanvasFrame canvas8;
//	static CanvasFrame canvas9;
//	
//	  int edgeThresh = 1;
//	  static int lowThreshold = 0;
////	  int max_lowThreshold = 100;
//	  static int ratio = 5;
//	  static int kernel_size = 5;
//
//	
//	static String window_name = "Edge Map";
//	
//	class Color {
//		int b = 0;
//		int g = 0;
//		int r = 0;
//	}
//
//	static Random rand = new Random();
//
//	static Color colors[] = new Color[256*3];
//	static Main init = new Main(true);
//	Main (boolean extra) {
//		for (int i = 0; i < colors.length; ++i) {
//			colors[i] = new Color();
//		}
//		
//		for (int i = 0; i < 256; ++i) {
//			colors[(i+0)%colors.length].b = 255-i;
//			colors[(i+256)%colors.length].g = 255-i;
//			colors[(i+512)%colors.length].r = 255-i;
//		}
//		for (int i = 256; i < 512; ++i) {
//			colors[(i+0)%colors.length].b = i-256;
//			colors[(i+256)%colors.length].g = i-256;
//			colors[(i+512)%colors.length].r = i-256;
//		}
//	}
//
//	
//	IplImage upperBoundHSV;
//	IplImage lowerBoundHSV;
////	int upperBoundH = 256;
////	int lowerBoundH = ;
////	int upperBoundS = ;
////	int lowerBoundS = ;
////	int upperBoundV = ;
////	int lowerBoundV = 128;
////	void ffffooo() {
////		src_gray = CvMat.create(srcImg.height(),srcImg.width(),opencv_core.CV_8U);
////	}
//
//	public static void updateCanvas() {
//		// Reduce noise with a kernel 3x3
//
//
////		opencv_imgproc.cvCvtColor(const CvArr* src, CvArr* dst, int code);
////		opencv_imgproc.cvCvtColor(srcImg,dst,opencv_imgproc.CV_BGR2HSV);
////		IplImage saturation = IplImage.create(srcImg.width(),srcImg.height(),srcImg.depth(),1);// = CvMat.create(src.height(),src.width(),opencv_core.CV_8U);
////		opencv_core.cvSplit(dst, saturation, null, null, null);
////		opencv_imgproc.GaussianBlur(saturation,saturation,
////				new CvSize(3,3), 0, 0, opencv_imgproc.BORDER_DEFAULT);
////		opencv_imgproc.cvCanny(saturation,saturation,lowThreshold,lowThreshold*ratio,kernel_size);
//
//		
////        System.out.println(opencv_core.cvGetSize(srcImg));
////		opencv_imgproc.cvCvtColor(srcImg,imgHsv,opencv_imgproc.CV_BGR2HSV);
////		opencv_imgproc.cvSmooth(imgHSV,imgHSV,opencv_imgproc.CV_GAUSSIAN,3);
////		opencv_imgproc.GaussianBlur(imgHSV,imgHSV,new CvSize(3,3),0,0,opencv_imgproc.BORDER_DEFAULT);
//        // 8-bit 1- color = monochrome
//        // cvScalar : ( H , S , V, A)
// 
//		
////		opencv_core.CvScalar lowerBound = new opencv_core.CvScalar(55);
////		opencv_core.CvScalar upperBound = new opencv_core.CvScalar(62);
////		IplImage saturation2 = IplImage.create(srcImg.width(),srcImg.height(),srcImg.depth(),1);
////		opencv_core.cvInRangeS(saturation,lowerBound,upperBound,saturation2);
//		
////		canvas4.showImage(imgThreshold);
//		
////		opencv_core.cvReleaseImage(saturation);
////		opencv_core.cvReleaseImage(saturation2);
//
//		
//		
////		cvInRange(const CvArr* src, const CvArr* lower, const CvArr* upper, CvArr* dst);
////		destination array, must have 8u or 8s type
//		
////		opencv_imgproc.cvCvtColor(dst,src_gray,opencv_imgproc.CV_BGR2GRAY);
//		
////		opencv_imgproc.GaussianBlur(src_gray, imgDetectedEdges, new CvSize(3,3), 0, 0, opencv_imgproc.BORDER_DEFAULT);
////		opencv_imgproc.cvEqualizeHist(src_gray,detected_edges);
//
//		// Canny detector
//		computeImgDetectedEdges();
//		computeImgDilated();
//	//opencv_imgproc.cvCanny(src_gray,detected_edges,
//	//		lowThreshold, lowThreshold*ratio, kernel_size);
////		opencv_imgproc.cvCanny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);
////		opencv_imgproc.cvCanny(imgThreshold,
////				imgDetectedEdges, lowThreshold,
////				lowThreshold*ratio, kernel_size);
//
////		cvDilate(binary, bg, null /* 3x3 square */ , 6 /* iterations */);
//
////		opencv_imgproc.cvDilate(imgDetectedEdges,
////				imgDetectedEdges, null, 1);
////		int g_thresh = 100;
////		opencv_imgproc.cvThreshold( detected_edges, detected_edges, g_thresh, 255, opencv_imgproc.CV_THRESH_BINARY );
//
//		
////		canvas3.showImage(imgDetectedEdges);
////		opencv_imgproc.GaussianBlur(detected_edges, detected_edges, new CvSize(11,11), 0, 0, opencv_imgproc.BORDER_DEFAULT);
////		opencv_imgproc.cvCanny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);
//		
//		// Using Canny's output as a mask, we display our result
////		dst = Scalar::all(0);
////		src.copyFrom();// dst, detected_edges);
//
//
////		imgContours.copyFrom(imgDilated.getBufferedImage());
//		
////		opencv_core.cvCopy(imgDilated,imgContours);
//		opencv_core.cvCopy(imgDetectedEdges,imgContours);
//		
//		CvMemStorage cvStorage = null;
//		if (cvStorage == null) {
//			cvStorage = opencv_core.cvCreateMemStorage(0);
//		}
//		CvSeq first_contour = new CvSeq();
//		CvSeq contour = first_contour;
//		int header_size = Loader.sizeof(CvContour.class);
//		int result;
//		result = opencv_imgproc.cvFindContours(
//				imgContours,cvStorage,first_contour,header_size,
////			src_gray,cvStorage,first_contour,header_size,
//			opencv_imgproc.CV_RETR_LIST,
////			opencv_imgproc.CV_RETR_TREE,
//			opencv_imgproc.CV_CHAIN_APPROX_SIMPLE);
//		canvas5.showImage(imgContours);
//		
//		// contours overlayed
//		ArrayList<ArrayList<Point>> contourList =
//			new ArrayList<ArrayList<Point>>();
//		while (contour != null && !contour.isNull()) {
//			ArrayList<Point> points = new ArrayList<Point>();
//			for (int i = 0; i < contour.total(); ++i) {
//				Pointer ptrPoint = opencv_core.cvGetSeqElem(contour, i);
//                CvPoint point = new CvPoint(ptrPoint);
////                System.out.print(point.x() + " ");
//                points.add(new Point(point.x(),point.y()));
//			}
////            System.out.println("next");
//            contourList.add(points);
//			contour = contour.h_next();
//		}
//		opencv_core.cvCopy(srcImg,imgContoursOverlayed);
//		ArrayList<Point> bestContour = VisualLocalization.findBestContour(contourList,
//				60, .75);
//		ArrayList<Point2D.Double> interpolated = VisualLocalization.
//			getInterpolatedPointsFromContour(bestContour,1);
//		//		CvMat
//		CvMat homo = CvMat.create(3,3);
//		for (int i = 0; i < 3; ++i) {
//			for (int j = 0; j < 3; ++j) {
//				if (i != j) {
//					homo.put(i,j,0.0);
//				} else {
//					homo.put(i,j,1.0);
//				}
//			}
//		}
//		CvMat res = VisualLocalization.getFloorPointsLocalSpace(
//				interpolated,homo);
//		ArrayList<ArrayList<Point2D.Double>> singleContour =
//			new ArrayList<ArrayList<Point2D.Double>>();
//		singleContour.add(interpolated);
////		drawPointsOnImage(imgContoursOverlayed,contourList);
//		drawPoint2DsOnImage(imgContoursOverlayed,singleContour);
//		canvas9.showImage(imgContoursOverlayed);
//		// end: contours overlayed
//		
////		canvas.showImage(detected_edges);
//		ArrayList<ArrayList<Point>> shapes = new ArrayList<ArrayList<Point>>();
//		int count = 0;
//		while (contour != null && !contour.isNull()) {
//			int numPoints = contour.total();
//			if (true || numPoints >= 6) {
//				double epsilon = 0.03;
//				epsilon *= opencv_imgproc.cvContourPerimeter(contour);
//				CvSeq seqPoints2 = contour;
////					opencv_imgproc.cvApproxPoly(contour,
////						Loader.sizeof(CvContour.class),cvStorage,
////						opencv_imgproc.CV_POLY_APPROX_DP, epsilon,
////						1);
////				epsilon = .03*opencv_imgproc.cvContourPerimeter(contour);
//				CvSeq seqPoints = opencv_imgproc.cvApproxPoly(seqPoints2,
//						Loader.sizeof(CvContour.class),cvStorage,
//						opencv_imgproc.CV_POLY_APPROX_DP, epsilon,
//						0);//1);
//				int numSeqPts = seqPoints.total();
//				if (true || numSeqPts >= 6) {
//					ArrayList<Point> points = new ArrayList<Point>();
//					
////					for (int i = 0; i < contour.total(); ++i) {
////						Pointer ptrPoint = opencv_core.cvGetSeqElem(contour, i);
////		                CvPoint point = new CvPoint(ptrPoint);
////		                points.add(new Point(point.x(),point.y()));
////					}
//					
//					for (int i = 0; i < numSeqPts; ++i) {
//						Pointer ptrPoint = opencv_core.cvGetSeqElem(seqPoints,i);
//						CvPoint point = new CvPoint(ptrPoint);
//		                points.add(new Point(point.x(),point.y()));
//					}
//					shapes.add(points);
//					count++;
//				}
//			}
//			
//			contour = contour.h_next();
//		}
//		
//
////		opencv_imgproc.GaussianBlur(detected_edges, detected_edges, new CvSize(11,11), 0, 0, opencv_imgproc.BORDER_DEFAULT);
////		opencv_imgproc.cvCanny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);
////		result = opencv_imgproc.cvFindContours(
////				detected_edges,cvStorage,first_contour,header_size,
////				opencv_imgproc.CV_RETR_LIST,
////				opencv_imgproc.CV_CHAIN_APPROX_SIMPLE);
//	
//		
//		
////		IplImage detected_edges_colored = IplImage.create(
////				imgDetectedEdges.width(),imgDetectedEdges.height(),
////				imgDetectedEdges.depth(),3);
////		opencv_imgproc.cvCvtColor(imgDetectedEdges,detected_edges_colored,
////				opencv_imgproc.CV_GRAY2BGR);
//		
//		int w = imgDetectedEdges.width();
//		int h = imgDetectedEdges.height();
////		ByteBuffer bp = detected_edges_colored.getByteBuffer();
////		System.out.println("width" + detected_edges_colored.width());
////		System.out.println("widthstep" + detected_edges_colored.widthStep());
////		int widthStep = detected_edges_colored.widthStep();
//		int widthStep = imgContoursOverlayed.widthStep();
////		for (int i = 0;i < 5000;++i) {
////			
////			img.
//////	    	y*widthstep + x
////			p = y*widthStep + x;
////			bp.put(i*3 + 0, (byte) 255);
////			bp.put(i*3 + 1, (byte) 0);
////			bp.put(i*3 + 2, (byte) 0);
////			
////		}
///*
////		opencv_core.cvCopy(srcImg,dst);
////		dst = src.clone();//src.copyTo(dst)IplImage.c(src);
////		ByteBuffer canvasBP = dst.getByteBuffer();
//		ByteBuffer canvasBP = imgContoursOverlayed.getByteBuffer();
////		int colorIdx = rand.nextInt() % colors.length;
//		for (ArrayList<Point> s: shapes) {
//			
//			int colorIdx = rand.nextInt(colors.length);
//			int blue = colors[colorIdx].b;
//			int green = colors[colorIdx].g;
//			int red = colors[colorIdx].r;
//			
//			for (Point pt: s) {
////				bp.put(pt.y*widthStep + pt.x*3 + 0, (byte) blue);
////				bp.put(pt.y*widthStep + pt.x*3 + 1, (byte) green);
////				bp.put(pt.y*widthStep + pt.x*3 + 2, (byte) red);
//
//				canvasBP.put(pt.y*widthStep + pt.x*3 + 0, (byte) blue);
//				canvasBP.put(pt.y*widthStep + pt.x*3 + 1, (byte) green);
//				canvasBP.put(pt.y*widthStep + pt.x*3 + 2, (byte) red);
//			}
//		}
//		*/
//
////		Random r = new Random();
////		for (int i=0; i < 200; ++i) {
////			int x = r.nextInt(w);
////			int y = r.nextInt(h);
////			bp.put(y*widthStep + x*3 + 0, (byte) 255);
////			bp.put(y*widthStep + x*3 + 1, (byte) 0);
////			bp.put(y*widthStep + x*3 + 2, (byte) 0);
////		}
//		
////		canvas2.showImage(dst);
////		canvas1.showImage(detected_edges_colored);
//
////		canvas5.showImage(imgContoursOverlayed);
//		
////		System.out.println("num shapes found: " + count);
//		System.out.println("result: " + result);
//
//		//canvas.showImage(detected_edges);
//		
//		opencv_core.cvClearMemStorage(cvStorage);
//	}
//
//	
//	private static void drawPointsOnImage(IplImage dstImg,
//			ArrayList<ArrayList<Point>> shapes) {
//		ByteBuffer canvasBP = dstImg.getByteBuffer();
//		int widthStep = dstImg.widthStep();
//		for (ArrayList<Point> s: shapes) {
//			int colorIdx = rand.nextInt(colors.length);
//			int blue = colors[colorIdx].b;
//			int green = colors[colorIdx].g;
//			int red = colors[colorIdx].r;
//			for (Point pt: s) {
//				canvasBP.put(pt.y*widthStep + pt.x*3 + 0, (byte) blue);
//				canvasBP.put(pt.y*widthStep + pt.x*3 + 1, (byte) green);
//				canvasBP.put(pt.y*widthStep + pt.x*3 + 2, (byte) red);
//			}
//		}
//	}
//
//	private static void drawPoint2DsOnImage(IplImage dstImg,
//			ArrayList<ArrayList<Point2D.Double>> shapes) {
//		ByteBuffer canvasBP = dstImg.getByteBuffer();
//		int widthStep = dstImg.widthStep();
//		for (ArrayList<Point2D.Double> s: shapes) {
//			int colorIdx = rand.nextInt(colors.length);
//			int blue = colors[colorIdx].b;
//			int green = colors[colorIdx].g;
//			int red = colors[colorIdx].r;
//			for (Point2D.Double pt: s) {
//				canvasBP.put((int)(pt.y*widthStep + pt.x*3 + 0), (byte) blue);
//				canvasBP.put((int)(pt.y*widthStep + pt.x*3 + 1), (byte) green);
//				canvasBP.put((int)(pt.y*widthStep + pt.x*3 + 2), (byte) red);
//			}
//		}
//	}
//
//	public static void main(String[] args) {
//		
//		canvas0 = new CanvasFrame("Canvas0: Original Image");
//		canvas1 = new CanvasFrame("Canvas1: HSV");
//		canvas2 = new CanvasFrame("Canvas2: Threshold");
//		canvas3 = new CanvasFrame("Canvas3: Detected Edges");
//		canvas4 = new CanvasFrame("Canvas4: Dilated");
//		canvas5 = new CanvasFrame("Canvas5: Contours");
//		canvas6 = new CanvasFrame("Canvas6: Image H");
//		canvas7 = new CanvasFrame("Canvas7: Image S");
//		canvas8 = new CanvasFrame("Canvas8: Image V");
//		canvas9 = new CanvasFrame("Canvas9: Contours Overlayed");
//		canvas0.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);
////		canvas1.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);
////		canvas2.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);
////		canvas3.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);
////		canvas4.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);
////		canvas5.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);
////		canvas6.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);
////		canvas7.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);
////		canvas8.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);
////		canvas9.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);
//
////		String filename = "C:\\Users\\Anthony\\Desktop\\Notcolored.png";
////		String filename = "C:\\Users\\Anthony\\Desktop\\visionTest\\visiontest6.png";
//		String filename = "C:\\Users\\Anthony\\Desktop\\laserPics\\laser0.png";
////		String filename = "C:\\Users\\Anthony\\Desktop\\testingBlockDetect.png";
////		String filename = "C:\\Users\\Anthony\\Desktop\\simple.bmp";
//		//canvas2 = new CanvasFrame("Original Image");
//
//		srcImg = opencv_highgui.cvLoadImage(filename);
//
////		imgHsv = opencv_core.cvCreateImage(
////				opencv_core.cvGetSize(srcImg), 8, 3);
//		imgHsv = IplImage.create(srcImg.width(),srcImg.height(),
//				srcImg.depth(),srcImg.nChannels());
//		imgH = IplImage.create(srcImg.width(),srcImg.height(),
//				srcImg.depth(),1);
//		imgS = IplImage.create(srcImg.width(),srcImg.height(),
//				srcImg.depth(),1);
//		imgV = IplImage.create(srcImg.width(),srcImg.height(),
//				srcImg.depth(),1);
//        imgThreshold = opencv_core.cvCreateImage(
//        		opencv_core.cvGetSize(srcImg), 8, 1);
//		imgDetectedEdges = IplImage.create(
//				srcImg.width(),srcImg.height(),
//				srcImg.depth(),1);
//		imgDilated = IplImage.create(
//				srcImg.width(),srcImg.height(),
//				srcImg.depth(),1);
//		imgContours = IplImage.create(
//				srcImg.width(),srcImg.height(),
//				srcImg.depth(),1);
//		imgContoursOverlayed = IplImage.create(
//				srcImg.width(),srcImg.height(),
//				srcImg.depth(),srcImg.nChannels());
//		
//
//		
//		// laser
////		int hueLowerLaser = 18;
////		int hueUpperLaser = 40;
////		int satLowerLaser = 0;
////		int satUpperLaser = 90;
////		int valLowerLaser = 150;//160;
////		int valUpperLaser = 255;
//		int hueLowerLaser = 0;//18;
//		int hueUpperLaser = 255;
//		int satLowerLaser = 0;
//		int satUpperLaser = 40;//25;//30;//40;
//		int valLowerLaser = 160;
//		int valUpperLaser = 255;
//		
//		// compute static images
//		computeImgHsv();
//		computeImgThreshhold(
//				hueLowerLaser, hueUpperLaser,
//				satLowerLaser, satUpperLaser,
//				valLowerLaser, valUpperLaser);
//		computeImgDetectedEdges();
//		computeImgDilated();
//
//		// show images
//		canvas0.showImage(srcImg);
//		canvas1.showImage(imgHsv);
//		canvas2.showImage(imgThreshold);
//		canvas3.showImage(imgDetectedEdges);
//		canvas4.showImage(imgDilated);
//		canvas6.showImage(imgH);
//		canvas7.showImage(imgS);
//		canvas8.showImage(imgV);
//		
//
//		
//		
//		/// Load an image
//
//		/// Create a matrix of the same type and size as src (for dst)
////		dst = new IplImage(srcImg);//.crea()CvMat.create(src.height(),src.width(),src.type());
////		dst = IplImage.create(srcImg.width(),srcImg.height(),srcImg.depth(),srcImg.nChannels());//.crea()CvMat.create(src.height(),src.width(),src.type());
////		src_gray = CvMat.create(srcImg.height(),srcImg.width(),opencv_core.CV_8U);
//		
////		System.out.println(
////				srcImg.height() + " " +
////				srcImg.width() + " " +
////				 " ");
////		System.out.println(
////				dst.height() + " " +
////				dst.width() + " " +
////				" ");
////		System.out.println(
////				src_gray.rows() + " " +
////				src_gray.cols() + " " +
////				" ");
//
//		/// Convert the image to grayscale
////		opencv_imgproc.cvCvtColor(srcImg,src_gray,opencv_imgproc.CV_BGR2GRAY);
//
//		// Create a window
//		opencv_highgui.cvNamedWindow(window_name,opencv_highgui.CV_WINDOW_AUTOSIZE);
//
//		// Create a Trackbar for user to enter threshold
//		LowThresholdCallBack lowThreshcallback = new LowThresholdCallBack();
//		opencv_highgui.cvCreateTrackbar("Min Threshold:", window_name, null, 1000, lowThreshcallback);// , ,&lowThreshold, max_lowThreshold );
//		
////		KernelCallback kernelCallback = new KernelCallback();
////		int[] initKernelSize = new int[1];
////		initKernelSize[0] = 3;
////		opencv_highgui.cvCreateTrackbar("Kernel Size:", window_name, initKernelSize, 100, kernelCallback);
//		
//		//opencv_highgui.cvg
//
//		// Show the image
//		//kernelCallback.call(Main.kernel_size);
//		//lowThreshcallback.call(Main.lowThreshold);
//		
//		/// Wait until user exit program by pressing a key
//		opencv_highgui.cvWaitKey(0);
//	}
//
//	
//
//	private static void computeImgHsv() {
//		opencv_imgproc.cvCvtColor(srcImg,imgHsv,
//				opencv_imgproc.CV_BGR2HSV);
//
//		// blur
//		opencv_imgproc.cvSmooth(imgHsv,imgHsv,opencv_imgproc.CV_GAUSSIAN, 3);
//
//		opencv_core.cvSplit(imgHsv, imgH, null, null, null);
//		opencv_core.cvSplit(imgHsv, null, imgS, null, null);
//		opencv_core.cvSplit(imgHsv, null, null, imgV, null);
//	}
//
//	private static void computeImgThreshhold(
//			int hueLower, int hueUpper,
//			int satLower, int satUpper,
//			int valLower, int valUpper) {
//		int hueGreenLowerR = 55; // green
//		int hueGreenUpperR = 62; // green
////	        int hueRedLowerR = 0; // red
////	        weNeedToHandleTheWrapAround!int hueRedUpperR = 25;//25; // red
//		int hueWallLower = 15; // wall
//		int hueWallUpper = 62; // wall
//		int hueLowerR = hueWallLower;
//		int hueUpperR = hueWallUpper;
//
//		
////		opencv_core.cvInRangeS(imgHsv,
////				opencv_core.cvScalar(hueLowerR,220,0,0),
////				opencv_core.cvScalar(hueUpperR,255,255,0),
////				imgThreshold);
//		opencv_core.cvInRangeS(imgHsv,
//				opencv_core.cvScalar(hueLower,satLower,valLower,0),
//				opencv_core.cvScalar(hueUpper,satUpper,valUpper,0),
//				imgThreshold);
//	}
//
//    private static void computeImgDetectedEdges() {
//    	opencv_imgproc.cvCanny(imgThreshold,
//    			imgDetectedEdges, lowThreshold,
//    			lowThreshold*ratio, kernel_size);
//	}
//
//    private static void computeImgDilated() {
//		opencv_imgproc.cvDilate(imgDetectedEdges,
//				imgDilated, null, 1);
//	}
//
//    
//    private static void
//    printContoursWithGetStructure(CvSeq firstContour) {
//    	CvSeq contour = firstContour;//.getStructure();
//    	int numPoints = 0;
//    	int numContours = 0;
//    	
//    	while (contour != null) {
//    		++numContours;
//            // Print all points in contour
//            for (int i = 0; i < contour.total(); ++i)
//            {
//                ++numPoints;
//
//                Pointer ptrPoint = opencv_core.cvGetSeqElem(contour, i);
//                CvPoint point = new CvPoint(ptrPoint);
//                System.out.print("" + point.x() + "," +point.y() + " ");
//            }
//            System.out.println();
//            contour = contour.h_next();
//    	}
//    }
//    
//    void blurImage(IplImage src, IplImage dst,int numChannels) {
//		opencv_imgproc.cvSmooth(src,dst,opencv_imgproc.CV_GAUSSIAN, 3);
////		opencv_imgproc.GaussianBlur(src, detected_edges,
////				new CvSize(3,3), 0, 0, opencv_imgproc.BORDER_DEFAULT);
//	}
//	
//    
//
//	public static class KernelCallback extends CvTrackbarCallback {
//		@Override
//		public void call(int arg0) {
//			kernel_size = Math.max(arg0, 3);
//			updateCanvas();
//		}
//	}
//	
//	public static class LowThresholdCallBack extends CvTrackbarCallback {
//		@Override
//		public void call(int arg0) {
//			lowThreshold = arg0;
//			updateCanvas();
//		}
//	}
//	
//}
