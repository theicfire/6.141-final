package Testing;

import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.CvArr;
import com.googlecode.javacv.cpp.opencv_core.IplImage;
import com.googlecode.javacv.cpp.opencv_highgui;
import com.googlecode.javacv.cpp.opencv_imgproc;

public class testing {

	public static void main(String[] args) {

		// load image
		String filename = "/home/rss-student/Desktop/testingBlockDetect.png";
		IplImage originalImage = opencv_highgui.cvLoadImage(null);
		
//		CvArr image = null;
//		CvArr edges = null;
//		double thresh1 = 0.0;
//		double thresh2 = 0.0;
//		int apertureSize = 3; // kernel size
//		opencv_imgproc.cvCanny(image, edges, thresh1, thresh2, apertureSize);
		
	}
	
	
}
