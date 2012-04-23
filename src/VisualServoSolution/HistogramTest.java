package VisualServoSolution;

import java.awt.image.BufferedImage;
import java.awt.image.Raster;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

import VisualServoSolution.Histogram;
import VisualServoSolution.HistogramTest;
import VisualServoSolution.Image;

/**
 * <p>Class for testing the histogram without the robot</p>
 *
 * @author Carrick Detweiler
 **/
public class HistogramTest{


  /**
   * <p>Gets an rss VisualServo.Image type of image from the given
   * BufferedImage.</p>
   *
   * @param image BufferedImage to convert to a VisualServo.Image
   * @return VisualServo.Image representation of image
   **/
  public static Image getRSSImage(BufferedImage image){
    int width = image.getWidth();
    int height = image.getHeight();
    Image rssimg = new Image(width,height);

    System.out.println("BufferedImage.getRSSImage input width: " + width + " height: " + height);

    //Copy the data in
    for(int i = 0; i < width; i++){
      for(int j = 0; j < height; j++){
        int val = image.getRGB(i,j);
        Image.Pixel p = new Image.Pixel(
          ((val>>16) & 0xff),
          ((val>>8) & 0xff),
          (val & 0xff));
        rssimg.setPixel(i,j,p);
      }
    }

    return rssimg;
  }


  /**
   * <p>Gets a BufferedImage from an rss VisualServo.Image</p>
   *
   * @param image rss image
   * @return a BufferedImage representation of image
   **/
  public static BufferedImage getBufferedImage(Image image){
    int width = image.getWidth();
    int height = image.getHeight();
    BufferedImage bimage = new BufferedImage(width,height,BufferedImage.TYPE_INT_RGB);

    System.out.println("BufferedImage.getBufferedImage input width: " + width + " height: " + height);

    //Copy the data in
    for(int i = 0; i < width; i++){
      for(int j = 0; j < height; j++){
        Image.Pixel p = image.getPixel(i,j);
        int rgbVal = (p.getRed()<<16) + (p.getGreen()<<8) + p.getBlue();
        bimage.setRGB(i,j,rgbVal);
      }
    }

    return bimage;
  }

  /**
   * <p>Just read in the image passed image and the desired output image</p>
   **/
  public static void main(String[] arg) {
    if(arg.length != 2){
      System.out.println("Usage: java VisualServo.HistogramTest inimage outimage");
      System.exit(1);
    }

    String inputFilename = arg[0];
    String outputFilename = arg[1];
    System.out.println("Reading image " + inputFilename 
                       + " writting image " + outputFilename);

    try{
      //Get the image in
      BufferedImage inputImage = ImageIO.read(new File(inputFilename));
      Image rssImage = HistogramTest.getRSSImage(inputImage);

      Histogram.getHistogram(rssImage,rssImage,false);

      //Now ouput it
      BufferedImage outputImage = HistogramTest.getBufferedImage(rssImage);
      ImageIO.write(outputImage,"JPEG",new File(outputFilename));
    } 
    catch (IOException e){
      e.printStackTrace();
    }
  }

}