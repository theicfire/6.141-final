package VisualServoSolution;

import java.util.Arrays;
import java.lang.Math;

import javax.swing.*;
import java.awt.Polygon;
import java.awt.Graphics;
import java.awt.image.MemoryImageSource;
import java.awt.Image;
import java.awt.Rectangle;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.awt.Insets;

/*
 *Author: Edsinger, from http://in3www.epfl.ch/~jpilet/serie6/GaussianBlur.java
 *  Apply a gaussian filter of size 5x5, with sigma=1.
 *
 * The 2D filter: 
 *         [  0.7  3.3  5.5  3.3  0.7 ]
 *         [  3.3 15.0 24.7 15.0  3.3 ]
 * 1/256 * [  5.5 24.7 40.7 24.7  5.5 ]
 *         [  3.3 15.0 24.7 15.0  3.3 ]
 *         [  0.7  3.3  5.5  3.3  0.7 ]
 *
 * can be separated into 2 1D filter:
 * 1/16 * [ 0.9  3.9  6.4  3.9  0.9 ]
 *
 * the  following approximation is acceptable:
 * 1/16 * [ 1 4 6 4 1 ] 
 *
 */ 

public class GaussianBlur {


  // extract a channel value from a RGB 'int' packed color 
  static int getChannel(int color, int channel) {
    return  (color >> (8*channel)) & 0xFF;
  }

  // shift a color value of the corresponding channel offset
  static int channelShift(int color, int channel) {
    return (color&0xFF)<<(8*channel);
  }

  // sample a repeated image. Returns a valid result for any x and y.
  // w is the image width, h the image height and pix the image itself.
  static int getPixRepeat(int x, int y, int w, int h, int [] pix)
  {
    int x2 = x % w;
    if (w==1) x2=0;
    int y2 = y % h;
    if (h==1) x2=0;
    if (x2<0) x2+= w;
    if (y2<0) y2+= h;
    return pix[y2*w+x2];
  }
  
  // appy a 5x5 gaussian blur of sigma = 1.
  // Put the result into dstpix. Both images must have the same size, 
  // defined by w and h (for width and height).
  static void apply(int[] srcpix, int[] dstpix, int w, int h) {

    int [] tmppix = new int[w*h + 1];

    // horizontal filtering
    for (int y=0; y<h; ++y) {
      
      int pos = y*w;

      for(int x=0; x<w; ++x) {

        // accumulate each channel for this pixel
        int r=0;
        for (int c=0; c<3; c++) {
          // [1 4 6 4 1] filter
          r += channelShift((
          (getChannel(getPixRepeat(x-2,y,w,h,srcpix), c) +
           getChannel(getPixRepeat(x-1,y,w,h,srcpix), c)*4 +
           getChannel(getPixRepeat(x  ,y,w,h,srcpix), c)*6 +
           getChannel(getPixRepeat(x+1,y,w,h,srcpix), c)*4 +
           getChannel(getPixRepeat(x+2,y,w,h,srcpix), c)) / 16
          ), c);
        }

        // store the pixel
        tmppix[pos + x] = r;
      }
    }

    // vertical filtering
    for (int x=0; x<w; ++x) {

      int pos = x;
      
      for (int y=0; y<h; y++) {
        int r=0;
        for (int c=0; c<3; c++) {
          r += channelShift((
          (getChannel(getPixRepeat(x,y-2,w,h,tmppix), c) +
           getChannel(getPixRepeat(x,y-1,w,h,tmppix), c)*4 +
           getChannel(getPixRepeat(x,y  ,w,h,tmppix), c)*6 +
           getChannel(getPixRepeat(x,y+1,w,h,tmppix), c)*4 +
           getChannel(getPixRepeat(x,y+2,w,h,tmppix), c)) / 16
          ), c);
        }
        dstpix[pos] = r+(0xff << 24);
        pos += w;
      }
    }
  }

  
  static void apply(byte[] src, byte[] dest, int width, int height) {
    int imagePacked[] = new int[width * height];
    int imageFiltered[] = new int[width * height];
    
    convertToPacked(src, imagePacked, width, height);
    apply(imagePacked, imageFiltered, width, height);
    convertFromPacked(imageFiltered, dest, width, height);
  }
  
  /**
   * <p>Convert an image to a packed image for use by Gaussian Blur</p>
   * 
   * @param src
   * @param dest
   */
  static void convertToPacked(byte[] src, int[] dest, int width, int height) {
    int srcIndex = 0;
    int destIndex = 0;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int red = src[srcIndex++] & 0xff;
        int green = src[srcIndex++] & 0xff;
        int blue = src[srcIndex++] & 0xff;
        dest[destIndex++] = (0xff << 24) | (red << 16) | (green << 8)
            | blue;
      }
    }
  }

  
  /**
   * <p>Unpack an image, for use after Gaussian Blur</p>
   * 
   * @param src
   * @param dest
   */
  static void convertFromPacked(int[] src, byte[] dest, int width, int height) {
    int srcIndex = 0;
    int destIndex = 0;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int red = src[srcIndex] >> 16 & 0xff;
        int green = src[srcIndex] >> 8 & 0xff;
        int blue = src[srcIndex++] & 0xff;
        dest[destIndex++] = (byte) red;
        dest[destIndex++] = (byte) green;
        dest[destIndex++] = (byte) blue;
      }
    }
  }

  
}
