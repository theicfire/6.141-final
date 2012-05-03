package VisualServoSolution;

import java.util.ArrayList;
import java.util.List;

import VisualServoSolution.Image;

/**
 * An Image object is a easy-to-use representation of the pixelated image data
 * from Carmen
 * 
 * @author Vinayak Ranade
 *
 */
public class Image{

	/**
	 * Encapsulates the information of a single pixel
	 * 
	 * @author Vinayak Ranade
	 *
	 */
	public static class Pixel{
		private int red, green, blue;
		private float hue, sat, bright;

    /**
     * <p>Copy constructor for the pixels</p>
     **/
    public Pixel(Pixel p){
      this(p.getRed(),p.getGreen(),p.getBlue());
    }

		public Pixel(int r, int g, int b){
			this.red = r;
			this.green = g;
			this.blue = b;

			float[] hsb = java.awt.Color.RGBtoHSB(r, g, b, null);
			hue = hsb[0];
			sat = hsb[1];
			bright = hsb[2];
		}
		public int getRed() {
			return red;
		}
		public int getGreen() {
			return green;
		}
		public int getBlue() {
			return blue;
		}
		public float getHue() {
			return hue;
		}
		public void setRed(int red) {
			this.red = red;
		}
		public void setGreen(int green) {
			this.green = green;
		}
		public void setBlue(int blue) {
			this.blue = blue;
		}
		public void setHue(float hue) {
			this.hue = hue;
		}
		public void setSaturation(float sat) {
			this.sat = sat;
		}
		public void setBrightness(float bright) {
			this.bright = bright;
		}
		public float getSaturation() {
			return sat;
		}
		public float getBrightness() {
			return bright;
		}
	}

	//FIELDS
	private int width, height;
	public List<Pixel> pixels;

  /**
   * <p>Gets the width of this image</p>
   **/
  public int getWidth(){
    return width;
  }

  /**
   * <p>Gets the height of this image</p>
   **/
  public int getHeight(){
    return height;
  }

	/**
	 * Takes in the source character array (the Carmen compatible form)
	 * along with the width and height of the Image. This information
	 * is all available in any CameraMessage
	 * 
	 * @param src
	 * @param width
	 * @param height
	 */
	public Image(byte[] src, int width, int height){
		pixels = new ArrayList<Pixel>();
		this.width = width;
		this.height = height;
		int srcIndex = 0;
		int r, g, b;

		for(int y = 0; y<height; y++){
			for(int x = 0; x<width; x++){
				r = src[srcIndex++] & 0xff;
				g = src[srcIndex++] & 0xff;
				b = src[srcIndex++] & 0xff;
				pixels.add(new Pixel(r, g, b));
			}
		}
	}

	public Image(byte[] src, int width, int height, int widthStep){
		// widthStep is the number of bytes per line in src
		pixels = new ArrayList<Pixel>();
		this.width = width;
		this.height = height;
		int srcIndex = 0;
		int r, g, b;

		for(int y = 0; y<height; y++){
			int yTimesWStep = y*widthStep;
			for(int x = 0; x<width; x++) {
				int threeX = 3*x;
				r = src[yTimesWStep + threeX + 0] & 0xff;
				g = src[yTimesWStep + threeX + 1] & 0xff;
				b = src[yTimesWStep + threeX + 2] & 0xff;
				pixels.add(new Pixel(r, g, b));
			}
		}
	}

	/**
	 * Makes an Image out of a linear representation of the pixel matrix
	 * @param pixels
	 * @param width
	 * @param height
	 */
	public Image(List<Pixel> pixels, int width, int height){
		this.pixels = pixels;
		this.width = width;
		this.height = height;
	}

	/**
	 * Makes a default Image with all pixels set to black
	 * @param width
	 * @param height
	 */
	public Image(int width, int height){
		this.pixels = new ArrayList<Pixel>(width*height);
		this.width = width;
		this.height = height;
		for(int i=0; i<width*height; i++){
			pixels.add(new Pixel(255,255,255));
		}
	}

  /**
   * <p>Creates a copy of the given image</p>
   **/
  public Image(Image image){
    this.width = image.getWidth();
    this.height = image.getHeight();
    this.pixels = new ArrayList<Pixel>(width*height);
    List<Pixel> imagePixels = image.getAllPixels();
		for(int i=0; i<width*height; i++){
			pixels.add(new Pixel(imagePixels.get(i)));
		}
  }
	
	/**
	 * Gets a Pixel from the pixel matrix
	 * @param x The x coordinate of the pixel (0<=x<=width)
	 * @param y The y coordinate of the pixel (0<=y<=width)
	 * @return
	 */
	public Pixel getPixel(int x, int y){
		assert x>=0 && y>=0;
		assert x < width && y < height;
		return pixels.get(width*(y) + x);
	}

	/** Gets a Pixel from the pixel matrix
	 * @param x The x coordinate of the pixel (0<=x<=width)
	 * @param y The y coordinate of the pixel (0<=y<=width)
	 */
	public void setPixel(int x, int y, Pixel p){
		assert x>=0 && y>=0;
		assert x < width && y < height;
		pixels.set(width*y + x, p);
	}
	
	/**
	 * Return a list of all the pixels indexed from
	 * 0 to width*height-1
	 * @return
	 */
	public List<Pixel> getAllPixels(){
		List<Pixel> allPixels = new ArrayList<Pixel>(pixels.size());
		for(Pixel p : pixels){
			allPixels.add(p);
		}
		return allPixels;		
	}
	
//	/**
//	 * Do NOT use this more than once for a given Image object
//	 * (computation and memory intensive)
//	 */
//	public List<List<Pixel>> getRows(){
//
//		List<List<Pixel>> rows = new ArrayList<List<Pixel>>();
//
//		for(int j = 1; j<=height; j++){
//			List<Pixel> row = new ArrayList<Pixel>();
//			for(int i = 1; i<=width; i++){
//				row.add(getPixel(i, j));
//			}
//			rows.add(row);
//		}
//
//		return rows;
//	}
//
//	/** 
//	 * Do NOT use this more than once for a given Image object
//	 * (computation and memory intensive)
//	 */
//	public List<List<Pixel>> getColumns(){
//
//		List<List<Pixel>> columns = new ArrayList<List<Pixel>>();
//
//		for(int j = 1; j<=height; j++){
//			List<Pixel> column = new ArrayList<Pixel>();
//			for(int i = 1; i<=width; i++){
//				column.add(getPixel(j, i));
//			}
//			columns.add(column);
//		}
//		return columns;
//	}
//
//		for(int j = 1; j<=height; j++){
//			List<Pixel> column = new ArrayList<Pixel>();
//			for(int i = 1; i<=width; i++){
//				column.add(getPixel(j, i));
//			}
//			columns.add(column);
//		}
//		return columns;
//	}

	/**
	 * Converts the Image into a byte[] array
	 * @return
	 */
	public byte[] toArray(){
		byte[] image = new byte[width*height*3+1];
		int destIndex = 0;

		for(int i = 0; i< pixels.size(); i++){
			Pixel p = pixels.get(i);
			image[destIndex++] = (byte)p.red;
			image[destIndex++] = (byte)p.green;
			image[destIndex++] = (byte)p.blue;
			if(destIndex > image.length){
				System.out.println("The length of pixels is" + pixels.size());
				System.out.println("But the length it is supposed to be is " + width*height*3);
				System.out.println("Failed when i = " + i + " and destIndex = " + destIndex);
				throw new RuntimeException("Halt");
			}
		}
		return image;
	}

        public static byte[] RGB2BGR(byte[] data, int width, int height) {
	    byte[] ret = new byte[width*height*3];
	    for (int r = 0; r < height; r++) {
		for (int c = 0; c < width; c++) {
		    int i = (r*(int)width + c)*3;
		    ret[i+0] = data[i+2];
		    ret[i+1] = data[i+1];
		    ret[i+2] = data[i+0];
		}
	    }
	    return ret;
	}

}