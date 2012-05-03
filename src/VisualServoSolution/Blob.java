package VisualServoSolution;

public class Blob {
	enum BlobColor {
		BLUE,
		GREEN,
		RED,
		YELLOW
	}
	
	BlobColor color;
	double area;
	int px_x;
	int px_y;
	
	Blob(int x, int y, double area, BlobColor color) {
		this.px_x = x;
		this.px_y = y;
		this.area = area;
		this.color = color;
	}
}
