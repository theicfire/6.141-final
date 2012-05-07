//import java.awt.BorderLayout;
//import java.awt.Container;
//import java.awt.GridBagConstraints;
//import java.awt.GridBagLayout;
//import java.awt.Point;
//import java.awt.event.ActionEvent;
//import java.awt.event.ActionListener;
//import java.io.File;
//import java.net.MalformedURLException;
//import java.net.URL;
//import java.util.ArrayList;
//
//import javax.swing.BorderFactory;
//import javax.swing.ImageIcon;
//import javax.swing.JButton;
//import javax.swing.JFileChooser;
//import javax.swing.JFrame;
//import javax.swing.JLabel;
//import javax.swing.JMenu;
//import javax.swing.JMenuBar;
//import javax.swing.JMenuItem;
//import javax.swing.JPanel;
//
//import com.googlecode.javacv.cpp.opencv_calib3d;
//import com.googlecode.javacv.cpp.opencv_core;
//import com.googlecode.javacv.cpp.opencv_core.CvMat;
//
//public class ImageViewer extends JFrame implements ActionListener {
//	
//	HomographyPointsTable homographyTable;
//	private JLabel label;
//	private JMenuItem openItem;
//	private JMenuItem exitItem;
//	JPanel leftPane;
//	ImageComponent image;
//
//	public ImageViewer() {
//		this.homographyTable = new HomographyPointsTable();
//		setTitle("ImageViewer");
//		//setSize(300, 400);
//
//		JMenuBar mbar = new JMenuBar();
//		JMenu m = new JMenu("File");
//		openItem = new JMenuItem("Open");
//		openItem.addActionListener(this);
//		m.add(openItem);
//		exitItem = new JMenuItem("Exit");
//		exitItem.addActionListener(this);
//		m.add(exitItem);
//		mbar.add(m);
//		setJMenuBar(mbar);
//
//		//Set up the content pane.
//		addComponentsToPane();
//		this.setDefaultCloseOperation(EXIT_ON_CLOSE);
//  }
//
//
//
//	public static void main(String[] args) {
//	    JFrame frame = new ImageViewer();
//
//	    frame.pack(); // resize as necessary
//	    frame.setVisible(true); // show window
//  }
//
//	protected static final String STRING_TEXT_FIELD = "JTextField";
//	protected static final String STRING_BUTTON_COMPUTE = "STRING_BUTTON_COMPUTE";
//	protected static final String STRING_BUTTON_DRAW_SRC_POINTS = "STRING_BUTTON_DRAW_SRC_POINTS";
//	protected static final String STRING_BUTTON_ADD_POINT = "BUTTON_ADD_POINT";
//	protected static final String STRING_BUTTON_DEL_POINTS = "BUTTON_DEL_POINTS";
//
//	private void addComponentsToPane() {
//		Container contentPane = getContentPane();
//		GridBagConstraints gbc;
//		contentPane.setLayout(new GridBagLayout());
//
//		// image
//		label = new JLabel();
//		gbc = new GridBagConstraints();
//		gbc.fill = GridBagConstraints.HORIZONTAL;
//		gbc.gridx = 0;
//		gbc.gridy = 0;
//        // left pane
//		leftPane = new JPanel(new BorderLayout());
////        leftPane.add(label);
////        C:\\Users\\Anthony\\Desktop\\Notcolored.png
////        
////        
//		contentPane.add(leftPane,gbc);
//
//		// button
//		JButton computeButton = new JButton("Compute Homography");
//		computeButton.setActionCommand(STRING_BUTTON_COMPUTE);
//		computeButton.addActionListener(this);
//		// button
//		JButton drawButton = new JButton("Draw Src Points");
//		drawButton.setActionCommand(STRING_BUTTON_DRAW_SRC_POINTS);
//		drawButton.addActionListener(this);
//		// button
//		JButton addButton = new JButton("Add Point");
//		addButton.setActionCommand(STRING_BUTTON_ADD_POINT);
//		addButton.addActionListener(this);
//		// button
//		JButton delButton = new JButton("Delete Points");
//		delButton.setActionCommand(STRING_BUTTON_DEL_POINTS);
//		delButton.addActionListener(this);
//		JPanel buttonPanel = new JPanel();
//		buttonPanel.add(computeButton);
//		buttonPanel.add(drawButton);
//		buttonPanel.add(addButton);
//		buttonPanel.add(delButton);
//
//		GridBagLayout rightPanelLayout = new GridBagLayout();
//		JPanel rightPanel = new JPanel(rightPanelLayout);
//		gbc = new GridBagConstraints();
//		gbc.fill = GridBagConstraints.BOTH;
////		gbc.weighty = 0.0;
//		gbc.gridx = 0;
//		gbc.gridy = 0;
//		rightPanel.add(this.homographyTable,gbc);
//		
//		gbc = new GridBagConstraints();
//		gbc.fill = GridBagConstraints.BOTH;
////		gbc.weighty = 1.0;
//		gbc.gridx = 0;
//		gbc.gridy = 1;
//		rightPanel.add(buttonPanel,gbc);
//
//		rightPanel.setBorder(BorderFactory.createCompoundBorder(
//				BorderFactory.createTitledBorder("Homography Mappings"),
//				BorderFactory.createEmptyBorder(5,5,5,5)));
//        gbc = new GridBagConstraints();
//		gbc.fill = GridBagConstraints.BOTH;
//		gbc.weightx = 0.5;
//		gbc.weighty = 1.0;
//		gbc.gridx = 1;
//		gbc.gridy = 0;
//		gbc.gridheight = 1;
//		contentPane.add(rightPanel,gbc);
//
//	
//	}
//
//	@Override
//	public void actionPerformed(ActionEvent evt) {
//		Object source = evt.getSource();
//		if (source == openItem) {
//			JFileChooser chooser = new JFileChooser();
//			chooser.setCurrentDirectory(new File("."));
//			chooser.setFileFilter(new javax.swing.filechooser.FileFilter() {
//				public boolean accept(File f) {
//					return f.getName().toLowerCase().endsWith(".png")
//						|| f.isDirectory();
//				}
//				public String getDescription() {
//					return "PNG Images";
//				}
//			});
//
//			int r = chooser.showOpenDialog(this);
//			if (r == JFileChooser.APPROVE_OPTION) {
//				String name = chooser.getSelectedFile().getAbsolutePath();
////				label.setIcon(new ImageIcon(name));
//				
//				leftPane.removeAll();
//				try {
//					this.image = new ImageComponent(new URL("file:/" +name));
//				} catch (MalformedURLException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//				
//				leftPane.add(this.image);
//				
//			}
//		} else if (source == exitItem) {
//			System.exit(0);
//		} else if (evt.getActionCommand() == STRING_BUTTON_ADD_POINT) {
//			this.homographyTable.addPoint();
//		} else if (evt.getActionCommand() == STRING_BUTTON_DEL_POINTS) {
//			this.homographyTable.delPoint();
//		} else if (evt.getActionCommand() == STRING_BUTTON_COMPUTE) {
//			this.computeHomography();
//		} else if (evt.getActionCommand() == STRING_BUTTON_DRAW_SRC_POINTS) {
//			this.drawPoints();
//		}
//	}
//	  
//	private void drawPoints() {
//		ArrayList<HomographySrcDstPoint> list =
//			this.homographyTable.getHomographySrcDst();
//
//		if (image != null) {
//			this.image.points.clear();
//			if (list != null) {
//				for (int i = 0; i < list.size(); ++i) {
//					HomographySrcDstPoint p = list.get(i);
//					this.image.points.add(new Point((int)(p.src_x),(int)(p.src_y)));
//				}
//			}
//			this.image.repaint();
//		}
//
//	}
//
//
//
//	private void computeHomography() {
//		ArrayList<HomographySrcDstPoint> list =
//			this.homographyTable.getHomographySrcDst();
//		
//		if (list == null || list.size() < 4) {
//			return;
//		}
//
//		CvMat matSrc = CvMat.create(list.size(),2);
//		CvMat matDst = CvMat.create(list.size(),2);
//
//		for(int i = 0; i < list.size(); ++i){
//	        //Add this point to matSrc and matDst
//			HomographySrcDstPoint p = list.get(i);
//	        matSrc.put(i,0,p.src_x);
//	        matSrc.put(i,1,p.src_y);
//	        matDst.put(i,0,p.dst_x);
//	        matDst.put(i,1,p.dst_y);
//	    }
//		
//		double ransacReprojThreshold = 3;
//		CvMat mask = null;
////		CvMat matH1 = opencv_core.cvCreateMat(3,3,opencv_core.CV_32FC1);
//		CvMat matH2 = opencv_core.cvCreateMat(3,3,opencv_core.CV_32FC1);
////		int result = opencv_calib3d.cvFindHomography(matSrc,matDst,matH1);
//		int result2 = opencv_calib3d.cvFindHomography(matSrc,matDst,matH2,
//				opencv_calib3d.CV_RANSAC, ransacReprojThreshold, mask);
//
////	    for( int i = 0; i < 3; ++i) {
////	        for( int j = 0; j < 3; ++j) {
////	        System.out.print(matH1.get(i,j) + ",");
////	        }
////	        System.out.println();
////	    }
////        System.out.println();
//	    for( int i = 0; i < 3; ++i) {
//	        for( int j = 0; j < 3; ++j) {
//	        System.out.print(matH2.get(i,j) + ",");
//	        }
//	        System.out.println();
//	    }
//		
//	}
//
//}
