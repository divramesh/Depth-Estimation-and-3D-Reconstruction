#include <stdio.h>
#include"opencv\cv.h"
#include"opencv\highgui.h"
#include <iostream>
#include <sstream>
#include <time.h>
#include<opencv2\imgproc\imgproc.hpp>

#define DEPTH_SENSOR		2
#define RGB_SENSOR			0
#define EXT_SENSOR			1
#define NUM_COLOR_SENSORS	2
#define NUM_SENSORS			3

using namespace cv;
using namespace std;

// Interpolate color of a point with non-integer coordinates in an image
// You can use this function to get smoother outputs, but it is optional
Vec3b avSubPixelValue8U3( const Point2f pt, const Mat img )
{
	int floorx = (int)floor( pt.x );
	int floory = (int)floor( pt.y );

	if( floorx < 0 || floorx >= img.cols-1 || 
		floory < 0 || floory >= img.rows-1 )
		return 0;

	float px = pt.x - floorx;
	float py = pt.y - floory;

	Vec3b tl = img.at<Vec3b>(floory,floorx);
	Vec3b tr = img.at<Vec3b>(floory,floorx+1);
	Vec3b bl = img.at<Vec3b>(floory+1,floorx);
	Vec3b br = img.at<Vec3b>(floory+1,floorx+1);
	Vec3b result;
	for (int i=0;i<3;i++)
		result[i] = (unsigned char)floor(tl[i] * (1-px)*(1-py) + tr[i]*px*(1-py) + bl[i]*(1-px)*py + br[i]*px*py + 0.5 );

	return result;
}

// Load the depth image, the RGB & the external image for a specific frame
// If all images can be loaded, return true
// Otherwise, return false
bool loadImages(char* src, int frame, Mat& depthMat, Mat& rgbMat, Mat& extMat){
	char fname[10];
	char fpath[100];

	// load depth image
	printf("Frame %04d\n",frame);
	sprintf(fname,"%04d.png",frame);
	sprintf(fpath,"%s/Depth/%s",src,fname);
	depthMat = imread(fpath, CV_LOAD_IMAGE_ANYDEPTH);
	if (!depthMat.data ) {
		return false;
	}
	
	// load RGB image
	sprintf(fpath,"%s/RGB/%s",src,fname);
	rgbMat = imread(fpath);
	if (!rgbMat.data) {
		return false;
	}
	
	// load external image
	sprintf(fpath,"%s/Ext/%s",src,fname);
	extMat = imread(fpath);
	if (!extMat.data) {
		return false;
	}
	// TODO 0: In the sample dataset, the external images need to be filipped vertically to have the same direction with the others
	// In your own dataset, however, you may not need to do it. Check your images and comment out the statement below if not needed.
	flip(extMat,extMat,1);
	return true;
}

// Extract board information from the input file
bool getboardInfor(char* boardInforPath, Size &boardSize, float &squareSize, Rect_<float> &boardRegion){
	FILE* file = fopen(boardInforPath, "r");
	if (!file) {
		fprintf(stderr,"Error! Can't find the board information file!\n");
		return  false;
	}

	float right, bottom;
	fscanf(file, "%d %d %f %f %f %f %f",&boardSize.width,&boardSize.height,&squareSize, &boardRegion.x, &boardRegion.y, &right, &bottom);
	boardRegion.width = right - boardRegion.x;
	boardRegion.height = bottom - boardRegion.y;
	fclose(file);
	return true;
}

// Compute 3-D coordinates of the checker inner corners in {B}
// Input:
//		boardSize : checker size
//		squareSize: side of each cell in the checker pattern
// Output:
//		inCorners3D: list of computed 3-D coordinates
//		matInCorners3D: matrices 4xN of computed 3-D homogeneous coordinates
//
void calcInnerCorner(Size boardSize, float squareSize, vector<Point3f>& inCorners3D, Mat &matInCorners3D)
{
	// TODO I.1.a: Compute 3-D coordinates of the checker inner corners in {B}
	//             Remember to list them in order from top-left corner to the right-bottom one
	//
	// fill your code here
	//
	float p, q;
	p=-(boardSize.height-1)/2.0;
	for (int i=0; i<boardSize.height; i++) {
		q=-(boardSize.width-1)/2.0;
		for (int j=0; j<boardSize.width; j++) {
			inCorners3D.push_back(cv::Point3f(q*squareSize, p*squareSize, 0.0f));
			q++;
		}
		p++;
	}

	convertPointsToHomogeneous(inCorners3D, matInCorners3D);
	return;
}

// Compute 3-D coordinates of the board outer corners in {B}
// Input:
//		boardRegion
// Output:
//		boardCorners3D
//		matBoardCorners3D
//
void calcOuterCorner(Rect_<float> boardRegion, vector<Point3f>& boardCorners3D, Mat &matBoardCorners3D)
{
	// TODO I.1.b: Compute 3-D coordinates of the board 4 outer corners in {B}
	//
	// fill your code here
	//
	 
		 boardCorners3D.push_back(cv::Point3f(-boardRegion.width/2.0, -boardRegion.height/2.0, 0.0f));
		 boardCorners3D.push_back(cv::Point3f( boardRegion.width/2.0, -boardRegion.height/2.0, 0.0f));
		 boardCorners3D.push_back(cv::Point3f(-boardRegion.width/2.0, boardRegion.height/2.0, 0.0f));
		 boardCorners3D.push_back(cv::Point3f( boardRegion.width/2.0, boardRegion.height/2.0, 0.0f));

		 convertPointsToHomogeneous(boardCorners3D, matBoardCorners3D);
}

// calibrate a camera using Zhang's method
bool runCalibration(vector<vector<Point3f>> newinCorners3D, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, 
	vector<vector<Point2f> > incorner2DPoints, vector<Mat>& rvecs, vector<Mat>& tvecs)
{
	// TODO II.1.2: Calibrate the camera using function calibrateCamera of OpenCV
	//              Choose suitable flags 
	//
	// fill your code here
	//
	  

		
	calibrateCamera(newinCorners3D, incorner2DPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_FIX_PRINCIPAL_POINT + CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_FIX_K1 + CV_CALIB_FIX_K2 + CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5 + CV_CALIB_FIX_K6);

	return true;
}

// Find a non-zero point from a starting point in a thresholded image
// Input:
//		img    : the thresholded image
//		start  : starting point
//		maxDist: maximum distance to search
// Output:
//		dst    : a non-zero point
// If no point is found, return false
bool findClosestPoint(Mat img, Point2f start, Point2f &dst, int maxDist = 10){
	int x = floor(start.x + 0.5);
	int y = floor(start.y + 0.5);
	if (img.at<unsigned char>(y,x) > 0){
		dst = start;
		return true;
	}
	for (int i=1;i<=maxDist;i++){
		if (x - i >= 0){				// Search on the left
			for (int j=-i;j<=i;j++){
				if (y + j < 0 || y + j > img.rows || img.at<unsigned char>(y + j,x-i) == 0) continue;
				dst.x = x -i;
				dst.y = y + j;
				return true;
			}
		}
		
		if (x + i < img.cols){			// Search on the right
			for (int j=-i;j<=i;j++){
				if (y + j < 0 || y + j > img.rows || img.at<unsigned char>(y + j,x+i) == 0) continue;
				dst.x = x + i;
				dst.y = y + j;
				return true;
			}
		}
		
		if (y - i >= 0){				// Search upward
			for (int j=-i;j<=i;j++){
				if (x + j < 0 || x + j > img.cols || img.at<unsigned char>(y - i,x+j) == 0) continue;
				dst.x = x + j;
				dst.y = y - i;
				return true;
			}
		}
		
		if (y + i < img.rows){			// Search downward
			for (int j=-i;j<=i;j++){
				if (x + j < 0 || x + j > img.cols || img.at<unsigned char>(y + i,x+j) == 0) continue;
				dst.x = x + j;
				dst.y = y + i;
				return true;
			}
		}
	}
	dst = start;
	return false;
}

// Write the pointcloud extracted from a depth image & its colored image to PLY file
// Input:
//		count  : the number of points in the cloud
//		f      : focal length of the depth sensor
//		color  : the colored depth image
//		fname  : file name
void writePLY(int count, float f, Mat color, Mat depth, char* fname){
	FILE* file = fopen( fname, "w");

	if ( !file )
    {
		std::cerr << "Creation Error\n";
        return;
    }

	fprintf( file, "ply\n");
	fprintf( file, "format ascii 1.0\n" );
	fprintf( file, "element vertex %d\n", count );
	fprintf( file, "property float x\nproperty float y\nproperty float z\n" );
	fprintf( file, "property uchar blue\nproperty uchar green\nproperty uchar red\n");
	fprintf( file, "end_header\n");

	for (int i=0;i<depth.cols;i++){
		for (int j=0;j<depth.rows;j++){
			if (depth.at<short>(j,i) > 0){
				float Z = depth.at<short>(j,i);
				float X = (i-depth.cols/2) * Z/f;
				float Y = (j-depth.rows/2) * Z/f;
				Vec3b colors = color.at<Vec3b>(j,i);
				fprintf( file, "%f %f %f %d %d %d\n",X, Y, Z, colors[0], colors[1], colors[2]);
			}
		}
	}
	fclose(file);
}


cv::Point2f computeIntersect(cv::Vec4i a, cv::Vec4i b)
{
    int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3];
    int x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];

    if (float d = ((float)(x1-x2) * (y3-y4)) - ((y1-y2) * (x3-x4)))
    {
        cv::Point2f pt;
        pt.x = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)) / d;
        pt.y = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)) / d;
        return pt;
    }
    else
        return cv::Point2f(-1, -1);
}


void sortCorners(vector<Point2f>& board2DCorners, Point2f center)
{
   vector<Point2f> top, bot;

    for (int i = 0; i < board2DCorners.size(); i++)
    {
        if (board2DCorners[i].y < center.y)
            top.push_back(board2DCorners[i]);
        else
            bot.push_back(board2DCorners[i]);
    }

    cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
    cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
    cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
    cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];

    board2DCorners.clear();
    board2DCorners.push_back(tl);
    board2DCorners.push_back(tr);
    board2DCorners.push_back(bl);
    board2DCorners.push_back(br);
}



int main(int argc, char** argv){
	char src[80] = "D:/USC/Spring_2014/CSCI_574/Project/Project2/Data_Sh-2014-04-23/Data_Sh";					// path to the dataset
	char dst[81] = "D:/USC/Spring_2014/CSCI_574/Project/CSCI_574/SensorsCalibration_skeleton/Output2";				// path to the output directory
	char boardInfor[80] = "D:/USC/Spring_2014/CSCI_574/Project/CSCI_574/Data/board.txt";  // the board information file

	int start = 40;								// start frame
	int end = 100;								// the last frame can be accessed in the dataset
	int step = 1;								// step in frame numbers to process
	int nFrame2Use = 10;						// number of frames can be used to calibrate
	Rect_<float> boardRegion(-150, -103.5, 138, 110.5);		// boardRegion

	vector<int> goodFrames;						// list of good frames to use for calibration
	Size boardSize;
	boardSize.width = 5;
	boardSize.height = 4;
	float squareSize = 35;
	float fc = 285.171;
	float fd = 285.171;

	int arg = 0;								// Process input parameters
	while( ++arg < argc) 
	{ 
		// Input directory
		if( !strcmp(argv[arg], "-i") )
			strcpy(src, argv[++arg] );
		
		// Output directory
		if( !strcmp(argv[arg], "-o") )
			strcpy(dst, argv[++arg] );

		// First frame
		if( !strcmp(argv[arg], "-b") )
			start = atoi( argv[++arg] );
		
		// Step
		if( !strcmp(argv[arg], "-s") )
			step = atoi( argv[++arg] );

		// Last frame
		if( !strcmp(argv[arg], "-e") )
			end = atoi( argv[++arg] );

		// Number of frames to calibrate
		if( !strcmp(argv[arg], "-n") )
			nFrame2Use = atoi( argv[++arg] );

		// Focal length
		if( !strcmp(argv[arg], "-fc") )
			fc = atof( argv[++arg] );
		
		// Focal length
		if( !strcmp(argv[arg], "-fd") )
			fd = atof( argv[++arg] );

		// Board information file
		if( !strcmp(argv[arg], "-v") )
			strcpy(boardInfor, argv[++arg] );
	}

	// get board information from file
	getboardInfor(boardInfor, boardSize, squareSize, boardRegion);
	
	bool gotSize = false;
	bool extCalibrated = false;
	Size imageSize[NUM_SENSORS];
	Mat cameraMatrix[NUM_SENSORS];										// cameras' intrinsic matrices
	vector<Mat> rvecs[NUM_COLOR_SENSORS], tvecs[NUM_COLOR_SENSORS];		// board pose regarding 2 color cameras
	vector<vector<Point2f> > incorner2DPoints[NUM_COLOR_SENSORS];		// extracted inner corners
	vector<vector<Point2f> > boardCorner2DPoints;						// extracted board outer corners from depth images

	vector<float> reprojErrs[NUM_SENSORS];
	vector<Point3f> inCorners3D;											// 3D coordinates of inner corners in {B}
	Mat matInCorners3D;
	vector<Point3f> boardCorners3D;										// 3D coordinates of the board outer corners in {B}
	Mat matBoardCorners3D;
	vector<vector<short>> allBoardCornerDepth;
	vector<vector<Point3f>> newinCorners3D;

	// prepared reference 3-D coordinates
	calcInnerCorner(boardSize, squareSize, inCorners3D, matInCorners3D);
	calcOuterCorner(boardRegion, boardCorners3D,matBoardCorners3D);
	
	
	// Initiate camera matrices
	float _cam[9] = {fc, 0 , 0, 0, fc, 0, 0 , 0, 1};
	cameraMatrix[RGB_SENSOR] = Mat(3,3,CV_32F,_cam);
	float _extCam[9] = {0, 0 , 0, 0, 0, 0, 0 , 0, 1};
	cameraMatrix[EXT_SENSOR] = Mat(3,3,CV_32F,_extCam);	
	float _depthcam[9] = {fd, 0 , 0, 0, fd, 0, 0 , 0, 1};
	cameraMatrix[DEPTH_SENSOR] = Mat(3,3,CV_32F,_depthcam);
	// null distortation parameters
	Mat distCoeffs(1,4,CV_32F);
	distCoeffs = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////   STEP 1: Scan over images & collect useful information    ////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	char fpath[100];
	for (int frame=start;frame<=end;frame+=step){
		Mat depthMat, rgbMat, extMat;

		vector<Point2f> rgbPointBuf;			// temporary extracted 2-D inner corners
		vector<Point2f> extPointBuf;

		/// Load images & validate
		if (!loadImages(src, frame, depthMat, rgbMat, extMat)){
			continue;
		}

		if (!gotSize) {
			// get image sizes
			imageSize[RGB_SENSOR].height = rgbMat.rows;
			imageSize[RGB_SENSOR].width  = rgbMat.cols;
			imageSize[DEPTH_SENSOR].height = depthMat.rows;
		
			imageSize[DEPTH_SENSOR].width  = depthMat.cols;
			imageSize[EXT_SENSOR].height = extMat.rows;
			imageSize[EXT_SENSOR].width  = extMat.cols;

			// set cameras' principal points
			_cam[2] = imageSize[RGB_SENSOR].width/2;
			_cam[5] = imageSize[RGB_SENSOR].height/2;
			_extCam[2] = imageSize[EXT_SENSOR].width/2;
			_extCam[5] = imageSize[EXT_SENSOR].height/2;
			_depthcam[2] = imageSize[DEPTH_SENSOR].width/2;
			_depthcam[5] = imageSize[DEPTH_SENSOR].height/2;
			gotSize = true;
		}

		//////  Process the color images
		// TODO II.1.1: Extract inner corners of the checkerboard (extPointBuf) from the external image (extMat)
		//              If no corner is detected, skip this frame
		//			    If the detected corners are in a wrong order, revert it
		//
		// fill your code here
		//

		findChessboardCorners(extMat, boardSize, extPointBuf);
		 
	//	std::reverse(&extPointBuf[0], &extPointBuf[19]);

		if (extPointBuf.size() > 0)
			drawChessboardCorners( extMat, boardSize, Mat(extPointBuf), true );
			

		//resize extMat
		Mat extResized;
		resize(extMat,extResized,rgbMat.size());
		cv::namedWindow( "External image", 10 );
		cv::imshow( "External image", extResized );
		cv::waitKey( 20 );
		// TODO I.2.1: Extract inner corners of the checkerboard (extPointBuf) from the external image (extMat)
		//             If no corner is detected, skip this frame
		//			   If the detected corners are in a wrong order, revert it
		//
		// fill your code here
		//

		findChessboardCorners(rgbMat, boardSize, rgbPointBuf);

		//std::reverse(&rgbPointBuf[0], &rgbPointBuf[23]);
		if (rgbPointBuf.size() > 0)
			drawChessboardCorners( rgbMat, boardSize, Mat(rgbPointBuf), true );
		cv::namedWindow( "RGB image", 11 );
		cv::imshow( "RGB image", rgbMat );
		cv::waitKey( 20 );

		Mat rVec, tVec;
		// TODO I.2.2: From the detected 2D inner corners (rgbPointBuf) & their 3D coordinates in {B} (inCorners3D), estimate the board pose 
		//             (regarding the color sensor coordinate system {C}) using function solvePnPRansac() of OpenCV. 
		//			   Save your result into rVec and tVec for later use
		//
		// fill your code here
		//

		solvePnPRansac(inCorners3D, rgbPointBuf, cameraMatrix[RGB_SENSOR],distCoeffs,rVec,tVec);

		
		//Vec<float,3>rr(rVec);
		//Vec<float,3>tt(tVec);
		
		

		vector<Point2f> rgbPoints2;
		projectPoints(inCorners3D,rVec,tVec,cameraMatrix[RGB_SENSOR],distCoeffs,rgbPoints2);	// back-project inner corners into the RGB image
		for (int i=0;i<rgbPoints2.size();i++)
			circle(rgbMat,Point(floor(rgbPoints2[i].x + 0.5),floor(rgbPoints2[i].y + 0.5)),2,Scalar(128,128,0),2);
	    cv::imshow( "RGB image", rgbMat );
		cv::waitKey( 20 );

		rVec.convertTo(rVec, CV_32F);
		tVec.convertTo(tVec, CV_32F);

		//////  Process the depth image
		Mat depthMat0 = depthMat / 4;					// scale depth map
		depthMat0.convertTo(depthMat0, CV_8U);			// gray-scale depth image

		// duplicate channels
		Mat depthMat1(depthMat.rows,depthMat.cols,CV_8UC3);	// true-color depth image
		insertChannel(depthMat0,depthMat1,0);
		insertChannel(depthMat0,depthMat1,1);
		insertChannel(depthMat0,depthMat1,2);

		cv::namedWindow( "Depth image", 12 );		
		cv::imshow( "Depth image", depthMat1 );
		cv::waitKey( 20 );
		
		// plot locations of the inner corners (from the RGB image) onto the depth image
		for (int i=0;i<boardSize.height;i++){
			for (int j=0;j<boardSize.width;j++){
				circle(depthMat1,Point(floor(rgbPointBuf[i*boardSize.width +j].x),floor(rgbPointBuf[i*boardSize.width +j].y)),2,Scalar(0,0,255));
			}
		}
		cv::imshow( "Depth image", depthMat1 );
		cv::waitKey( 20 );
		
		Mat segmentedDepthMat;
		// TODO I.2.3.a: Detect the board region using a segmentation technique (thresholding, watershed, mean-shift ...). 
		//					Input: the gray-scale depth map (depthMat0)
		//					Output: a depth map with only the extracted board region (segmentedDepthMat)
		//               Note that you can get hints about the board position from the inner corners location in the color image.
		//
		// fill your code here
		//
	
	
	threshold(depthMat0, segmentedDepthMat, 180, 255, 4);
	cv::imshow("segmented depth mat", segmentedDepthMat);
	cv::waitKey( 20 );

	/*
	// Perform the distance transform algorithm
	cv::Mat dist;
	cv::distanceTransform(segmentedDepthMat, dist, CV_DIST_L2, 3);

	// Normalize the distance image for range = {0.0, 1.0}
	// so we can visualize and threshold it
	cv::normalize(dist, dist, 0, 1., cv::NORM_MINMAX);
	cv::imshow("dist", dist);
	cv::waitKey( 20 );

	// Threshold to obtain the peaks 
	// This will be the markers for the foreground objects
	cv::threshold(dist, dist, .5, 1., CV_THRESH_BINARY);
	cv::imshow("dist2", dist);
	cv::waitKey( 20 );

	// Create the CV_8U version of the distance image
	// It is needed for cv::findContours()
	cv::Mat dist_8u;
	dist.convertTo(dist_8u, CV_8U);

	// Find total markers
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	int ncomp = contours.size();

	// Create the marker image for the watershed algorithm
	cv::Mat markers = cv::Mat::zeros(dist.size(), CV_32SC1);

	// Draw the foreground markers
	for (int i = 0; i < ncomp; i++)
		cv::drawContours(markers, contours, i, cv::Scalar::all(i+1), -1);

	// Draw the background marker
	cv::circle(markers, cv::Point(5,5), 3, CV_RGB(255,255,255), -1);
	cv::imshow("markers", markers*10000);
	cv::waitKey(20);

	// Perform the watershed algorithm
	cv::watershed(depthMat1, markers);

	// Generate random colors
	std::vector<cv::Vec3b> colors;
	for (int i = 0; i < ncomp; i++)
	{
		int b = cv::theRNG().uniform(0, 255);
		int g = cv::theRNG().uniform(0, 255);
		int r = cv::theRNG().uniform(0, 255);

		colors.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
	}

	// Create the result image
	cv::Mat dst = cv::Mat::zeros(markers.size(), CV_8UC3);

	// Fill labeled objects with random colors
	for (int i = 0; i < markers.rows; i++)
	{
		for (int j = 0; j < markers.cols; j++)
		{
			int index = markers.at<int>(i,j);
			if (index > 0 && index <= ncomp)
				dst.at<cv::Vec3b>(i,j) = colors[index-1];
			else
				dst.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
		}
	}

	cv::imshow("dst", dst);
	cv::waitKey(20);


	*/
		vector<Vec4i> lines;
		// TODO I.2.3.b: From segmentedDepthMat, extract edges by Canny (or other) detector
		//				 Detect lines using HoughLinesP	
		//
		// fill your code here
		//
		cv::Mat cannyOut = cv::Mat::zeros(segmentedDepthMat.size(), CV_8UC3);
		 Canny(segmentedDepthMat, cannyOut, 10, 100, 3);
		 cv::imshow("dst", cannyOut);
	 	 cv::waitKey(20);

		
		//Mat cdst= cv::Mat::zeros(dst.size(), CV_32SC1);
  HoughLinesP(cannyOut, lines, 1, CV_PI/180, 30, 60, 25 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
	
	
    line( cannyOut, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(127,127,255), 10, 8);
  }

		cv::imshow("cdst", cannyOut);
	 	cv::waitKey(20);

		bool boardEdgeFound[4] = {0};
		Vec4i boardEdges[4];
		// TODO I.2.3.c: Use the known inner corners to filter the detected lines. Save them into vector boardEdges
		//
		// fill your code here
		//

		//Filtering the unwanted lines

		Vec2f tpl=rgbPointBuf[0];
		 Vec2f tpr=rgbPointBuf[4];
		 Vec2f btl=rgbPointBuf[15];
		 Vec2f btr=rgbPointBuf[19];
		 Vec4i tempLines[4]={0};
		 bool tFlag=false, bFlag=false, lFlag=false, rFlag=false;
		 int cter=0;

		 for (int i=0; i<lines.size(); i++)
		 {
			 if (lines[i][0]<=tpl[0] && lines[i][2]<=btl[0] && lFlag==false){
				 tempLines[0]=lines[i];
				 cter++;
				 lFlag=true;
			}
			 else if (lines[i][1]<=tpl[1] && lines[i][3]<=tpr[1] && tFlag==false){
				 tempLines[1]=lines[i];
				 cter++;
				 tFlag=true;
			 }
			 else if (lines[i][0]>tpl[0] && lines[i][2] > btr[0] && rFlag==false){
				 tempLines[2]=lines[i];
				 cter++;
				 rFlag=true;
			 }
			 else if (lines[i][1]>tpl[1] && lines[i][3]>tpr[1] && bFlag==false){
				 tempLines[3]=lines[i];
				 cter++;
				 bFlag=true;
			 }		 
		 }

		 lines.clear();
		 for(int i=0; i<4; i++){
			 if(tempLines[i][0]!=0)
		     lines.push_back(tempLines[i]);
		 }

		for(int i=0; i<lines.size(); i++){
			boardEdges[i]=lines[i];
			boardEdgeFound[i]=1;
		}


		bool allBorderFound = true;
		for (int i=0;i<4;i++){
			if (!boardEdgeFound[i]) {
				allBorderFound = false;
				break;
			}
			else
			{
				line(depthMat1,Point(boardEdges[i][0],boardEdges[i][1]),Point(boardEdges[i][2],boardEdges[i][3]),Scalar(0,0,255),2);
			}
		}
		if (!allBorderFound) continue;
		cv::imshow( "Depth image", depthMat1 );
		cv::waitKey( 20 );

		vector<Point2f> board2DCorners;
		// TODO I.2.3.d: Compute the board outer corners as the intersections of the board edges (in boardEdges)
		//				 Save them into board2DCorners
		//				 Make sure that your corners are sorted in the correct order regarding the computed 3-D coordinates in part I.1.b
		//
		// fill your code here

		

	
        
		for (int i = 0; i < lines.size(); i++)
		{
			for (int j=(i+1); j<lines.size(); j++){
				cv::Point2f pt = computeIntersect(lines[i], lines[j]);
				if (pt.x >= 0 && pt.y >= 0 && pt.x<400 && pt.y<400)
					board2DCorners.push_back(pt);
			}
		}

	/*	//sort the corners obtained according from top left to bottom right

		Point2f center(0,0);
		for(int i=0; i<board2DCorners.size(); i++){
			center = center + board2DCorners[i];
		}

		center*=1./board2DCorners.size();
		sortCorners(board2DCorners, center);
*/
		if (board2DCorners.size() < 4) continue;

			//sort the corners obtained according from top left to bottom right

		Point2f center(0,0);
		for(int i=0; i<board2DCorners.size(); i++){
			center = center + board2DCorners[i];
		}

		center*=1./board2DCorners.size();
		sortCorners(board2DCorners, center);

		cout<<"The board corners in 2D are: \n"<<endl;
		for(int iii=0; iii<board2DCorners.size(); iii++){
			for(int jjj=0; jjj<2; jjj++){
				cout<<Mat(board2DCorners).at<float>(iii,jjj) <<endl;
			}
			cout<<"\n";
		}
		
		cout<<"The board corners in 3D are: \n"<<endl;
		for(int iii=0; iii<boardCorners3D.size(); iii++){
			for(int jjj=0; jjj<2; jjj++){
				cout<<Mat(boardCorners3D).at<float>(iii,jjj) <<endl;
			}
			cout<<"\n";
		}
	
		// estimate the outer corners' depth
		vector<short> boardCornersDepth;
		for (int j=0;j<4;j++){
			Point2f dstPoint;
			if (!findClosestPoint(segmentedDepthMat, board2DCorners[j], dstPoint)) 
				break;
			boardCornersDepth.push_back(depthMat.at<short>(floor(dstPoint.y+0.5),floor(dstPoint.x+0.5)));
			circle(depthMat1,board2DCorners[j],2,Scalar(0,255,0),2);
			circle(depthMat1,dstPoint,2,Scalar(255,0,0),2);
		}
		if (boardCornersDepth.size() < 4) continue;
		cv::imshow( "Depth image", depthMat1 );
		cv::waitKey( 20 );

		// a good frame -> save all useful information
		goodFrames.push_back(frame);
		incorner2DPoints[RGB_SENSOR].push_back(rgbPointBuf);
		incorner2DPoints[EXT_SENSOR].push_back(extPointBuf);
		boardCorner2DPoints.push_back(board2DCorners);
		allBoardCornerDepth.push_back(boardCornersDepth);
		rvecs[RGB_SENSOR].push_back(rVec);
		tvecs[RGB_SENSOR].push_back(tVec);
		newinCorners3D.push_back(inCorners3D);
		

		// combine the processed images
		Mat tmpOut;
		hconcat(rgbMat,extResized,tmpOut);
		//
		Mat outView;
		hconcat(depthMat1,tmpOut,outView);
		cv::namedWindow( "Kinect Calibration", 1 );
		cv::imshow( "Kinect Calibration", outView );
		cv::waitKey( 100 );

		

		if (goodFrames.size() >= nFrame2Use){
			runCalibration(newinCorners3D, imageSize[EXT_SENSOR],  cameraMatrix[EXT_SENSOR], distCoeffs, incorner2DPoints[EXT_SENSOR],rvecs[EXT_SENSOR],tvecs[EXT_SENSOR]);
			extCalibrated = true;
			break;
		}
	}

	if (!extCalibrated) {
		runCalibration(newinCorners3D, imageSize[EXT_SENSOR],  cameraMatrix[EXT_SENSOR], distCoeffs, incorner2DPoints[EXT_SENSOR],rvecs[EXT_SENSOR],tvecs[EXT_SENSOR]);
	}
	
	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////             STEP 2: Calibrate the depth camera             ////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	double cx = imageSize[DEPTH_SENSOR].width/2;		// the depth sensor's principal point
	double cy = imageSize[DEPTH_SENSOR].height/2;
	
	// Compute translation t1 between the RGB & depth sensor
	Mat t1 = Mat::zeros(3,1,CV_32F);
	for (int i=0;i<goodFrames.size();i++){
		Mat matR;
		// TODO I.3: Compute translation t1 between the RGB & depth sensor
		//           (see project decription for more details)
		// Inputs:
		//	  - Rotation vector for RGB image (rvecs[RGB_SENSOR][i])
		//	  - Translation vector for RGB image (tvecs[RGB_SENSOR][i])
		//    - 3-D coordinates of the board corners in {B} (matBoardCorners3D)
		//    - Detected 2-D board corners in the depth image (boardCorner2DPoints)
		//    - Board corners' depth in {D} (allBoardCornerDepth)
		// Task:
		//	  update t1 according to equation (6)
		//
		// fill your code here
		//
		Vector<Point3f> PDm;

		Rodrigues(rvecs[RGB_SENSOR][i], matR); 

		for(int j=0; j<4; j++){
			PDm.push_back(cv::Vec3f((boardCorner2DPoints[i][j].x-cx)/fd*allBoardCornerDepth[i][j], (boardCorner2DPoints[i][j].y-cy)/fd*allBoardCornerDepth[i][j], allBoardCornerDepth[i][j]));
		}
	
		
	
		Mat tempV=Mat::zeros(3,1,CV_32F);
        
		
		

		for(int j=0; j<4; j++){
			tempV=tempV+(Mat(PDm[j])-matR*Mat(boardCorners3D[j])-Mat(tvecs[RGB_SENSOR][i]));
			
		}

		t1=t1+tempV;
	}
		
	t1 = t1/(4*goodFrames.size());

	for(int p=0; p<t1.rows; p++){
			cout<<t1.at<float>(p)<<"\n";
		}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////             STEP 3: Calibrate the external camera          ////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	Mat r2;
	Mat t2;
	vector<Point3f> point3D;
	vector<Point2f> point2D;
	for (int i=0;i<goodFrames.size();i++){
		// TODO II.1.3: collect the inner corners' 3-D coordinates in {D} and their projection in {E}
		//              (see project decription for more details)
		// Inputs:
		//	  - Rotation vector for RGB image (rvecs[RGB_SENSOR][i])
		//	  - Translation vector for RGB image (tvecs[RGB_SENSOR][i])
		//	  - Translation vector between the RGB & the depth sensor (t1)
		//    - Detected inner corners in the external image (incorner2DPoints[EXT_SENSOR])
		// Task:
		//	  update point3D & point2D
		//
		// fill your code here
		//
		Mat matR;
		Rodrigues(rvecs[RGB_SENSOR][i], matR); 
		

		 for(int j=0; j<20; j++){
			 point2D.push_back(incorner2DPoints[EXT_SENSOR][i][j]);
		 }

		Mat tempP3D;
		for(int j=0; j<20; j++){
			tempP3D=matR*Mat(newinCorners3D[i][j])+Mat(tvecs[RGB_SENSOR][i])+t1;
			point3D.push_back(Point3f(tempP3D));
		}
	

	}
	// TODO II.1.4: use solvePnPRansac to compute r2 & t2
	//
	// fill your code here
	//

	solvePnPRansac(point3D, point2D, cameraMatrix[EXT_SENSOR], distCoeffs, r2, t2); 
	
	cout<<"t2 values are"<<endl;
	for(int p=0; p<t2.rows; p++){
			cout<<t2.at<double>(p)<<"\n";
		}

	cout<<"r2 values are"<<endl;
	for(int p=0; p<r2.rows; p++){
			cout<<r2.at<double>(p)<<"\n";
		}

	////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////         STEP 4: Paint depth images by RGB & external images        ////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i=0;i<goodFrames.size();i++){
		int frame = goodFrames.at(i);
		Mat depthMat, rgbMat0, extMat0, rgbMat, extMat;

		if (!loadImages(src, frame, depthMat, rgbMat0, extMat0)){
			continue;
		}

		// comput 3D coordinates of every pixel in the depth map in {D}
		vector<Point3f> _3DPoints;
		vector<int> pSet, qSet;							// saved row & col indices of _3DPoints
		for (int p=0;p<depthMat.rows;p++){				// row index
			for (int q=0;q<depthMat.cols;q++){			// col index
				if (depthMat.at<short>(p,q) != 0){
					float X, Y, Z;
					// TODO I.4.a: compute 3D coordinate (X,Y,Z) of each pixel in the depth image according to {D}
					//
					// fill your code here
					//
					X=(q-cx)*depthMat.at<short>(p,q)/fd;
					Y=(p-cy)*depthMat.at<short>(p,q)/fd;
					Z=depthMat.at<short>(p,q);
					_3DPoints.push_back(Point3f(X,Y,Z));
					pSet.push_back(p);
					qSet.push_back(q);
				}
			}
		}

		////////   From the RGB image
		printf("Projecting the RGB image...\n");
		vector<Point2f> projectedPoints;
		Mat outTest = Mat::zeros(rgbMat0.size(),rgbMat0.type());
		int pointCount = 0;
		// TODO I.4.b: 
		//     - Project these 3D points (_3DPoints) onto the color image using transformation [I, -t1]
		//	   - If the projected point is inside the RGB image, pick its color & paint the corresponding point 
		//       (see pSet & qSet) in the output image (outTest)
		//       You may want to use avSubPixelValue8U3 to get a smoother result (optional)
		//	   - Count the number of the ploted points
		//
		// fill your code here
		//
		
		

	/*	 for(int i=0; i<_3DPoints.size(); i++){
			projectedPoints.push_back(Point2f(_3DPoints[i].x-t1.at<float>(0), _3DPoints[i].y-t1.at<float>(1)));
		 }
		 */
		Mat rrvec = Mat::zeros(3,1,CV_32F);
		projectPoints(_3DPoints, rrvec, -t1, cameraMatrix[RGB_SENSOR], distCoeffs, projectedPoints);  
		 
		 for(int i=0; i<projectedPoints.size(); i++){
			 if (projectedPoints[i].x>=0 && projectedPoints[i].x<rgbMat0.cols && projectedPoints[i].y>=0 && projectedPoints[i].y<rgbMat0.rows){
				 outTest.at<Vec3b>(pSet[i], qSet[i])=avSubPixelValue8U3(Point2f((projectedPoints[i].x),(projectedPoints[i].y)),rgbMat0);
			     pointCount++;
			 }
		 }
		// save the output image
		sprintf(fpath,"%s/rgb_%04d.png",dst,frame);
		imwrite(fpath, outTest);
		// save the output point cloud
		sprintf(fpath,"%s/rgb_%04d.ply",dst,frame);
		writePLY(pointCount,fd,outTest,depthMat,fpath);

		
		////////   From the external image
		printf("Projecting the external image...\n");

		vector<Point2f> projectedPoints2;
		vector<Point2f>_3DPoints2;
		Mat outTest2 = Mat::zeros(rgbMat0.size(),rgbMat0.type());
		Mat extDepth = Mat::ones(extMat0.rows/4,extMat0.cols/4,CV_32F) * 1E+10;
		int pointCount2 = 0;
		Mat outDepth = Mat::zeros(depthMat.size(),depthMat.type());		// save depth value of ploted points
																		// used when writing the point cloud
		// TODO II.2: 
		//     - Project these 3D points (_3DPoints) onto the external image using transformation [r2, t2]
		//	   - Update the Z-buffer (extDepth).
		//     - Project these 3D points (_3DPoints) again onto the external image. 
		//		 If a 3D point has depth < 120% the stored depth in Z-buffer:
		//			+ Pick color & paint the corresponding point (see pSet & qSet) in the output image (outTest2)
		//			  You may want to use avSubPixelValue8U3 to get a smoother result (optional)
		//			+ Set its depth in outDepth
		//			+ Count the number of the ploted points
		//       Otherwise, skip the point
		//
		// fill your code here
		//

		projectPoints(_3DPoints, r2, t2, cameraMatrix[EXT_SENSOR], distCoeffs, projectedPoints2);
		int zrows, zcols;


		for(int ii=0; ii<pSet.size(); ii++){
			zrows=floor(projectedPoints2[ii].y/4);
			zcols=floor(projectedPoints2[ii].x/4);

			if(zrows>0 && zrows<extDepth.rows && zcols >0 && zcols<extDepth.cols){

			if (extDepth.at<float>(zrows, zcols)>_3DPoints[ii].z){
				extDepth.at<float>(zrows, zcols)=_3DPoints[ii].z;
			}
			}
		}


		for(int ii=0; ii<pSet.size(); ii++){
			zrows=floor(projectedPoints2[ii].y/4);
			zcols=floor(projectedPoints2[ii].x/4);

			if(zrows>0 && zrows<extDepth.rows && zcols >0 && zcols<extDepth.cols){

			if (1.2*(extDepth.at<float>(zrows, zcols))>_3DPoints[ii].z){
				outTest2.at<Vec3b>(pSet[ii], qSet[ii])=avSubPixelValue8U3(Point2f(floor(projectedPoints2[ii].x),floor(projectedPoints2[ii].y)),extMat0);
				outDepth.at<short>(pSet[ii],qSet[ii])=_3DPoints[ii].z;
				pointCount2++;
			}
			}
		}

		

		

		// save the output image
		sprintf(fpath,"%s/ext_%04d.png",dst,frame);
		imwrite(fpath,outTest2);
		// save the output point cloud
		sprintf(fpath,"%s/ext_%04d.ply",dst,frame);
		writePLY(pointCount2,fd,outTest2,outDepth,fpath);
		
		// output the depth map
		Mat depthMat0 = depthMat / 4;
		depthMat0.convertTo(depthMat0, CV_8U);

		Mat depthMat1(depthMat.rows,depthMat.cols,CV_8UC3);
		insertChannel(depthMat0,depthMat1,0);
		insertChannel(depthMat0,depthMat1,1);
		insertChannel(depthMat0,depthMat1,2);

		// combine output & display
		Mat tmpOut,outfinal;
		hconcat(outTest,outTest2,tmpOut);
		hconcat(depthMat1,tmpOut,outfinal);

		cv::imshow( "Kinect Calibration", outfinal );
		cv::waitKey( 50 );
	}
	return 0;
}