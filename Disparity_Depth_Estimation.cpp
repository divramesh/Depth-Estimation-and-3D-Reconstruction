#include <stdio.h>
#include <string.h>
#include <math.h>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\opencv.hpp>
#include<opencv2\core\core.hpp>

using namespace cv;

int EstimateDisparity(Mat ir, Mat pattern, Mat &disparity, int min = -7, int max = 80, int S = 8);
int EstimateDepth(Mat disparity, Mat &depth, float z0, float b, float f);
int normalizeDepth(Mat depth, Mat &normalizedDepth);

int main(int argc, char* argv[])
{
	char irFile[80] = "ir.png";
	char patternFile[80] = "pattern.png";
	float z0 = 2000;
	float b = 80;
	float f = 200;

	int arg = 0;
	
	while( ++arg < argc) 
	{ 
		// IR image path+
		if( !strcmp(argv[arg], "-i") )
			strcpy(irFile, argv[++arg] );

		// Pattern image path
		if( !strcmp(argv[arg], "-p") )
			strcpy(patternFile, argv[++arg] );

		// Reference depth
		if( !strcmp(argv[arg], "-z0") )
			z0 = atof( argv[++arg] );
		
		// Baseline
		if( !strcmp(argv[arg], "-b") )
			b = atof( argv[++arg] );

		// Focal length
		if( !strcmp(argv[arg], "-f") )
			f = atof( argv[++arg] );
	}

	Mat disparity, depth, normalizedDepth, ir, pattern;
	Mat irBGR = imread(irFile);
	cvtColor(irBGR, ir, CV_BGR2GRAY);
	Mat patternBGR = imread(patternFile);
	cvtColor(patternBGR, pattern, CV_BGR2GRAY);

	if (EstimateDisparity(ir, pattern, disparity) != 0){
		fprintf(stderr,"Error when computing disparity map!\n");
		return -1;
	}

	cvSaveImage("disparity.png", &(IplImage(disparity)));

	if (EstimateDepth(disparity, depth, z0, b, f) != 0){
		fprintf(stderr,"Error when computing depth map!\n");
		return -1;
	}
	printf("%d %d %d\n",depth.rows,depth.cols,depth.type());
	depth = depth/10; // CHANGE: Divide depth by 10 to get a clearer output. Depth is computed in centimeter now
	cvSaveImage("depth.png", &(IplImage(depth)));
	normalizeDepth(depth, normalizedDepth);
	printf("%d %d %d\n",normalizedDepth.rows,normalizedDepth.cols,normalizedDepth.type());
	//imwrite("normalizedDepth.png", normalizedDepth);
	cvSaveImage("normalizedDepth.png", &(IplImage(normalizedDepth)));

	getchar();
	return 0;
}

/**
 *	EstimateDisparity:
 *		Input:
 *			ir			- IR image data
 *			pattern		- Reference image data
 *			numIter		- number of iterations for Ransac
 *			min			- Minimum disparity value
 *			max			- Maximum disparity value
 *			S			- Sub pixel accuracy
 *
 *		Output:
 *			disparity	- disparity data
 */
int EstimateDisparity(Mat ir, Mat pattern, Mat &disparity, int min, int max, int S){
	int windowSize = 9;
	int margin = (windowSize-1)/2;

	int h = ir.rows;
	int w = ir.cols;
	printf("IR image size: %d %d\n",h,w);
	disparity = Mat::ones(h-windowSize+1,w-windowSize+1, CV_32F) * 100;
	
	Mat* patternList = new Mat[S];
	Mat patternShift(h, w, CV_8U);
	float SAD;
	// Step 1: create a list of reference images with subpixel disparities: {0, 1/S, 2/S, ..., (S-1)/S}
	// Hint:  shift right the pattern 1 pixel to have patternShift
	//        patternList[i] = ((S-i) * pattern + i * patternShift)/S 
	//
	// TODO 1: fill your code here
	//
	
	for (int i=0; i<h; i++){
		for (int j=0; j<w-1; j++){
			patternShift.at<unsigned char>(i,j)=pattern.at<unsigned char>(i,j+1);
		}
	}
	
	int j=w-1;
	for(int i=0; i<h; i++){
		patternShift.at<unsigned char>(i,j)=0;
	}
	
	
	for(int i=0; i<8; i++){
		patternList[i] = ((S-i) * pattern + i * patternShift)/S;
	}
	
	char fname[200];
	for (int i=0; i<S; i++){
		sprintf(fname,"pattern%d.png",i);
		cvSaveImage(fname, &(IplImage(patternList[i])));
	}

	//  Step 2: scanline algorithm
	for (int i=margin;i<h-margin;i++) {
		for (int j=margin;j<w-margin;j++) {
			printf("Check pixel %d %d\n",i,j);

			float bestDisparityInt = -1;		//integer part
			float bestDisparityFrac = -1;		//fractional part
			float minDiff = 10000000;
			int start = ((j+min)>=margin) ? j+min : margin;
			int end = ((j+max)>w-margin-1) ? w-margin-1: (j+max);
			
			// For each subpixel value
			for (int sub = 0; sub < S; sub++){	
				// - For each pixel, do template matching
				// - Save the best disparity value to the corresponding position in the disparity matrix
				//
				// TODO 2: fill your code here
				//
				for ( int k=start; k<=end; k++){
					float SAD=0.0;
					for (int p=-margin; p<=margin; p++){
						for (int q=-margin; q<=margin; q++){
								SAD=SAD+abs(ir.at<unsigned char>(i+p, j+q)-patternList[sub].at<unsigned char>(i+p,k+q));				
						}
					}
					if (SAD<=minDiff){
						minDiff=SAD;
						bestDisparityInt=k-j;
						bestDisparityFrac=sub;
					}
				}
			}
			disparity.at<float>(i-margin,j-margin) = bestDisparityInt + bestDisparityFrac/S;
		}
	}
	return 0;
}


/**
 *	EstimateDepth:
 *		Input:
 *			disparity	- disparity data
 *			z0			- Reference depth (mm)
 *			b			- baseline (mm)
 *			f			- focal length (mm)
 *
 *		Ouput:
 *			depth		- disparity data
 */
int EstimateDepth(Mat disparity, Mat &depth, float z0, float b, float f){
	//
	// TODO 3: fill your code here
	//

	Mat value(disparity.rows, disparity.cols, CV_32F);
	float minDepth=10000000;
	float maxDepth=0;


	for(int i=0; i<disparity.rows; i++){
		for(int j=0; j<disparity.cols; j++){
			value.at<float>(i,j)=z0/(1+z0*disparity.at<float>(i,j)/(f*b));
			if (value.at<float>(i,j)<minDepth){
				minDepth=value.at<float>(i,j);
			}

			if (value.at<float>(i,j)>maxDepth){
				maxDepth=value.at<float>(i,j);
			}
		}
	}
	depth=value;
	printf("The range of depth values is from %f to %f\n", minDepth, maxDepth);
	return 0;
}

int normalizeDepth(Mat depth, Mat &normalizedDepth){
	MatIterator_<float> mn = std::min_element(depth.begin<float>(), depth.end<float>());
	MatIterator_<float> mx = std::max_element(depth.begin<float>(), depth.end<float>());
	float scale = 255.0f / (*mx - *mn);
	normalizedDepth = (depth - *mn) * scale;
	return 1;
}