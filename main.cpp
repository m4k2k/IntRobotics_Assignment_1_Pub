/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
#################################################################################################################################################




													Circle Detection for Icub Robot

Software Flow (simplified):

Loop:
Connect-to-Left-Eye-Cam -> Acquire-Image -> Convert-Image ->
-> Filter-Image -> Detect-Hough-Circles -> Paint-the-circles -> Send-Filtered-Image-Back ->
-> Check-iKinGaze-Gaze-Position -> Paint-where-the-robot-looks -> Send-Detected-Circles-X-Y-to-iKinGaze ->



#################################################################################################################################################
*/


// ######################################
// Includes, Name-spaces and Definitions
// ######################################


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv/cv.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

// windows system error codes
// https://msdn.microsoft.com/de-de/library/windows/desktop/ms681382(v=vs.85).aspx

#define ERROR_SUCCESS 0																		// The operation completed successfully.
#define ERROR_INVALID_FUNCTION 1															// Incorrect function.
#define ERROR_BAD_ENVIRONMENT 10															// The environment is incorrect.
#define ERROR_DEV_NOT_EXIST 55																// The specified network resource or device is no longer available.

using namespace yarp::os;
using namespace yarp::sig;
using namespace cv;

// ############################################
// Forward Declaration Functions and Procedures
// ############################################


Mat ConvertImage(ImageOf<PixelBgr>* input);

ImageOf<PixelBgr> ConvertImage(Mat& Input);

Mat FilterImage(Mat& Input);

void SendImage(Mat& Output);

int MainLoop();

int LookAt(Point& center, Mat& CircleMat);


// ################
// Global Variables
// ################


Network yarpPort;																			// set up yarp

BufferedPort<ImageOf<PixelBgr> > imageInPort, imageOutPort;									// create a port for reading images

BufferedPort<Bottle> iKinInPort, iKinOutPort;												// create ports for iKinGaze communication

bool demo = false;																			// is used for demo mode - stepwise program running


int main(int argc, char** argv)
{

	/*
	######################################################################################
	############################## GENERAL PORT CREATION #################################
	######################################################################################
	*/


	bool imgIn = imageInPort.open("/home/image/in");										// open the ports and save the return value
	bool imgOut = imageOutPort.open("/home/image/out");										// ^^
	bool iKinIn = iKinInPort.open("/home/iKinIn");											// ^^
	bool iKinOut = iKinOutPort.open("/home/iKinOut");										// ^^

	waitKey(50);																			// give the connections some time to be established

	if (!iKinIn || !iKinOut || !imgIn || !imgOut) {											// Error handling in case a connection couldn't be established
		printf("Failed to create ports.\n");												// notify the user
		printf("Maybe you need to start a nameserver (run 'yarpserver' \n");
		printf("Press Return to Exit\n");
		std::cin.get();																		// Wait for the Return Key
		return ERROR_DEV_NOT_EXIST;															// give an appropriate error code
	}

	bool yc1 = yarpPort.connect("/icubSim/cam/left", "/home/image/in");						// connect the ports and save the return value
	bool yc2 = yarpPort.connect("/iKinGazeCtrl/x:o", iKinInPort.getName());					// ^^
	bool yc3 = yarpPort.connect(iKinOutPort.getName(), "/iKinGazeCtrl/xd:i");				// ^^
	yarpPort.connect(imageOutPort.getName(), "/view/ase");									// we connect the YarpView -> but in case its not yet open, we don't care

	waitKey(250);																			// give the connections some time to be established

	if (!yc1 || !yc2 || !yc3) {																// Error handling in case a connection couldn't be established
		printf("Failed to connect the ports.\n");											// notify the user
		printf("Maybe you need to start a nameserver (run 'yarpserver' \n");
		printf("Press Return to Exit\n");
		std::cin.get();																		// Wait for the Return Key
		return ERROR_DEV_NOT_EXIST;															// give an appropriate error code
	}

	/*
	######################################################################################
	#################################### MAIN LOOP #######################################
	######################################################################################
	*/

	for (;;)//(int i = 0;i < 500;i++)
	{
		if (int return_val = MainLoop())													// Running the inner while encapsulated, helps managing the memory
			return return_val;																// catching possible error messages
	}

	// #################
	// hypothetical exit (for using an ending loop)
	// #################


	yarpPort.disconnect("/icubSim/cam/left", "/home/image/in");								// we disconnect all connections to ensure a nice application exit
	yarpPort.disconnect("/iKinGazeCtrl/x:o", iKinInPort.getName());
	yarpPort.disconnect(iKinOutPort.getName(), "/iKinGazeCtrl/xd:i");

	std::cin.get();																			// if we ever get here, wait for user input
	return ERROR_SUCCESS;
}


/*
MainLoop()

This runs our main loop, we encapsulate it for memory reasons
This starts and runs all actions inside the For() loop
*/
int MainLoop()																				// returning a number because we want the return code to be nice		
{
	// ###########################
	// Image Import and Converting
	// ###########################

	ImageOf<PixelBgr> *image = imageInPort.read();											// read the image from the iKinGaze bottle

	if (image != NULL)																		// check if we actually got something
	{
		Mat src = ConvertImage(image);														// convert our imported picture to the Mat format
		Mat mod = FilterImage(src);															// filter our input image

		vector<Vec3f> circles;																// create a variable to save detected circles


		// ###############################
		// Detect circles and Send Picture
		// ###############################

																							// apply hough transform filter
		HoughCircles(mod.clone(), circles, CV_HOUGH_GRADIENT, 1, 20, 100, 50, 40, 50);		// now lets apply hough transform - for parameter rational see supplied documentation

		cvtColor(mod.clone(), mod, CV_GRAY2BGR);											// prepare picture for sending

		SendImage(mod);																		// sending the Image to YarpView


		// #########################
		// Draw the circles detected
		// #########################

		Mat circ = src;																		// we copy the pointer of src to circ (not a deep copy - we loose the src)
		Point center;																		// carries the center point of the last circle detected
		for (size_t s = 0; s < circles.size(); s++)											// center detection and circle paint
		{
			center = Point(cvRound(circles[s][0]), cvRound(circles[s][1]));					// calculate the center point of the circle
			int radius = cvRound(circles[s][2]);											// calculate the radius of the circle
			circle(circ, center, 3, Scalar(255, 0, 0), -1, 8, 0);							// paint the circle center     
			circle(circ, center, radius, Scalar(255, 0, 0), 3, 8, 0);						// paint the circle outline
		}


		if (int return_val = LookAt(center, circ))
			return return_val;


		//imshow("Original Image", src);													// shows the original image (only possible of a deep copy has been created)
		imshow("Smoothed Image", mod);														// shows the filtered image
		imshow("Circle Detection", circ);													// shows the original image with detected circle
		waitKey(5);																			// 5 ms time day to help the display actually view something
	}
	else
	{
		printf("Can't get an Image! Please check the Simulator and the yarpserver!\n");
		printf("Press Return to Exit\n");
		std::cin.get();
		return ERROR_DEV_NOT_EXIST;
	}

	return ERROR_SUCCESS;

}


/*
int LookAt(Pointer to an Point where the robot should look, Pointer to the Mat where we can paint in)

1. Reading the Head Position
2. Point a dot where the Robot looks
3. if oldpoint <-> newpoint -> send head command
*/

int LookAt(Point& center, Mat& CircleMat)													// returning a number because we want the return code to be nice
{
	Bottle *RiKinIn = iKinInPort.read();													// At first we read the Input Bottle (it will be a vector of doubles)
	if (RiKinIn != NULL)																	// Check if we got something
	{
		// ##########################
		// Current Focus of Attention
		// ##########################

		double z = RiKinIn->get(0).asDouble();												// read the coordinates where the robot currently looks
		double x = RiKinIn->get(1).asDouble();
		double y = RiKinIn->get(2).asDouble();

		circle(CircleMat, Point((160.0 + 640.0 * x), (360.0 - 600.0 * y)), 3, Scalar(0, 0, 255), 3, 8, 0);	// Painting a red dot where the robot looks (and converting to the move_head reference)

		printf("Looking at: %.2f - %.2f \n", x, y);											// printing where the robot looks

		Bottle &RiKinOut = iKinOutPort.prepare();											// prepare sending to the robot (and get a reference to the bottle)
		RiKinOut.clear();																	// clear the cache

		// ######################
		// New Focus of Attention
		// ######################

		double NewX = (static_cast<double>(center.x) - 160.0) / 640.0;						// convert where the Robot should look into the move_head reference
		double NewY = (static_cast<double>(center.y) - 360.0) / -600.0;
		double NewZ = -3.0;

		if ((abs(NewX - x) > 0.02) || (abs(NewY - y) > 0.02))								// We move the head only if the difference between the point where the robot currently looks
		{																					// and where we want the robot to look is big enough (this ensures not flooding the controller)
			RiKinOut.addDouble(NewZ);														// Add our coordinates to the bottle
			RiKinOut.addDouble(NewX);
			RiKinOut.addDouble(NewY);
			iKinOutPort.writeStrict();														// writing to the output port and wait until writing is done
			printf("Look at: %.2f(%i) - %.2f(%i)\n", NewX, center.x, NewY, center.y);		// inform the user were we want the robot to look at
		}

	}
	else
	{																						// if our bottle is empty :( then we should deal with it
		printf("Error in the iKinGazeCtrl Interface! Please restart the program and the iKinGazeCtrl Interface.\n");
		printf("Press Return to Exit\n");
		std::cin.get();
		return ERROR_DEV_NOT_EXIST;
	}
	return ERROR_SUCCESS;
}


/*
Mat FilterImage(Pointer to the Input Mat we want to filter)

Filters and prepares the image for circle detection
*/

Mat FilterImage(Mat& input)
{

	Mat mgray, mbw, mblur, mgaus, out, cvtgray;												// declaration of Mats we use

	// apply gray-scale filter
	cvtColor(input.clone(), cvtgray, CV_BGR2GRAY);											// we use the default gray-scale filter of opencv (just transfer the color space)

	// apply a black-white filter																				
	// mbw = cvtgray > 128;																	// alternative, more simple way to apply threshold
	threshold(cvtgray, mbw, 127.0, 255.0, THRESH_BINARY);									// apply black-white filter using a threshold

	// apply homogeneous smoothing (aka box filter)
	blur(mbw, mblur, Size(22, 22));															// this makes the circle very round (good for hough T), but with very transparent borders (bad for hough transform)

	


	// apply Gaussian filter
	GaussianBlur(mblur, mgaus, Size(9, 9), 2, 2);											// use a Gaussian filter to create nice crisp borders

	if (demo)																				//  is created to allow a 'demo mode'
	{
		imshow("Original", input);
		imshow("Gray Scale Preview", cvtgray);
		imshow("BW Threshold Preview", mbw);
		imshow("Homog. Blur Preview", mblur);
		imshow("Gaussian Blur Preview", mgaus);
		waitKey();
	}

	return mgaus;
}


/*
SendImage(address to input image)

This send a Mat to the Yarpview - waits until sending is finished
*/
void SendImage(Mat& input)
{
	ImageOf<PixelBgr>& outp = imageOutPort.prepare();										// we prepare our output image and get a pointer to an image object
	outp.wrapIplImage(ConvertImage(input).getIplImage());									// At first we convert the image, the we getting an ipl image out of this and wrapping this into our output image
	imageOutPort.writeStrict();																// now we send the image, AND wait until its sent
}


/*
OutputImageFormat ConvertImage(address to input image)

converts an Mat image to an pixel-bgr image
*/
ImageOf<PixelBgr> ConvertImage(Mat& input)
{
	IplImage ipback = static_cast<IplImage>(input);											// At first we casting the Mat to an IPL image
	ImageOf<PixelBgr> back;
	back.wrapIplImage(&ipback);																// then we are wrapping the ipl image into our Output format
	return back;
}

/*
OutputImageFormat ConvertImage(address to input image)

converts an pixel-bgr image to an Mat Image
*/
Mat ConvertImage(ImageOf<PixelBgr>* input)
{
	return Mat(static_cast<IplImage*>(input->getIplImage()));								// get an ipl image from the input (by casting an void*), then converting it to an Mat 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////