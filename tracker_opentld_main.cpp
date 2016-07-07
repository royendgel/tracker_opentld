

#include <stdio.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>

#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/bumblebee_basic_messages.h>

#include <carmen/visual_tracker_interface.h>
#include <carmen/visual_tracker_messages.h>

#include <carmen/rotation_geometry.h>
#include <carmen/fused_odometry_messages.h>
#include <carmen/fused_odometry_interface.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <carmen/fused_odometry_interface.h>

//OPENTLD
#include "TLD.h"
#include "Trajectory.h"
#include "Gui.h"

using namespace cv;
//using namespace std;


#define BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH 640
#define BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT 480
#define BUMBLEBEE_BASIC_VIEW_NUM_COLORS 3

static int received_image = 0;

static int tld_image_width = 0;
static int tld_image_height = 0;

static int camera_side = 0;

static carmen_bumblebee_basic_stereoimage_message last_message;

static int msg_fps = 0, msg_last_fps = 0; //message fps
static int disp_fps = 0, disp_last_fps = 0; //display fps

static carmen_visual_tracker_output_message message_output;

enum Retval
{
    PROGRAM_EXIT = 0,
    SUCCESS = 1
};

tld::TLD *g_tld_track;
tld::Gui *g_gui;
//TLD dowork things
bool showOutput;
bool showTrajectory;
int trajectoryLength;
const char *printResults = NULL;
double threshold_tld;
bool showForeground = false;
bool showNotConfident = true;
bool selectManually = 0;
int *initialBB;
bool reinit = 0;
bool exportModelAfterRun = false;
bool loadModel = false;
const char *modelPath = NULL;
const char *modelExportFile = "model";
int seed = 0;
static int current_frame = 0;
const char *saveDir = ".";
tld::Trajectory trajectory;
bool skipProcessingOnce = false;
//-------------------

//-------------------Configuracao TLD------------------------------



//----------------------------------------------------


//list
//static vector<carmen_localize_ackerman_globalpos_message> lista_poses;
static vector<carmen_fused_odometry_message> lista_poses;
static vector<carmen_bumblebee_basic_stereoimage_message> lista_imagens;

//values to draw annotation

struct annotation
{
    double x;
    double y;
    double x1;
    double y1;
    double x2;
    double y2;
};
std::vector<annotation> annotation_point;
std::vector<carmen_rddf_annotation_message> annotations;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_visual_tracker_output_message()
{
	IPC_RETURN_TYPE err;
	err = IPC_publishData(CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_NAME, &message_output);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_NAME);
}

void
carmen_visual_tracker_define_messages()
{
	carmen_visual_tracker_define_message_output();
}

///////////////////////////////////////////////////////////////////////////////////////////////

void inicialize_TLD_parameters()
{
//main
	tld::DetectorCascade *detectorCascade = g_tld_track->detectorCascade;
	g_tld_track->trackerEnabled = true;
	showOutput = true;
	showTrajectory = true;
	trajectoryLength = 10;
	threshold_tld = 0.7;
	showForeground = false;
	showNotConfident = true;
	g_tld_track->alternating = false;
	g_tld_track->learningEnabled = true;
	selectManually = false;
	exportModelAfterRun = false;
	modelExportFile = "model";
	loadModel = false;
	seed = 0;
	srand(seed);
	//	Detector
	detectorCascade->varianceFilter->enabled = true;
	detectorCascade->ensembleClassifier->enabled = true;
	detectorCascade->nnClassifier->enabled = true;

	// classifier
	detectorCascade->useShift = true;
	detectorCascade->shift = 0.1;
	detectorCascade->minScale = -10;
	detectorCascade->maxScale = 10;
	detectorCascade->minSize = 25;
	detectorCascade->numTrees = 10;
	detectorCascade->numFeatures = 13;
	detectorCascade->nnClassifier->thetaTP = 0.65;
	detectorCascade->nnClassifier->thetaFP = 0.5;
	//dowork initial
	if (showTrajectory)
	{
		trajectory.init(trajectoryLength);
	}
	g_gui->init();
}
//verificar isso ai pra publicar
//if(tld->currBB != NULL)
//{
//	fprintf(resultsFile, "%d %.2d %.2d %.2d %.2d %f\n", imAcq->currentFrame - 1, tld->currBB->x, tld->currBB->y, tld->currBB->width, tld->currBB->height, tld->currConf);
//}
//else
//{
//	fprintf(resultsFile, "%d NaN NaN NaN NaN NaN\n", imAcq->currentFrame - 1);
//}

void
process_TLD_detection(IplImage * img, double time_stamp)
{
	current_frame++;
	Mat grey(img->height, img->width, CV_8UC1);
	cvtColor(cvarrToMat(img), grey, CV_BGR2GRAY);

	g_tld_track->detectorCascade->imgWidth = grey.cols;
	g_tld_track->detectorCascade->imgHeight = grey.rows;
	g_tld_track->detectorCascade->imgWidthStep = grey.step;

	if(!skipProcessingOnce)
	{
		g_tld_track->processImage(cvarrToMat(img));
	}
	else
	{
		skipProcessingOnce = false;
	}
	int confident = (g_tld_track->currConf >= threshold_tld) ? 1 : 0;

	if(showOutput || saveDir != NULL)
	{
		char string1[128];

		char learningString[10] = "";

		if(g_tld_track->learning)
		{
			strcpy(learningString, "Learning");
		}

		sprintf(string1, "Time Stamp:%.2f,Confidence:%.2f, fps:%d, #numwindows:%d, %s", time_stamp,
				g_tld_track->currConf, disp_last_fps, g_tld_track->detectorCascade->numWindows, learningString);

		CvScalar yellow = CV_RGB(255, 255, 0);
		CvScalar blue = CV_RGB(0, 0, 255);
		CvScalar black = CV_RGB(0, 0, 0);
		CvScalar white = CV_RGB(255, 255, 255);

		if(g_tld_track->currBB != NULL)
		{
			CvScalar rectangleColor = (confident) ? blue : yellow;
			cvRectangle(img, g_tld_track->currBB->tl(), g_tld_track->currBB->br(), rectangleColor, 8, 8, 0);

			if(showTrajectory)
			{
				CvPoint center = cvPoint(g_tld_track->currBB->x+g_tld_track->currBB->width/2, g_tld_track->currBB->y+g_tld_track->currBB->height/2);
				cvLine(img, cvPoint(center.x-2, center.y-2), cvPoint(center.x+2, center.y+2), rectangleColor, 2);
				cvLine(img, cvPoint(center.x-2, center.y+2), cvPoint(center.x+2, center.y-2), rectangleColor, 2);
				trajectory.addPoint(center, rectangleColor);
			}
		}
		else if(showTrajectory)
		{
			trajectory.addPoint(cvPoint(-1, -1), cvScalar(-1, -1, -1));
		}

		if(showTrajectory)
		{
			trajectory.drawTrajectory(img);
		}

		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, .5, .5, 0, 1, 8);
		cvRectangle(img, cvPoint(0, 0), cvPoint(img->width, 50), black, CV_FILLED, 8, 0);
		cvPutText(img, string1, cvPoint(25, 25), &font, white);

		if(showForeground)
		{
			for(size_t i = 0; i < g_tld_track->detectorCascade->detectionResult->fgList->size(); i++)
			{
				Rect r = g_tld_track->detectorCascade->detectionResult->fgList->at(i);
				cvRectangle(img, r.tl(), r.br(), white, 1);
			}

		}

		if (showOutput)
		{
			g_gui->showImage(img);
			char key = g_gui->getKey();

			if (key == 'q') return;

			if (key == 't')
				showTrajectory =  (showTrajectory) ? false : true;

			if (key == 'b')
			{

				tld::ForegroundDetector *fg = g_tld_track->detectorCascade->foregroundDetector;

				if(fg->bgImg.empty())
				{
					fg->bgImg = grey.clone();
				}
				else
				{
					fg->bgImg.release();
				}
			}

			if(key == 'c')
			{
				//clear everything
				g_tld_track->release();
			}

			if(key == 'l')
			{
				g_tld_track->learningEnabled = !g_tld_track->learningEnabled;
				printf("LearningEnabled: %d\n", g_tld_track->learningEnabled);
			}

			if(key == 'a')
			{
				g_tld_track->alternating = !g_tld_track->alternating;
				printf("alternating: %d\n", g_tld_track->alternating);
			}

			if(key == 'r')
			{
				CvRect box;

				if(getBBFromUser(img, box, g_gui) == PROGRAM_EXIT)
				{
					printf("apertou q");
					return;
				}

				Rect r = Rect(box);

				g_tld_track->selectObject(grey, &r);
			}
		}
	}
}

static void
process_image(carmen_bumblebee_basic_stereoimage_message *msg)
{
    IplImage *src_image = NULL;
    IplImage *rgb_image = NULL;
    IplImage *resized_rgb_image = NULL;
    static char msg_fps_string[256];
    static char disp_fps_string[256];
    if (tld_image_width == 0)
    {
    	tld_image_width = msg->width;
    	tld_image_height = msg->height;
    }

    src_image = cvCreateImage(cvSize(msg->width, msg->height), IPL_DEPTH_8U, BUMBLEBEE_BASIC_VIEW_NUM_COLORS);
    rgb_image = cvCreateImage(cvSize(msg->width, msg->height), IPL_DEPTH_8U, BUMBLEBEE_BASIC_VIEW_NUM_COLORS);

    //TODO(verifica qual camera(essquerda ou direita) foi escolhida)
    if (camera_side == 0)
    	src_image->imageData = (char*) msg->raw_left;
    else
    	src_image->imageData = (char*) msg->raw_right;


    cvtColor(cvarrToMat(src_image), cvarrToMat(rgb_image), cv::COLOR_RGB2BGR);
//    TODO verificar quando fazer o resize
    resized_rgb_image = cvCreateImage(cvSize(640 , 480), rgb_image->depth, rgb_image->nChannels);

    cvResize(rgb_image, resized_rgb_image);
    CvFont font;
//    sprintf(msg_fps_string, "MSG_FPS: %d", msg_last_fps);
//    sprintf(disp_fps_string, "DISP_FPS: %d", disp_last_fps);
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.7, 0.7, 0, 1.0, CV_AA);
//    cvPutText(rgb_image, msg_fps_string, cvPoint(10, 30), &font, cvScalar(255, 255, 0, 0));

    //----Implementacoes do TLD
    process_TLD_detection(resized_rgb_image, msg->timestamp);
    //----FIM do TLD


//    Show the image
//    cvShowImage("TLD", resized_rgb_image);
//    waitKey(5);

    cvReleaseImage(&rgb_image);
    cvReleaseImage(&resized_rgb_image);
    cvReleaseImage(&src_image);
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
static void
image_handler(carmen_bumblebee_basic_stereoimage_message* image_msg)
{
	static double last_timestamp = 0.0;
	static double last_time = 0.0;
	double time_now = carmen_get_time();
	bounding_box box_detected;

	//Just process Rectified images
	if (image_msg->isRectified)
	{

		if (!received_image)
		{
			received_image = 1;
			last_timestamp = image_msg->timestamp;
			last_time = time_now;
		}


		if ((image_msg->timestamp - last_timestamp) > 1.0)
		{
			msg_last_fps = msg_fps;
			msg_fps = 0;
			last_timestamp = image_msg->timestamp;
		}
		msg_fps++;

		if ((time_now - last_time) > 1.0)
		{

			disp_last_fps = disp_fps;
			disp_fps = 0;
			last_time = time_now;
		}
		disp_fps++;


		last_message = *image_msg;

		process_image(image_msg);

		if (g_tld_track->currBB != NULL)
		{
			box_detected.x = g_tld_track->currBB->x;
			box_detected.y = g_tld_track->currBB->y;
			box_detected.width = g_tld_track->currBB->width;
			box_detected.height = g_tld_track->currBB->width;

			message_output.rect = box_detected;
			message_output.confidence = g_tld_track->currConf;
			message_output.host = image_msg->host;
			message_output.timestamp = image_msg->timestamp;

			publish_visual_tracker_output_message();

			//	verificar isso ai pra publicar
				printf("%lf %.2d %.2d %.2d %.2d %lf\n", image_msg->timestamp, g_tld_track->currBB->x,
						g_tld_track->currBB->y, g_tld_track->currBB->width, g_tld_track->currBB->height, message_output.confidence);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////

static void
shutdown_camera_view(int x)
{
    if (x == SIGINT)
    {
    	delete g_tld_track;
    	delete g_gui;
        carmen_ipc_disconnect();
        printf("Disconnected from robot.\n");
        exit(0);
    }
}

//
static int
read_parameters(int argc, char **argv, int camera)
{
    int num_items;

    std::string bumblebee_string = "bumblebee_basic";
    std::ostringstream myStream;
    myStream << camera << std::flush;
    bumblebee_string.append(myStream.str());
    std::string camera_string = "camera";
    camera_string.append(myStream.str());

//    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);
//
    carmen_param_t param_list[] = {
        {(char*) bumblebee_string.c_str(), (char*) "width", CARMEN_PARAM_INT, &tld_image_width, 0, NULL},
        {(char*) bumblebee_string.c_str(), (char*) "height", CARMEN_PARAM_INT, &tld_image_height, 0, NULL},
//
    };
//
    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);
//
    return 0;
}

int
main(int argc, char **argv)
{

    int camera = 0;

    if (argc != 3)
    {
        fprintf(stderr, "%s: Wrong number of parameters. TLD requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)\n>", argv[0], argc - 1, argv[0]);
        exit(1);
    }

    camera = atoi(argv[1]);
    camera_side = atoi(argv[2]);

    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    read_parameters(argc, argv, camera);
    g_tld_track = new tld::TLD();
    g_gui = new tld::Gui;

    signal(SIGINT, shutdown_camera_view);

    inicialize_TLD_parameters();

    carmen_visual_tracker_define_messages();

    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_ipc_dispatch();

}
