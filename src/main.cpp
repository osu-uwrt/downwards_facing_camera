#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <unistd.h>
#include <time.h>
#include "tools/mySerial.h"

mySerial serialPort("/dev/ttyAMA0", 600);

void createCamera(lccv::PiCamera &camera_, int id)
{
    camera_.options->camera = id;
    camera_.options->video_width = 1200;
    camera_.options->video_height = 720;
    camera_.options->framerate = 60;
    // camera_.options->verbose = true;
}

void sendSerial()
{
    serialPort.Send('\x00');
}

int numDisparities = 8;
int blockSize = 5;
int preFilterType = 1;
int preFilterSize = 1;
int preFilterCap = 31;
int minDisparity = 0;
int textureThreshold = 10;
int uniquenessRatio = 15;
int speckleRange = 0;
int speckleWindowSize = 0;
int disp12MaxDiff = -1;
int dispType = CV_16S;


cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(0);

static void on_trackbar1( int, void* )
{
  stereo->setNumDisparities(numDisparities*16);
  numDisparities = numDisparities*16;
}
 
static void on_trackbar2( int, void* )
{
  stereo->setBlockSize(blockSize*2+5);
  blockSize = blockSize*2+5;
}
 
static void on_trackbar3( int, void* )
{
  stereo->setPreFilterType(preFilterType);
}
 
static void on_trackbar4( int, void* )
{
  stereo->setPreFilterSize(preFilterSize*2+5);
  preFilterSize = preFilterSize*2+5;
}
 
static void on_trackbar5( int, void* )
{
  stereo->setPreFilterCap(preFilterCap);
}
 
static void on_trackbar6( int, void* )
{
  stereo->setTextureThreshold(textureThreshold);
}
 
static void on_trackbar7( int, void* )
{
  stereo->setUniquenessRatio(uniquenessRatio);
}
 
static void on_trackbar8( int, void* )
{
  stereo->setSpeckleRange(speckleRange);
}
 
static void on_trackbar9( int, void* )
{
  stereo->setSpeckleWindowSize(speckleWindowSize*2);
  speckleWindowSize = speckleWindowSize*2;
}
 
static void on_trackbar10( int, void* )
{
  stereo->setDisp12MaxDiff(disp12MaxDiff);
}
 
static void on_trackbar11( int, void* )
{
  stereo->setMinDisparity(minDisparity);
}

int main()
{

    cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
    cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;

    cv::FileStorage mapsFile = cv::FileStorage("/home/pi/StereoMaps.xml", cv::FileStorage::READ);
    mapsFile["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
    mapsFile["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
    mapsFile["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
    mapsFile["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
    mapsFile.release();

    stereo->setMinDisparity(0);
    cv::Mat disparity;

    // cv::namedWindow("Video", cv::WINDOW_NORMAL);
    // cv::namedWindow("Video1", cv::WINDOW_NORMAL);

    cv::Mat cam_0_im, cam_1_im, cam_0_gray, cam_1_gray;
    lccv::PiCamera cam_0, cam_1;
    createCamera(cam_0, 0);
    createCamera(cam_1, 1);
    // cam_0.options->video_width = 1200;
    // cam_0.options->video_width = 720;

    // cam_1.options->video_width = 1200;
    // cam_1.options->video_width = 720;

    cam_0.startVideo();
    std::cout << "Camera 0 Started" << std::endl;
    cam_1.startVideo();
    std::cout << "Camera 1 Started" << std::endl;
    int ch = 0;
    int counter = 0;
    sleep(1); // Let libcamera start (And why do you want it so fast smh)

    // Need to fill up buffer (fairly sure why we need to do this, not entirely sure lmao)
    for (int i = 0; i < 8; i++)
    {
        sendSerial();
    }

    time_t start = time(0);

    cv::namedWindow("disparity",cv::WINDOW_NORMAL);
    cv::resizeWindow("disparity",600,600);
    
    // Creating trackbars to dynamically update the StereoBM parameters
    cv::createTrackbar("numDisparities", "disparity", &numDisparities, 18, on_trackbar1);
    cv::createTrackbar("blockSize", "disparity", &blockSize, 50, on_trackbar2);
    cv::createTrackbar("preFilterType", "disparity", &preFilterType, 1, on_trackbar3);
    cv::createTrackbar("preFilterSize", "disparity", &preFilterSize, 25, on_trackbar4);
    cv::createTrackbar("preFilterCap", "disparity", &preFilterCap, 62, on_trackbar5);
    cv::createTrackbar("textureThreshold", "disparity", &textureThreshold, 100, on_trackbar6);
    cv::createTrackbar("uniquenessRatio", "disparity", &uniquenessRatio, 100, on_trackbar7);
    cv::createTrackbar("speckleRange", "disparity", &speckleRange, 100, on_trackbar8);
    cv::createTrackbar("speckleWindowSize", "disparity", &speckleWindowSize, 25, on_trackbar9);
    cv::createTrackbar("disp12MaxDiff", "disparity", &disp12MaxDiff, 25, on_trackbar10);
    cv::createTrackbar("minDisparity", "disparity", &minDisparity, 25, on_trackbar11);

    while (ch != 27)
    {
        sendSerial();
        if (!cam_0.getVideoFrame(cam_0_im, 99999999) || !cam_1.getVideoFrame(cam_1_im, 99999999))
        {
            std::cout << "Timeout error " << counter << std::endl;
        }
        else
        {
            cv::cvtColor(cam_0_im, cam_0_gray, cv::COLOR_RGB2GRAY);
            cv::cvtColor(cam_1_im, cam_1_gray, cv::COLOR_RGB2GRAY);
            // cv::imshow("Video", cam_0_im);
            // cv::imshow("Video1", cam_1_im);
            // printf("%d, %d", cam_0_im.size().width, cam_0_im.size().height);

            cv::Mat leftNice, rightNice, disp;
            cv::remap(cam_0_gray, leftNice, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
            cv::remap(cam_1_gray, rightNice, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
            // printf("%d, %d\n", leftNice.size().height, leftNice.size().width);

            stereo->compute(leftNice, rightNice, disp);
            disp.convertTo(disparity, CV_32F, 1.0);
            // std::cout << disp.type() << std::endl;
            // disp.convertTo(disp, CV_8UC1);
            disparity = (disparity/16.0f - (float)minDisparity)/((float)numDisparities);

            short midDisp = disp.at<short>(360, 600);
            // midDisp = 123 * 5.17 / midDisp;  
            // std::cout << disp.rows << " " << disp.cols << std::endl;
            cv::cvtColor(disparity, disparity, cv::COLOR_GRAY2BGR);

            cv::putText(disparity, std::to_string(midDisp), cv::Point(0, 30), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0));
            cv::circle(disparity, cv::Point(600, 360), 10, cv::Scalar(0, 0, 255));

            // cv::resize(leftNice, leftNice, cv::Size(728, 544));
            // cv::imshow("Video", leftNice);
            // cv::imshow("Video1", rightNice);
            cv::imshow("Video", disparity);
            
            ch = cv::waitKey(10);

            counter++;
        }
        if ((time(0) - start) >= 1)
        {
            // printf("FPS: %d\n", counter);
            start = time(0);
            counter = 0;
            double max, min;
        }
    }
    cam_0.stopVideo();
    cam_1.stopVideo();
    cv::destroyWindow("Video");
}
