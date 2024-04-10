#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <unistd.h>
#include <time.h>
#include "tools/mySerial.h"
#include "coral_yolo.hpp"

mySerial serialPort("/dev/ttyAMA4", 600);

void createCamera(lccv::PiCamera &camera_, int id)
{
    camera_.options->camera = id;
    camera_.options->video_width = 640;
    camera_.options->video_height = 360;
    camera_.options->framerate = 60;
    // camera_.options->verbose = true;
}

void sendSerial()
{
    serialPort.Send('\x00');
}

// bool isInSegment(cv::Point2d point, float *segMap) {
//     if (segMap)
// }

int numDisparities = 18;
int blockSize = 2;
int preFilterType = 1;
int preFilterSize = 1;
int preFilterCap = 31;
int minDisparity = 50;
int textureThreshold = 10;
int uniquenessRatio = 15;
int speckleRange = 0;
int speckleWindowSize = 0;
int disp12MaxDiff = -1;
int dispType = CV_16S;


cv::Ptr<cv::StereoBM> leftStereo = cv::StereoBM::create(0);

static void on_trackbar1( int, void* )
{
  leftStereo->setNumDisparities(numDisparities*16);
  numDisparities = numDisparities*16;
}
 
static void on_trackbar2( int, void* )
{
  leftStereo->setBlockSize(blockSize*2+5);
  blockSize = blockSize*2+5;
}
 
static void on_trackbar3( int, void* )
{
  leftStereo->setPreFilterType(preFilterType);
}
 
static void on_trackbar4( int, void* )
{
  leftStereo->setPreFilterSize(preFilterSize*2+5);
  preFilterSize = preFilterSize*2+5;
}
 
static void on_trackbar5( int, void* )
{
  leftStereo->setPreFilterCap(preFilterCap);
}
 
static void on_trackbar6( int, void* )
{
  leftStereo->setTextureThreshold(textureThreshold);
}
 
static void on_trackbar7( int, void* )
{
  leftStereo->setUniquenessRatio(uniquenessRatio);
}
 
static void on_trackbar8( int, void* )
{
  leftStereo->setSpeckleRange(speckleRange);
}
 
static void on_trackbar9( int, void* )
{
  leftStereo->setSpeckleWindowSize(speckleWindowSize*2);
  speckleWindowSize = speckleWindowSize*2;
}
 
static void on_trackbar10( int, void* )
{
  leftStereo->setDisp12MaxDiff(disp12MaxDiff);
}
 
static void on_trackbar11( int, void* )
{
  leftStereo->setMinDisparity(minDisparity);
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

    leftStereo->setMinDisparity(0);
    leftStereo->setNumDisparities(128);
    leftStereo->setBlockSize(55); // 15
    leftStereo->setPreFilterType(1);
    leftStereo->setPreFilterSize(7);
    leftStereo->setPreFilterCap(62);
    leftStereo->setTextureThreshold(100);
    leftStereo->setUniquenessRatio(0);
    leftStereo->setSpeckleRange(100);
    leftStereo->setSpeckleWindowSize(3);
    leftStereo->setDisp12MaxDiff(-1);

    float M = 126.52103;

    cv::Ptr<cv::StereoMatcher> rightStereo = cv::ximgproc::createRightMatcher(leftStereo);
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wslFilter = cv::ximgproc::createDisparityWLSFilter(rightStereo);
    wslFilter->setLambda(12000);
    wslFilter->setSigmaColor(2.0);
    cv::Mat disparity;

    // cv::namedWindow("Video", cv::WINDOW_NORMAL);
    // cv::namedWindow("Video1", cv::WINDOW_NORMAL);

    cv::Mat cam_0_im, cam_1_im, cam_0_gray, cam_1_gray;
    lccv::PiCamera cam_0, cam_1;
    createCamera(cam_0, 0);
    createCamera(cam_1, 1);

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

    // cv::namedWindow("disparity",cv::WINDOW_NORMAL);
    // cv::resizeWindow("disparity",600,600);

    // // Creating trackbars to dynamically update the StereoBM parameters
    // cv::createTrackbar("numDisparities", "disparity", &numDisparities, 18, on_trackbar1);
    // cv::createTrackbar("blockSize", "disparity", &blockSize, 50, on_trackbar2);
    // cv::createTrackbar("preFilterType", "disparity", &preFilterType, 1, on_trackbar3);
    // cv::createTrackbar("preFilterSize", "disparity", &preFilterSize, 25, on_trackbar4);
    // cv::createTrackbar("preFilterCap", "disparity", &preFilterCap, 62, on_trackbar5);
    // cv::createTrackbar("textureThreshold", "disparity", &textureThreshold, 100, on_trackbar6);
    // cv::createTrackbar("uniquenessRatio", "disparity", &uniquenessRatio, 100, on_trackbar7);
    // cv::createTrackbar("speckleRange", "disparity", &speckleRange, 100, on_trackbar8);
    // cv::createTrackbar("speckleWindowSize", "disparity", &speckleWindowSize, 25, on_trackbar9);
    // // cv::createTrackbar("disp12MaxDiff", "disparity", &disp12MaxDiff, 25, on_trackbar10);
    // cv::createTrackbar("minDisparity", "disparity", &minDisparity, 25, on_trackbar11);

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
            // printf("%d\n", Left_Stereo_Map2.type());

            cv::Mat leftNice, rightNice, leftDisp, rightDisp, filteredDisp;
            cv::remap(cam_0_gray, leftNice, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
            cv::remap(cam_1_gray, rightNice, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, 0);
            // printf("%d, %d\n", leftNice.size().height, leftNice.size().width);

            leftStereo->compute(leftNice, rightNice, leftDisp);
            rightStereo->compute(rightNice, leftNice, rightDisp);
            wslFilter->filter(leftDisp, leftNice, filteredDisp, rightDisp);
            leftDisp.convertTo(disparity, CV_32F, 1.0);
            // std::cout << disp.type() << std::endl;
            filteredDisp.convertTo(filteredDisp, CV_32F, 1.0);
            // leftDisp = leftDisp / (short) 16;
            disparity = (filteredDisp/16.0f - (float)0)/((float)128);
            // disparity = (disparity/16.0f - (float)minDisparity)/((float)numDisparities);

            short midDisp = leftDisp.at<short>(180, 320) / (short) 16;
            float distance = M / (float) midDisp;
            cv::cvtColor(disparity, disparity, cv::COLOR_GRAY2BGR);

            cv::putText(disparity, std::to_string(distance) + " m", cv::Point(0, 30), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0));
            cv::circle(disparity, cv::Point(320, 180), 5, cv::Scalar(0, 255, 0), 5);

            // cv::resize(leftNice, leftNice, cv::Size(728, 544));
            // cv::imshow("Video", leftNice);
            cv::imshow("Video1", rightNice);
            // cv::imshow("Video", disparity);
            
            ch = cv::waitKey(10);

            counter++;
        }
        if ((time(0) - start) >= 1)
        {
            printf("FPS: %d\n", counter);
            start = time(0);
            counter = 0;
            double max, min;
        }
    }
    cam_0.stopVideo();
    cam_1.stopVideo();
    cv::destroyWindow("Video");
}
