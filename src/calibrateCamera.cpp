#include <unistd.h>

#include <lccv.hpp>
#include <tools/mySerial.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

mySerial serialPort("/dev/ttyAMA0", 450);

cv::Size checkerboardSize(8,6);

void createCamera(lccv::PiCamera &camera_, int id)
{
    camera_.options->camera = id;
    camera_.options->video_width = 1456;
    camera_.options->video_height = 1088;
    camera_.options->framerate = 60;
}

void sendSerial()
{
    serialPort.Send('\x00');
}

int main(int argc, char *argv[])
{

    lccv::PiCamera camera;
    cv::Mat feed;

    cv::namedWindow("Camera feed", cv::WINDOW_NORMAL);

    if (argc == 1)
    {
        printf("No arguments given, defaulting to cam 0\n");
        createCamera(camera, 0);
    }
    else
    {
        if (std::stoi(argv[0]) > 1)
        {
            printf("Invalid arguement\n");
            return 0;
        }
        createCamera(camera, std::stoi(argv[0]));
    }

    camera.startVideo();
    sleep(1); // Let libcamera start

    // Need to fill up buffer (fairly sure why we need to do this, not entirely sure lmao)
    for (int i = 0; i < 8; i++)
    {
        sendSerial();
    }

    for (int i = 0; i < 39; i++)
    {
        printf("Current image: %d\n", i);
        bool found = false;
        int press;
        while (!found)
        {
            sendSerial();
            if (!camera.getVideoFrame(feed, 1e6))
            {
                std::cout << "Timeout error " << std::endl;
            }
            else
            {
                cv::imshow("Camera feed", feed);
                press = cv::waitKey(10);
            }
            if (press == 97)
            {
                printf("Pressed\n");
                cv::Mat  grayIm, corners;
                cv::cvtColor(feed, grayIm, cv::COLOR_BGR2GRAY);
                found = cv::checkChessboard(grayIm, checkerboardSize);
                // found = cv::findChessboardCornersSB(grayIm, checkerboardSize, corners);
                printf("Past being stuck?");
                if (!found) {
                    printf("Couldn't find image\n");
                }
            }
        }
        printf("found");
        return 0;
    }
}