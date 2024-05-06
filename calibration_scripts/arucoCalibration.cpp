#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

#include <lccv.hpp>

void createCamera(lccv::PiCamera &camera_, int id)
{
    camera_.options->camera = id;
    camera_.options->video_width = 640;
    camera_.options->video_height = 360;
    camera_.options->framerate = 60;
    // camera_.options->verbose = true;
}


int main(int argc, char *argv[]) {
    lccv::PiCamera cam_0;
    createCamera(cam_0, 0);
    cam_0.startVideo();

    char ch = 0;

    cv::Mat cam_0_im;

    cv::aruco::DetectorParameters params  = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    cv::aruco::ArucoDetector detector(dict, params);

    while (ch != 27) {
        if (!cam_0.getVideoFrame(cam_0_im, 99999999))
        {
            std::cout << "Timeout error " << std::endl;
        }

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        detector.detectMarkers(cam_0_im, markerCorners, markerIds, rejectedCandidates);

        cv::aruco::drawDetectedMarkers(cam_0_im, markerCorners, markerIds);

        std::vector<cv::Point2f> tag0Corners, tag1Corners;
        for (int i = 0; i < markerIds.size(); i++) {
            if (markerIds[i] == 0) {
                tag0Corners = markerCorners[i];
            } else if (markerIds[i] == 1) {
                tag1Corners = markerCorners[i];
            }
        }
        if (tag0Corners.size() > 0 && tag1Corners.size() > 0) {
            cv::Mat mean;
            cv::reduce(tag0Corners, mean, 01, cv::REDUCE_AVG);
            cv::Point2f tag0Pos(mean.at<float>(0, 0), mean.at<float>(0, 1));
            cv::reduce(tag1Corners, mean, 01, cv::REDUCE_AVG);
            cv::Point2f tag1Pos(mean.at<float>(0, 0), mean.at<float>(0, 1));

            printf("Tag 0 pos: %f, %f\n", tag0Pos.x, tag0Pos.y);
            printf("Tag 1 pos: %f, %f\n", tag1Pos.x, tag1Pos.y);
        }

        // printf("%d markers detected\n", markerIds.size());

        cv::imshow("Image", cam_0_im);
        ch = cv::waitKey(5);
    }
    cv::destroyAllWindows();
    cam_0.stopVideo();
}