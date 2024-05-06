#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

#include "calibration/pointGenerator.hpp"

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

    cv::Mat cam_0_im, vis;

    cv::aruco::DetectorParameters params  = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    cv::aruco::ArucoDetector detector(dict, params);

    cv::Size patternSize_(6, 8);

    std::vector<std::pair<cv::Point2f, cv::Point2f>> points = generateIntrPoints(cv::Size(cam_0.options->video_width, cam_0.options->video_height));

    while (ch != 27) {
        if (!cam_0.getVideoFrame(cam_0_im, 99999999))
        {
            std::cout << "Timeout error " << std::endl;
        }

        cv::cvtColor(cam_0_im, vis, cv::COLOR_RGB2BGR);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        detector.detectMarkers(cam_0_im, markerCorners, markerIds, rejectedCandidates);

        cv::aruco::drawDetectedMarkers(vis, markerCorners, markerIds);

        std::vector<cv::Point2f> tag0Corners, tag1Corners;
        for (int i = 0; i < markerIds.size(); i++) {
            if (markerIds[i] == 0) {
                tag0Corners = markerCorners[i];
            } else if (markerIds[i] == 1) {
                tag1Corners = markerCorners[i];
            }
        }

        std::pair<cv::Point2f, cv::Point2f> tagLoc = points[1];

        cv::circle(vis, tagLoc.first, 5, cv::Scalar(255, 0, 0), 5);
        cv::circle(vis, tagLoc.second, 5, cv::Scalar(255, 0, 0), 5);


        if (tag0Corners.size() > 0 && tag1Corners.size() > 0) {
            cv::Mat mean;
            cv::reduce(tag0Corners, mean, 01, cv::REDUCE_AVG);
            cv::Point2f tag0Pos(mean.at<float>(0, 0), mean.at<float>(0, 1));
            cv::reduce(tag1Corners, mean, 01, cv::REDUCE_AVG);
            cv::Point2f tag1Pos(mean.at<float>(0, 0), mean.at<float>(0, 1));

            

            double tag0dist = cv::norm(tagLoc.first - tag0Pos);
            double tag1dist = cv::norm(tagLoc.second - tag1Pos);

            if (tag0dist < 10 && tag1dist < 10) {
                cv::Mat grayIm;
                cv::cvtColor(cam_0_im, grayIm, cv::COLOR_RGB2GRAY);

                std::vector<cv::Point2f> cornerPts;
                bool found = cv::findChessboardCornersSB(grayIm, patternSize_, cornerPts);

                if (found) {
                    cv::drawChessboardCorners(vis, patternSize_, cornerPts, found);
                }
            }

            printf("Tag 0 pos: %f, %f\n", tag0Pos.x, tag0Pos.y);
            printf("Tag 1 pos: %f, %f\n", tag1Pos.x, tag1Pos.y);
        }

        // printf("%d markers detected\n", markerIds.size());

        cv::imshow("Image", vis);
        ch = cv::waitKey(5);
    }
    cv::destroyAllWindows();
    cam_0.stopVideo();
}