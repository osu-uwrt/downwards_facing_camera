#include <opencv2/calib3d.hpp>
#include <tools/mySerial.h>

int totalImages = 0;
cv::Size imageSize = cv::Size(640, 360);
cv::Size patternSize = cv::Size(6, 8);

mySerial serialPort("/dev/ttyAMA4", 600);
