#include <opencv2/calib3d.hpp>
#include <tools/mySerial.h>

int totalImages = 41;
cv::Size imageSize = cv::Size(640, 360);
cv::Size patternSize = cv::Size(8, 6);

mySerial serialPort("/dev/ttyAMA0", 600);