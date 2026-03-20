#include <opencv2/opencv.hpp>
#include <iostream>

int thresh    = 150;
int blockSize = 2;   // Harris neighborhood size

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) { std::cerr << "Cannot open camera\n"; return -1; }

    cv::namedWindow("Harris Features");
    cv::createTrackbar("Threshold",  "Harris Features", &thresh,    255, [](int,void*){});
    cv::createTrackbar("Block Size", "Harris Features", &blockSize, 7,   [](int,void*){});

    std::cout << "Harris corner detection — q to quit\n";
    std::cout << "High-response points (red dots) are stable corners.\n";
    std::cout << "Such keypoints could replace the checkerboard: match them\n";
    std::cout << "across frames via descriptor matching (e.g. ORB/SIFT) to\n";
    std::cout << "recover homography or 3D pose for AR overlay.\n";

    cv::Mat frame, gray, harris, norm, norm8, colorMap;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        int bs = std::max(2, blockSize); // must be >= 2
        cv::cornerHarris(gray, harris, bs, 3, 0.04);

        cv::normalize(harris, norm, 0, 255, cv::NORM_MINMAX, CV_32FC1);
        norm.convertTo(norm8, CV_8U);

        cv::Mat display = frame.clone();
        int count = 0;
        for (int r = 0; r < norm.rows; r++)
            for (int c = 0; c < norm.cols; c++)
                if (norm.at<float>(r,c) > thresh) {
                    cv::circle(display, {c,r}, 3, {0,0,255}, cv::FILLED);
                    ++count;
                }

        cv::putText(display, "Features: " + std::to_string(count),
            {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,255,0}, 2);

        cv::applyColorMap(norm8, colorMap, cv::COLORMAP_JET);

        cv::imshow("Harris Features",    display);
        cv::imshow("Harris Response Map", colorMap);

        if (cv::waitKey(30) == 'q') break;
    }
    return 0;
}