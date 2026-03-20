#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

const cv::Size BOARD(9, 6);

std::vector<cv::Point3f> buildWorldPts() {
    std::vector<cv::Point3f> pts;
    for (int r = 0; r < BOARD.height; r++)
        for (int c = 0; c < BOARD.width; c++)
            pts.emplace_back((float)c, -(float)r, 0.f);
    return pts;
}

void drawTower(cv::Mat& frame, const cv::Mat& rvec, const cv::Mat& tvec,
               const cv::Mat& cam, const cv::Mat& dist) {
    std::vector<cv::Point3f> pts = {
        {1.5f,-0.f,0},{6.5f,-0.f,0},{6.5f,-5.f,0},{1.5f,-5.f,0},
        {3.f,-1.5f,2},{5.f,-1.5f,2},{5.f,-3.5f,2},{3.f,-3.5f,2},
        {3.5f,-2.f,4},{4.5f,-2.f,4},{4.5f,-3.f,4},{3.5f,-3.f,4},
        {4.f,-2.5f,5.5f},{4.f,-2.5f,7.f},
        {3.7f,-2.5f,7.f},{4.3f,-2.5f,7.f},{4.f,-2.2f,7.f},{4.f,-2.8f,7.f},
    };
    std::vector<cv::Point2f> ip;
    cv::projectPoints(pts, rvec, tvec, cam, dist, ip);
    auto L = [&](int a, int b, cv::Scalar c, int t=2) {
        cv::line(frame, ip[a], ip[b], c, t, cv::LINE_AA);
    };
    cv::Scalar gold(0,215,255), orange(0,140,255), red2(30,30,220), white(240,240,240);
    L(0,1,gold);L(1,2,gold);L(2,3,gold);L(3,0,gold);L(0,2,gold);L(1,3,gold);
    L(0,4,gold,3);L(1,5,gold,3);L(2,6,gold,3);L(3,7,gold,3);
    L(4,5,orange);L(5,6,orange);L(6,7,orange);L(7,4,orange);L(4,6,orange);L(5,7,orange);
    L(4,8,orange,3);L(5,9,orange,3);L(6,10,orange,3);L(7,11,orange,3);
    L(8,9,red2);L(9,10,red2);L(10,11,red2);L(11,8,red2);
    L(8,12,red2,3);L(9,12,red2,3);L(10,12,red2,3);L(11,12,red2,3);
    L(12,13,white,2);L(14,15,white,2);L(16,17,white,2);
}

int main() {
    cv::FileStorage fs("intrinsics.yaml", cv::FileStorage::READ);
    if (!fs.isOpened()) { std::cerr << "intrinsics.yaml not found\n"; return -1; }
    cv::Mat cam, dist;
    fs["camera_matrix"] >> cam;
    fs["dist_coeffs"]   >> dist;
    fs.release();

    const std::vector<cv::Point3f> world_pts = buildWorldPts();

    // Load all calibration images saved during calibration
    for (int i = 0; i < 20; i++) {
        std::string path = "calib_" + std::to_string(i) + ".jpg";
        cv::Mat img = cv::imread(path);
        if (img.empty()) continue;

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, BOARD, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (!found) { std::cout << path << ": board not found\n"; continue; }

        cv::cornerSubPix(gray, corners, {11,11}, {-1,-1},
            {cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001});

        cv::Mat rvec, tvec;
        cv::solvePnP(world_pts, corners, cam, dist, rvec, tvec);
        cv::drawChessboardCorners(img, BOARD, corners, true);
        drawTower(img, rvec, tvec, cam, dist);

        // Project 3D axes
        std::vector<cv::Point3f> axPts = {{0,0,0},{3,0,0},{0,-3,0},{0,0,3}};
        std::vector<cv::Point2f> axImg;
        cv::projectPoints(axPts, rvec, tvec, cam, dist, axImg);
        cv::arrowedLine(img, axImg[0], axImg[1], {0,0,255},   3, cv::LINE_AA);
        cv::arrowedLine(img, axImg[0], axImg[2], {0,255,0},   3, cv::LINE_AA);
        cv::arrowedLine(img, axImg[0], axImg[3], {255,80,0},  3, cv::LINE_AA);

        std::string outpath = "static_ar_" + std::to_string(i) + ".jpg";
        cv::imwrite(outpath, img);
        std::cout << "Saved: " << outpath << "\n";

        cv::imshow("Static AR - " + path, img);
        cv::waitKey(0); // press any key to advance to next image
    }
    std::cout << "Done — check cmake-build-debug/ for static_ar_*.jpg\n";
    return 0;
}