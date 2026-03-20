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

void drawAxes(cv::Mat& f, const std::vector<cv::Point2f>& ip) {
    cv::arrowedLine(f, ip[0], ip[1], {0,0,255},   3, cv::LINE_AA);
    cv::arrowedLine(f, ip[0], ip[2], {0,255,0},   3, cv::LINE_AA);
    cv::arrowedLine(f, ip[0], ip[3], {255,80,0},  3, cv::LINE_AA);
    cv::putText(f, "X", ip[1], cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,0,255},   2);
    cv::putText(f, "Y", ip[2], cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,255,0},   2);
    cv::putText(f, "Z", ip[3], cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,80,0},  2);
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

// Extension 2: fill the checkerboard quad with a solid green "grass" color
void hideCheckerboard(cv::Mat& frame,
                      const cv::Mat& rvec, const cv::Mat& tvec,
                      const cv::Mat& cam,  const cv::Mat& dist) {
    // Project board boundary + a grid of fill points
    std::vector<cv::Point3f> boundary;
    // Dense grid across board surface
    for (float r = 0; r <= 5.f; r += 0.5f)
        for (float c = 0; c <= 8.f; c += 0.5f)
            boundary.push_back({c, -r, 0.f});

    std::vector<cv::Point2f> ip;
    cv::projectPoints(boundary, rvec, tvec, cam, dist, ip);

    // Build convex hull of projected points and fill it
    std::vector<cv::Point> hull_pts;
    for (auto& p : ip) hull_pts.push_back({(int)p.x, (int)p.y});
    std::vector<cv::Point> hull;
    cv::convexHull(hull_pts, hull);

    // Fill with grass green to hide the checkerboard
    cv::fillConvexPoly(frame, hull, {34, 139, 34}, cv::LINE_AA);

    // Add a subtle grid texture so it doesn't look flat
    std::vector<cv::Point3f> grid3d;
    std::vector<cv::Point2f> grid2d;
    for (int r = 0; r <= 5; r++) {
        grid3d.push_back({0.f, -(float)r, 0.f});
        grid3d.push_back({8.f, -(float)r, 0.f});
    }
    for (int c = 0; c <= 8; c++) {
        grid3d.push_back({(float)c, 0.f,  0.f});
        grid3d.push_back({(float)c, -5.f, 0.f});
    }
    cv::projectPoints(grid3d, rvec, tvec, cam, dist, grid2d);
    for (int i = 0; i < (int)grid2d.size(); i += 2)
        cv::line(frame, grid2d[i], grid2d[i+1], {0,100,0}, 1, cv::LINE_AA);
}

int main() {
    cv::FileStorage fs("intrinsics.yaml", cv::FileStorage::READ);
    if (!fs.isOpened()) { std::cerr << "intrinsics.yaml not found\n"; return -1; }
    cv::Mat cam, dist;
    fs["camera_matrix"] >> cam;
    fs["dist_coeffs"]   >> dist;
    fs.release();

    const std::vector<cv::Point3f> world_pts = buildWorldPts();
    cv::VideoCapture cap(0);
    cv::Mat frame, gray;
    bool showAxes = true, showCorners = true, showObj = true, hideBoard = false;

    std::cout << "a=axes  c=corners  v=object  h=hide board  q=quit\n";

    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, BOARD, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            cv::cornerSubPix(gray, corners, {11,11}, {-1,-1},
                {cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001});

            cv::Mat rvec, tvec;
            cv::solvePnP(world_pts, corners, cam, dist, rvec, tvec);

            printf("R:[%.2f %.2f %.2f]  T:[%.2f %.2f %.2f]\r",
                rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2),
                tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
            std::cout.flush();

            // Extension 2: hide board BEFORE drawing AR on top
            if (hideBoard) hideCheckerboard(frame, rvec, tvec, cam, dist);

            if (showCorners) {
                std::vector<cv::Point3f> outer = {{0,0,0},{8,0,0},{8,-5,0},{0,-5,0}};
                std::vector<cv::Point2f> oc;
                cv::projectPoints(outer, rvec, tvec, cam, dist, oc);
                for (auto& p : oc) cv::circle(frame, p, 7, {0,255,0}, cv::FILLED);
                cv::line(frame,oc[0],oc[1],{0,255,0},2,cv::LINE_AA);
                cv::line(frame,oc[1],oc[2],{0,255,0},2,cv::LINE_AA);
                cv::line(frame,oc[2],oc[3],{0,255,0},2,cv::LINE_AA);
                cv::line(frame,oc[3],oc[0],{0,255,0},2,cv::LINE_AA);
            }

            if (showAxes) {
                std::vector<cv::Point3f> axPts = {{0,0,0},{3,0,0},{0,-3,0},{0,0,3}};
                std::vector<cv::Point2f> axImg;
                cv::projectPoints(axPts, rvec, tvec, cam, dist, axImg);
                drawAxes(frame, axImg);
            }

            if (showObj) drawTower(frame, rvec, tvec, cam, dist);
        }

        cv::imshow("AR Viewer", frame);
        int key = cv::waitKey(30);
        if (key == 'q') break;
        if (key == 'a') showAxes    = !showAxes;
        if (key == 'c') showCorners = !showCorners;
        if (key == 'v') showObj     = !showObj;
        if (key == 'h') {
            hideBoard = !hideBoard;
            std::cout << "\nHide board: " << (hideBoard ? "ON" : "OFF") << "\n";
        }
    }
    return 0;
}