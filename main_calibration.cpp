#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

// 9x6 internal corners on the provided checkerboard
const cv::Size BOARD(9, 6);

// Fixed 3D world coords: one unit = one checkerboard square, Z=0 plane
std::vector<cv::Point3f> buildWorldPts() {
    std::vector<cv::Point3f> pts;
    for (int r = 0; r < BOARD.height; r++)
        for (int c = 0; c < BOARD.width; c++)
            pts.emplace_back((float)c, -(float)r, 0.f);
    return pts;
}

int main() {
    cv::VideoCapture cap(0);
    cv::waitKey(1000); // give camera time to initialize
    if (!cap.isOpened()) { std::cerr << "Cannot open camera\n"; return -1; }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    std::vector<std::vector<cv::Point3f>> point_list;
    std::vector<std::vector<cv::Point2f>> corner_list;
    const std::vector<cv::Point3f> point_set = buildWorldPts();

    cv::Mat frame, gray;
    std::vector<cv::Point2f> corner_set;
    bool found = false;
    int saved = 0;

    std::cout << "s=save frame | c=calibrate (>=5) | q=quit\n";

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        found = cv::findChessboardCorners(gray, BOARD, corner_set,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            cv::cornerSubPix(gray, corner_set, {11,11}, {-1,-1},
                {cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001});
            cv::drawChessboardCorners(frame, BOARD, corner_set, found);
            printf("Corners: %d  First: (%.1f, %.1f)\r",
                (int)corner_set.size(), corner_set[0].x, corner_set[0].y);
            std::cout.flush();
        }

        cv::imshow("Calibration", frame);
        int key = cv::waitKey(30);
        if (key == 'q') break;

        if (key == 's') {
            if (found) {
                corner_list.push_back(corner_set);
                point_list.push_back(point_set);
                cv::imwrite("calib_" + std::to_string(saved++) + ".jpg", frame);
                std::cout << "\nSaved. Total: " << corner_list.size() << "\n" << std::flush;
            } else {
                std::cout << "\nNot saved — checkerboard not detected! Make sure dots are visible.\n" << std::flush;
            }
        }

        if (key == 'c') {
            if ((int)corner_list.size() < 5) {
                std::cout << "\nNeed at least 5 calibration frames.\n"; continue;
            }

            // Initialize camera matrix with principal point at image center
            cv::Mat cam = (cv::Mat_<double>(3,3) <<
                1, 0, frame.cols / 2.0,
                0, 1, frame.rows / 2.0,
                0, 0, 1);
            cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F); // k1 k2 p1 p2 k3
            std::vector<cv::Mat> rvecs, tvecs;

            std::cout << "\n=== Before calibration ===\n" << cam << "\ndist: " << dist.t() << "\n";

            double rpe = cv::calibrateCamera(point_list, corner_list, frame.size(),
                cam, dist, rvecs, tvecs, cv::CALIB_FIX_ASPECT_RATIO);

            std::cout << "=== After calibration ===\n" << cam << "\ndist: " << dist.t() << "\n";
            std::cout << "Re-projection error: " << rpe << "\n";

            // Write intrinsics for use by ar_viewer
            cv::FileStorage fs("intrinsics.yaml", cv::FileStorage::WRITE);
            fs << "camera_matrix" << cam << "dist_coeffs" << dist;
            fs.release();
            std::cout << "Intrinsics saved to intrinsics.yaml\n";
        }
    }
    return 0;
}