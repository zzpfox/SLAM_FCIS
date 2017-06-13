//#include <opencv2/core.hpp>
//#include <opencv2/sfm.hpp>
//#include <opencv2/viz.hpp>
//#include <opencv2/calib3d.hpp>
//#include <iostream>
//#include <fstream>
//#include <string>
//using namespace std;
//using namespace cv;
//using namespace cv::sfm;
//static void help() {
//    cout
//            << "\n------------------------------------------------------------------------------------\n"
//            << " This program shows the multiview reconstruction capabilities in the \n"
//            << " OpenCV Structure From Motion (SFM) module.\n"
//            << " It reconstruct a scene from a set of 2D images \n"
//            << " Usage:\n"
//            << "        example_sfm_scene_reconstruction <path_to_file> <f> <cx> <cy>\n"
//            << " where: path_to_file is the file absolute path into your system which contains\n"
//            << "        the list of images to use for reconstruction. \n"
//            << "        f  is the focal lenght in pixels. \n"
//            << "        cx is the image principal point x coordinates in pixels. \n"
//            << "        cy is the image principal point y coordinates in pixels. \n"
//            << "------------------------------------------------------------------------------------\n\n"
//            << endl;
//}
//int getdir(const string _filename, vector<string> &files)
//{
//    ifstream myfile(_filename.c_str());
//    if (!myfile.is_open()) {
//        cout << "Unable to read file: " << _filename << endl;
//        exit(0);
//    } else {;
//        size_t found = _filename.find_last_of("/\\");
//        string line_str, path_to_file = _filename.substr(0, found);
//        while ( getline(myfile, line_str) )
//            files.push_back(path_to_file+string("/")+line_str);
//    }
//    return 1;
//}
//int main(int argc, char* argv[])
//{
//    // Read input parameters
//    if ( argc != 5 )
//    {
//        help();
//        exit(0);
//    }
//    // Parse the image paths
//    vector<string> images_paths;
//    getdir( argv[1], images_paths );
//    // Build instrinsics
//    float f  = atof(argv[2]),
//            cx = atof(argv[3]), cy = atof(argv[4]);
//    Matx33d K = Matx33d( f, 0, cx,
//                         0, f, cy,
//                         0, 0,  1);
//    bool is_projective = true;
//    vector<Mat> Rs_est, ts_est, points3d_estimated;
//    reconstruct(images_paths, Rs_est, ts_est, K, points3d_estimated, is_projective);
//    // Print output
//    cout << "\n----------------------------\n" << endl;
//    cout << "Reconstruction: " << endl;
//    cout << "============================" << endl;
//    cout << "Estimated 3D points: " << points3d_estimated.size() << endl;
//    cout << "Estimated cameras: " << Rs_est.size() << endl;
//    cout << "Refined intrinsics: " << endl << K << endl << endl;
//    cout << "3D Visualization: " << endl;
//    cout << "============================" << endl;
//    viz::Viz3d window("Coordinate Frame");
//    window.setWindowSize(Size(500,500));
//    window.setWindowPosition(Point(150,150));
//    window.setBackgroundColor(); // black by default
//    // Create the pointcloud
//    cout << "Recovering points  ... ";
//    // recover estimated points3d
//    vector<Vec3f> point_cloud_est;
//    for (int i = 0; i < points3d_estimated.size(); ++i)
//        point_cloud_est.push_back(Vec3f(points3d_estimated[i]));
//    cout << "[DONE]" << endl;
//    cout << "Recovering cameras ... ";
//    vector<Affine3d> path;
//    for (size_t i = 0; i < Rs_est.size(); ++i)
//        path.push_back(Affine3d(Rs_est[i],ts_est[i]));
//    cout << "[DONE]" << endl;
//    if ( point_cloud_est.size() > 0 )
//    {
//        cout << "Rendering points   ... ";
//        viz::WCloud cloud_widget(point_cloud_est, viz::Color::green());
//        window.showWidget("point_cloud", cloud_widget);
//        cout << "[DONE]" << endl;
//    }
//    else
//    {
//        cout << "Cannot render points: Empty pointcloud" << endl;
//    }
//    if ( path.size() > 0 )
//    {
//        cout << "Rendering Cameras  ... ";
//        window.showWidget("cameras_frames_and_lines", viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1, viz::Color::green()));
//        window.showWidget("cameras_frustums", viz::WTrajectoryFrustums(path, K, 0.1, viz::Color::yellow()));
//        window.setViewerPose(path[0]);
//        cout << "[DONE]" << endl;
//    }
//    else
//    {
//        cout << "Cannot render the cameras: Empty path" << endl;
//    }
//    cout << endl << "Press 'q' to close each windows ... " << endl;
//    window.spin();
//    return 0;
//}
#include <opencv2/core.hpp>
#include <opencv-3.2.0-dev/opencv/cv.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
    cv::Mat leftImage = cv::imread("/home/chentao/Desktop/stereo_images/left_0.jpg");
    cv::Mat rightImage = cv::imread("/home/chentao/Desktop/stereo_images/right_0.jpg");
    // Load settings related to stereo calibration
    cv::FileStorage fsSettings("/home/chentao/software/ORB_SLAM2/Examples/Stereo/logitech.yaml",
                               cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
       rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        std::cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << std::endl;
        return -1;
    }
    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    cv::Mat imLeft, imRight;
    cv::remap(leftImage,imLeft,M1l,M2l,cv::INTER_LINEAR);
    cv::remap(rightImage,imRight,M1r,M2r,cv::INTER_LINEAR);
    cv::namedWindow("Ori_left", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Ori_right", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Undistort_left", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Undistort_right", cv::WINDOW_AUTOSIZE);
    cv::imshow("Ori_left", leftImage);
    cv::imshow("Ori_right", rightImage);
    cv::imshow("Undistort_left", imLeft);
    cv::imshow("Undistort_right", imRight);
    cv::waitKey(0);


}