#include <cassert>
#include "pointcloud2bev.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

constexpr int max_corner_num = 20;
constexpr int redius = 4;
int main(int argc, char* argv[]) {
    assert(argc == 2);
    auto cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<PointT>(argv[1], *cloud_ptr);
 
    FocusRange range = {-30.0, 30.0, -30.0, 30.0, -0.4, 0.4, 0.02};
    PointCloudImgConvert convert(range);
    auto img = convert.pointCloud2Bev(cloud_ptr);

    std::vector<cv::Point2f> corners;
    double quality_level = 0.01;
    double min_dist = 5;
    int block_size = 3;
    double k = 0.04;

    cv::goodFeaturesToTrack(img, corners, max_corner_num, quality_level, min_dist, cv::noArray(), block_size, false, k);
    std::cout << corners.size() << '\n';
    for(auto pt : corners) {
        cv::circle(img, pt, redius, cv::Scalar(255));
    }
    
    cv::namedWindow("corner_detection", CV_WINDOW_NORMAL);
    cv::resizeWindow("corner_detection", 1280, 720);
    cv::imshow("corner_detection",img);
    cv::waitKey(0);

    return 0;
}
