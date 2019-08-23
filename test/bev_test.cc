#include <cassert>
#include "pointcloud2bev.h"

int main(int argc, char* argv[]) {
    assert(argc == 2);
    auto cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::loadPCDFile<PointT>(argv[1], *cloud_ptr);
 
    FocusRange range = {-30.0, 30.0, -30.0, 30.0, -0.2, 0.3, 0.02};
    PointCloudImgConvert convert(range);
    auto img = convert.pointCloud2Bev(cloud_ptr);
    cv::imwrite("example.jpg", img);
    cv::namedWindow("bev", CV_WINDOW_NORMAL);
    cv::resizeWindow("bev", 1280, 720);
    cv::imshow("bev",img);
    cv::waitKey(0);
    return 0;
}

