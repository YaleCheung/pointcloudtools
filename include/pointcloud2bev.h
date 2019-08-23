#ifndef POINTCLOUDIMGCONVERT_HHH
#define POINTCLOUDIMGCONVERT_HHH

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <boost/make_shared.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>


#include <cassert>

// assumption
// point cloud: x->forward, y->right, z->up
// img: x->right, y -> down, origin -> up left
using PointT = pcl::PointXYZ;

typedef struct FocusRange {
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
    float resolution;
} FocusRange;

class PointCloudImgConvert {
public:
    PointCloudImgConvert(const FocusRange& range) : 
        _focus_range(range) {
        assert(range.x_max > range.x_min 
            && range.y_max > range.y_min
            && range.z_max > range.z_min
            && range.resolution > 0);
    }

    cv::Mat pointCloud2Bev(pcl::PointCloud<PointT>::Ptr cloud) {
        int rows = 1 + static_cast<int>((_focus_range.y_max - _focus_range.y_min) / _focus_range.resolution);
        int cols = 1 + static_cast<int>((_focus_range.x_max - _focus_range.x_min) / _focus_range.resolution);
        
        cv::Mat img(rows, cols, CV_8UC1, cv::Scalar(0));

        for(const auto& pt : *cloud) {
            if (! _isValidPoint(pt))
                continue;
            int x_img =  (pt.y - _focus_range.y_min) / _focus_range.resolution;
            int y_img = -(pt.x - _focus_range.x_max) / _focus_range.resolution;
            int value = static_cast<int>((pt.z - _focus_range.z_min)* 255.0 / (_focus_range.z_max - _focus_range.z_min));
            img.at<uchar>(x_img, y_img) = value;
        }
        return img;
    }

private:
    bool _isValidPoint(const PointT& pt) {
        if (pt.x >= _focus_range.x_min && pt.x <= _focus_range.x_max 
         && pt.y >= _focus_range.y_min && pt.y <= _focus_range.y_max
         && pt.z >= _focus_range.z_min && pt.z <= _focus_range.z_max)
            return true;
        return false;
    }
    const FocusRange        _focus_range;
};

#endif // POINTCLOUDIMGCONVERT_HHH 
