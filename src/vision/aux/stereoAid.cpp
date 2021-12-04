#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


using pclPoint = pcl::PointXYZRGB;
using pclCloud = pcl::PointCloud<pclPoint>;

/* cv::Mat defineQ(int img_width, int img_height, float Tx, float f)
{
    // Initialize certain variables:
    float I_w_term {(float)-img_width/(float)2.0};
    float I_h_term {(float)-img_height/(float)2.0};
    float translation_X {(float)-1/Tx};
    float data [16] = {1, 0, 0, I_w_term, 0, 1, 0, I_h_term, 0, 0, 0, f, 0, 0, translation_X, 0};

    // Define Q matrix as per document
    cv::Mat matQ = cv::Mat(4,4, CV_32F, data);

    return matQ;
}
 */
cv::Mat defineQ(int img_width, int img_height)
{
    double cx = -img_width / 2.0, cy = -img_height / 2.0;
    float Tx{200}, f {img_width/tan(M_PI*50/(2.0*360.0))};

    f = 50;

	cv::Mat Q = cv::Mat::zeros(4, 4, CV_64F);
	Q.at<double>(0, 0) = f;
	Q.at<double>(1, 1) = f;
	Q.at<double>(0, 3) = cx;
	Q.at<double>(1, 3) = cy;
	Q.at<double>(2, 3) = f;
	Q.at<double>(3, 2) = -1 / Tx;
    std::cout << "Q is: " << Q << std::endl;
    return Q;
}

cv::Mat DisparityBM(cv::Mat &imgL, cv::Mat &imgR, int nDisparities, int BlockSize)
{
    cv::Mat result;

    // Compute disparity with cv::StereoBM
    cv::Ptr<cv::StereoBM> bM = cv::StereoBM::create(nDisparities, BlockSize);
    bM->compute(imgL, imgR, result);

    return result;
}

cv::Mat DisparitySGBM(cv::Mat &imgL, cv::Mat &imgR, int nDisparities, int BlockSize)
{
    cv::Mat result;

    // Compute disparity with cv::StereoSGBM
    cv::Ptr<cv::StereoSGBM> bM = cv::StereoSGBM::create(nDisparities, BlockSize);
    bM->compute(imgL, imgR, result);

    return result;
}

cv::Mat normDisparity(cv::Mat disp)
{
    cv::Mat dispNorm;
    cv::normalize(disp, dispNorm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    return dispNorm;
}

cv::Mat reproject3D(cv::Mat disp, cv::Mat Q)
{
    cv::Mat points;
    cv::reprojectImageTo3D(disp, points, Q, true);
    return points;
}


pclCloud::Ptr savePointCloud(std::string filename, cv::Mat points, cv::Mat colors, double max_z) {
    pclPoint p_default;
    pclCloud::Ptr dst(new pclCloud(points.rows, points.cols, p_default));
    for (size_t i = 0; i < points.rows; i++) {
        for (size_t j = 0; j < points.cols; j++) {
            cv::Vec3f xyz = points.at<cv::Vec3f>(i, j);
            cv::Vec3b bgr = colors.at<cv::Vec3b>(i, j);
            // Check if points are too far away, if not take them into account
            if (fabs(xyz[2]) < max_z) {
                pclPoint pn;
                pn.x = xyz[0];
                pn.y = xyz[1];
                pn.z = xyz[2];
                pn.r = bgr[2];
                pn.g = bgr[1];
                pn.b = bgr[0];
                dst->at(i, j) = pn;
            }
        }
    }
    pcl::io::savePCDFileASCII(filename, *dst);

    return dst;
}


pclCloud::Ptr obtainPointCloud(cv::Mat points, cv::Mat colors, double max_z) {
    pclPoint p_default;
    pclCloud::Ptr dst(new pclCloud(points.rows, points.cols, p_default));
    for (int i = 0; i < points.rows; i++) {
        for (int j = 0; j < points.cols; j++) {
            cv::Vec3f xyz = points.at<cv::Vec3f>(i, j);
            cv::Vec3b bgr = colors.at<cv::Vec3b>(i, j);
            // Check if points are too far away, if not take them into account
            if (fabs(xyz[2]) < max_z) {
                pclPoint pn;
                pn.x = xyz[0];
                pn.y = xyz[1];
                pn.z = xyz[2];
                pn.r = bgr[2];
                pn.g = bgr[1];
                pn.b = bgr[0];
                dst->at(i, j) = pn;
            }
        }
    }

    return dst;
}