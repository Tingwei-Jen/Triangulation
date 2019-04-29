#pragma once
#ifndef VISUALIZE_H
#define VISUALIZE_H
#include "mappoint.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <opencv2/core/core.hpp>

class Visualize
{
public:
	static void VisualizeMapPoint(MapPoint* mappoint);

private:
	static pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud(std::vector<cv::Point3f> pts3D);
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloudColor(std::vector<cv::Point3f> pts3D, std::vector<cv::Vec3b> pts_color);
	static boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	static boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::vector<cv::Mat> Rcws, std::vector<cv::Mat> tcws);
};
#endif //VISUALIZE_H