#include "visualize.h"
#include "converter.h"
using namespace cv;
using namespace std;

void Visualize::VisualizeMapPoint(MapPoint* mappoint)
{

    vector<Point3f> pts3D;
    vector<Vec3b> pts_color;
    vector<Mat> Rcws;
    vector<Mat> tcws;

    //Observed frames
    std::vector<Frame*> Frames = mappoint->GetObservationFrames();    
    for(int i=0; i<Frames.size(); i++)
    {
        pts3D.push_back(Converter::toCvPoint3f(Frames[i]->GetCameraCenter()));
        pts_color.push_back(Vec3b(0,255,0));
        Rcws.push_back(Frames[i]->GetRotation());
        tcws.push_back(Frames[i]->GetTranslation());
    }

    //mappoint.
    pts3D.push_back(mappoint->GetWorldPose());    
    pts_color.push_back(Vec3b(255,255,255));

    //viewer
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = generatePointCloudColor(pts3D, pts_color);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = rgbVis(point_cloud_ptr, Rcws, tcws);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Visualize::generatePointCloud(std::vector<cv::Point3f> pts3D)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

	for(int i=0; i<pts3D.size(); i++)
	{
	    pcl::PointXYZ basic_point;
	    basic_point.x = pts3D[i].x;
	    basic_point.y = pts3D[i].y;
	    basic_point.z = pts3D[i].z;
	    basic_cloud_ptr->points.push_back(basic_point);
	}

  	basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  	basic_cloud_ptr->height = 1;

	return basic_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Visualize::generatePointCloudColor(std::vector<cv::Point3f> pts3D, std::vector<cv::Vec3b> pts_color)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

	for(int i=0; i<pts3D.size(); i++)
    {
		uchar blue = pts_color[i].val[0];
	    uchar green = pts_color[i].val[1];
	    uchar red = pts_color[i].val[2];

		pcl::PointXYZRGB point;
		point.x = pts3D[i].x;
		point.y = pts3D[i].y;
		point.z = pts3D[i].z;
        point.r = red;
        point.g = green;
        point.b = blue;

        point_cloud_ptr->points.push_back (point);
	}

  	point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  	point_cloud_ptr->height = 1;

	return point_cloud_ptr;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualize::simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();
    return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualize::rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::vector<cv::Mat> Rcws, std::vector<cv::Mat> tcws)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();

    // ----------------------------------------------------
    // -----add camera cooridinate and sign coordinate-----
    // ----------------------------------------------------
    for(int i=0; i<Rcws.size(); i++)
    {
        pcl::PointXYZ origin;
        pcl::PointXYZ x_axis, y_axis, z_axis;
        std::string x_axis_n = "x_axis" + std::to_string(i);
        std::string y_axis_n = "y_axis" + std::to_string(i);
        std::string z_axis_n = "z_axis" + std::to_string(i);

        cv::Mat Rwc = Rcws[i].t();
        cv::Mat Ow = -Rwc*tcws[i];

        origin.x = Ow.at<float>(0,0);
        origin.y = Ow.at<float>(1,0);
        origin.z = Ow.at<float>(2,0);

        x_axis.x = Ow.at<float>(0,0) + Rwc.at<float>(0,0)/5.0;
        x_axis.y = Ow.at<float>(1,0) + Rwc.at<float>(1,0)/5.0;
        x_axis.z = Ow.at<float>(2,0) + Rwc.at<float>(2,0)/5.0;

        y_axis.x = Ow.at<float>(0,0) + Rwc.at<float>(0,1)/5.0;
        y_axis.y = Ow.at<float>(1,0) + Rwc.at<float>(1,1)/5.0;
        y_axis.z = Ow.at<float>(2,0) + Rwc.at<float>(2,1)/5.0;

        z_axis.x = Ow.at<float>(0,0) + Rwc.at<float>(0,2)/5.0;
        z_axis.y = Ow.at<float>(1,0) + Rwc.at<float>(1,2)/5.0;
        z_axis.z = Ow.at<float>(2,0) + Rwc.at<float>(2,2)/5.0;

        viewer->addLine<pcl::PointXYZ, pcl::PointXYZ> (origin, x_axis, 255, 0.0, 0.0, x_axis_n);   //rgb
        viewer->addLine<pcl::PointXYZ, pcl::PointXYZ> (origin, y_axis, 0.0, 255, 0.0, y_axis_n);
        viewer->addLine<pcl::PointXYZ, pcl::PointXYZ> (origin, z_axis, 0.0, 0.0, 255, z_axis_n);
    }

    return (viewer);
}