#include "localmapping.h"
#include "converter.h"
#include "optimizer.h"
#include <cstdlib> /* 亂數相關函數 */
#include <ctime>   /* 時間相關函數 */
using namespace cv;
using namespace std;

LocalMapping::LocalMapping(Map* map, const cv::Mat& K, 
                        const std::vector<cv::Mat>& imgs, 
                        const std::vector<cv::Mat>& Tcws,
                        const std::vector<std::vector<cv::Point2f>>& verticess)
:mK(K.clone()),fx(K.at<float>(0,0)), fy(K.at<float>(1,1)), cx(K.at<float>(0,2)), cy(K.at<float>(1,2))
{
	this->mMap = map;
    this->mNframes = imgs.size();
    this->mNvertices = verticess[0].size();

    for(int i=0; i<this->mNframes; i++)
    {
        this->mImgs.push_back(imgs[i].clone());
        this->mTcws.push_back(Tcws[i].clone());
        this->mVerticess.push_back(verticess[i]);   
    }
}

void LocalMapping::Run()
{

    //Create frames
    vector<Frame*> Frames;
    for(int i=0; i<this->mNframes; i++)
    {
        Frame* frame = new Frame(this->mImgs[i], this->mK, this->mVerticess[i]);
        frame->SetPose(this->mTcws[i]);
        Frames.push_back(frame);
    }

    //Triangulation  
    vector<Mat> _Tcws; 
    for(int i=0; i<this->mNframes; i++)
    {
        _Tcws.push_back(Frames[i]->GetPose());
    }

    // SEPERATE EVERY VERTICE
    for(int i=0; i<this->mNvertices; i++)
    {
        //pts_cam
        vector<Point3f> pts_cam;
        for(int j=0; j<this->mNframes; j++)
        {
            pts_cam.push_back(Px2Cam(Frames[j]->GetVertices()[i], this->fx, this->fy, this->cx, this->cy));
        }

        //Triangulation
        Point3f x3Dp;
        bool ret = Triangulation(_Tcws, pts_cam, x3Dp);
        if(!ret) return; 

        //Triangulation Result
        float avg_dist;
        float avg_px_error;
        ComputeError(Frames, i, x3Dp, avg_dist, avg_px_error);
        cout<<"vertice: "<<i<<"   "<<"x3Dp: "<<x3Dp<<"   "<<"avg_dist: "<<avg_dist<<"   "<<"avg_px_error: "<<avg_px_error<<endl;

        //BA
        vector<Point2f> pxs;
        for(int j=0; j<this->mNframes; j++)
        {
            pxs.push_back(Frames[j]->GetVertices()[i]);
        }
        vector<Mat> New_Tcws; 
        Point3f New_pt3d;
        Optimizer::BundleAdjustment(_Tcws, x3Dp, pxs, this->fx, this->cx, this->cy, New_Tcws, New_pt3d, 10);
    
        //Update Frames
        for(int j=0; j<this->mNframes; j++)
        {
            Frames[j]->SetPose(New_Tcws[j]);
        }

        //BA Result
        float avg_dist_ba;
        float avg_px_error_ba;
        ComputeError(Frames, i, New_pt3d, avg_dist_ba, avg_px_error_ba);
        cout<<"vertice: "<<i<<"   "<<"x3Dp: "<<New_pt3d<<"   "<<"avg_dist_ba: "<<avg_dist_ba<<"   "<<"avg_px_error_ba: "<<avg_px_error_ba<<endl;

        //Create Mappoint
        MapPoint* mappoint = new MapPoint();
        mappoint->SetWorldPose(New_pt3d);

        for(int j=0; j<this->mNframes; j++)
        {
            mappoint->AddObservation(Frames[j], Frames[j]->GetVertices()[i]);
        }

        this->mMap->AddMapPoint(mappoint);
    }
}

void LocalMapping::RunAllBA()
{

    //Create frames
    vector<Frame*> Frames;
    for(int i=0; i<this->mNframes; i++)
    {
        Frame* frame = new Frame(this->mImgs[i], this->mK, this->mVerticess[i]);
        frame->SetPose(this->mTcws[i]);
        Frames.push_back(frame);
    }

    //_Tcws
    vector<Mat> _Tcws; 
    for(int i=0; i<this->mNframes; i++)
    {
        _Tcws.push_back(Frames[i]->GetPose());
    }

    //Triangulation
    vector<Point3f> pts3D;
    for(int i=0; i<this->mNvertices; i++)
    {
        //pts_cam
        vector<Point3f> pts_cam;
        for(int j=0; j<this->mNframes; j++)
        {
            pts_cam.push_back(Px2Cam(Frames[j]->GetVertices()[i], this->fx, this->fy, this->cx, this->cy));
        }

        //Triangulation
        Point3f x3Dp;
        bool ret = Triangulation(_Tcws, pts_cam, x3Dp);
        if(!ret) return; 
        pts3D.push_back(x3Dp);

    }
        
    //Triangulation Result
    vector<float> avg_dists; 
    vector<float> avg_px_errors;
    ComputeError(Frames, pts3D, avg_dists, avg_px_errors);

    for(int i=0; i<this->mNvertices; i++)
        cout<<"vertice: "<<i<<"   "<<"x3Dp: "<<pts3D[i]<<"   "<<"avg_dist: "<<avg_dists[i]<<"   "<<"avg_px_error: "<<avg_px_errors[i]<<endl;
        

    // //Create Mappoint
    // for(int i=0; i<this->mNvertices; i++)
    // {
    //     MapPoint* mappoint = new MapPoint();
    //     mappoint->SetWorldPose(pts3D[i]);
        
    //     for(int j=0; j<this->mNframes; j++)
    //     {
    //         mappoint->AddObservation(Frames[j], Frames[j]->GetVertices()[i]);
    //     }
        
    //     this->mMap->AddMapPoint(mappoint);
    // }



    //BA Together
    vector<vector<Point2f>> pxs;
    for(int i=0; i<this->mNframes; i++)
    {
        vector<Point2f> px;
        for(int j=0; j<this->mNvertices; j++)
        {
            px.push_back(Frames[i]->GetVertices()[j]);
        }
        pxs.push_back(px);
    }
    vector<Mat> New_Tcws;
    vector<Point3f> New_pts3d;
    Optimizer::BundleAdjustment(_Tcws, pts3D, pxs, this->fx, this->cx, this->cy, New_Tcws, New_pts3d, 10);

    //Update Frames
    for(int i=0; i<this->mNframes; i++)
    {
        Frames[i]->SetPose(New_Tcws[i]);
    }

    //BA Result
    vector<float> avg_dists_BA; 
    vector<float> avg_px_errors_BA;
    ComputeError(Frames, New_pts3d, avg_dists_BA, avg_px_errors_BA);

    for(int i=0; i<this->mNvertices; i++)
        cout<<"vertice: "<<i<<"   "<<"x3Dp: "<<New_pts3d[i]<<"   "<<"avg_dist_BA: "<<avg_dists_BA[i]<<"   "<<"avg_px_error_BA: "<<avg_px_errors_BA[i]<<endl;


    //Create Mappoint
    for(int i=0; i<this->mNvertices; i++)
    {
        MapPoint* mappoint = new MapPoint();
        mappoint->SetWorldPose(New_pts3d[i]);
        
        for(int j=0; j<this->mNframes; j++)
        {
            mappoint->AddObservation(Frames[j], Frames[j]->GetVertices()[i]);
        }
        
        this->mMap->AddMapPoint(mappoint);
    }
}

void LocalMapping::RunRansacAllBA(float avg_dist_thres, int nIterations)
{

    //create frames
    vector<Frame*> Frames;
    for(int i=0; i<this->mNframes; i++)
    {
        Frame* frame = new Frame(this->mImgs[i], this->mK, this->mVerticess[i]);
        frame->SetPose(this->mTcws[i]);
        Frames.push_back(frame);
    }


    //RANSAC
    vector<int> v_n_inliers;              //nIterations*1
    vector<vector<Point3f>> v_pts3ds;      //nIterations*mNvertices
    int cnt = 0;
    while(cnt<nIterations)
    {
        //Random choose two
        int n=2; 
        int arr[n];
        RandomNumbers(0, this->mNframes-1, n, cnt, arr);

        //Tcws
        vector<Mat> _Tcws_n; 
        for(int i=0; i<n; i++)
        {
            _Tcws_n.push_back(Frames[arr[i]]->GetPose());
        }

        //pts_cam
        vector<vector<Point3f>> pts_cams_n;
        for(int i=0; i<this->mNvertices; i++)
        {
            vector<Point3f> pts_cam_n;
            for(int j=0; j<n; j++)
            {
                pts_cam_n.push_back(Px2Cam(Frames[arr[j]]->GetVertices()[i], this->fx, this->fy, this->cx, this->cy));
            }
            pts_cams_n.push_back(pts_cam_n);
        }

        //Triangulation
        vector<Point3f> pts3d;
        for(int i=0; i<this->mNvertices; i++)
        {
            Point3f x3Dp;
            bool ret = Triangulation(_Tcws_n, pts_cams_n[i], x3Dp);
            if(!ret) continue; 
            pts3d.push_back(x3Dp);
        }

        //Counting number of inliers
        int n_inliers = 0;
        for(int i=0; i<this->mNframes; i++)
        {
            float avg_dist = 0.0;
            for(int j=0; j<this->mNvertices; j++)
            {
                avg_dist += Dist3Dpoint2Pxline(pts3d[j], Frames[i], j);
            }
            avg_dist/=this->mNvertices;

            if(avg_dist<avg_dist_thres)
                n_inliers++;        
        }

        //Record
        v_n_inliers.push_back(n_inliers);
        vector<Point3f> v_pts3d;
        for(int j=0; j<this->mNvertices; j++)
        {
            v_pts3d.push_back(pts3d[j]);
        }
        v_pts3ds.push_back(v_pts3d);
        cnt++;
    }

    //Find pts3d with max inliers
    int max_value = 0;
    vector<Point3f> pts3d_max_inliers;
    for(int i=0; i<nIterations; i++)
    {
        if( max_value < v_n_inliers[i] )
        {   
            max_value = v_n_inliers[i];
            pts3d_max_inliers = v_pts3ds[i];
        }
    }
    cout<<"max_n_inliers: "<<max_value<<endl;
    cout<<"pts3d_max_inliers:  ";
    for(int i=0; i<this->mNvertices; i++)
    {
        cout<<pts3d_max_inliers[i]<<"  ";
    }
    cout<<endl;
    

    //Inliers frames
    vector<Frame*> Frames_inliers;
    for(int i=0; i<this->mNframes; i++)
    {
        float avg_dist = 0.0;
        for(int j=0; j<this->mNvertices; j++)
        {
            avg_dist += Dist3Dpoint2Pxline(pts3d_max_inliers[j], Frames[i], j);
        }
        avg_dist/=this->mNvertices;

        if(avg_dist<avg_dist_thres)
        {
            Frames_inliers.push_back(Frames[i]);
        }
    }


    //Triangulation with inliers
    int n_frames_inliers = Frames_inliers.size();
    vector<Mat> _Tcws; 
    for(int i=0; i<n_frames_inliers; i++)
    {
        _Tcws.push_back(Frames_inliers[i]->GetPose());
    }

    vector<Point3f> pts3D;
    for(int i=0; i<this->mNvertices; i++)
    {
        //pts_cam
        vector<Point3f> pts_cam;
        for(int j=0; j<n_frames_inliers; j++)
        {
            pts_cam.push_back(Px2Cam(Frames_inliers[j]->GetVertices()[i], this->fx, this->fy, this->cx, this->cy));
        }

        //Triangulation
        Point3f x3Dp;
        bool ret = Triangulation(_Tcws, pts_cam, x3Dp);
        if(!ret) return; 
        pts3D.push_back(x3Dp);
    }


    //Triangulation Result
    vector<float> avg_dists; 
    vector<float> avg_px_errors;
    ComputeError(Frames_inliers, pts3D, avg_dists, avg_px_errors);

    

    for(int i=0; i<this->mNvertices; i++)
        cout<<"vertice: "<<i<<"   "<<"x3Dp: "<<pts3D[i]<<"   "<<"avg_dist: "<<avg_dists[i]<<"   "<<"avg_px_error: "<<avg_px_errors[i]<<endl;
        

    //BA Together
    vector<vector<Point2f>> pxs;
    for(int i=0; i<n_frames_inliers; i++)
    {
        vector<Point2f> px;
        for(int j=0; j<this->mNvertices; j++)
        {
            px.push_back(Frames_inliers[i]->GetVertices()[j]);
        }
        pxs.push_back(px);
    }
    vector<Mat> New_Tcws;
    vector<Point3f> New_pts3d;
    Optimizer::BundleAdjustment(_Tcws, pts3D, pxs, this->fx, this->cx, this->cy, New_Tcws, New_pts3d, 10);

   

    //Update Frames
    for(int i=0; i<n_frames_inliers; i++)
    {
        Frames_inliers[i]->SetPose(New_Tcws[i]);
    }

    //BA Result
    vector<float> avg_dists_BA; 
    vector<float> avg_px_errors_BA;
    ComputeError(Frames_inliers, New_pts3d, avg_dists_BA, avg_px_errors_BA);

    for(int i=0; i<this->mNvertices; i++)
        cout<<"vertice: "<<i<<"   "<<"x3Dp: "<<New_pts3d[i]<<"   "<<"avg_dist_BA: "<<avg_dists_BA[i]<<"   "<<"avg_px_error_BA: "<<avg_px_errors_BA[i]<<endl;


    //Create Mappoint
    for(int i=0; i<this->mNvertices; i++)
    {
        MapPoint* mappoint = new MapPoint();
        mappoint->SetWorldPose(New_pts3d[i]);
        
        for(int j=0; j<n_frames_inliers; j++)
        {
            mappoint->AddObservation(Frames_inliers[j], Frames_inliers[j]->GetVertices()[i]);
        }
        
        this->mMap->AddMapPoint(mappoint);
    }

}


/*
private
*/
// |xp2  - p0 |     |0|
// |yp2  - p1 | X = |0| ===> AX = 0
// |x'p2'- p0'|     |0|
// |y'p2'- p1'|     |0|
bool LocalMapping::Triangulation(const std::vector<cv::Mat>& Tcws, const std::vector<cv::Point3f>& pts_cam, cv::Point3f& x3Dp)
{
    int n_frames = Tcws.size();

    Mat A(n_frames*2, 4, CV_32F);
   
    for(int i=0; i<n_frames; i++)
    {
        A.row(i*2) = pts_cam[i].x*Tcws[i].row(2)-Tcws[i].row(0);
        A.row(i*2+1) = pts_cam[i].y*Tcws[i].row(2)-Tcws[i].row(1);
    }
    
    Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    Mat x3D = vt.row(3).t();

    if(x3D.at<float>(3) == 0)
        return false;
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);  

    // check triangulation in front of camera.
    for(int i=0; i<n_frames; i++)
    {
        Mat Rcw, tcw;
        Tcws[i].rowRange(0,3).colRange(0,3).copyTo(Rcw);
        Tcws[i].rowRange(0,3).col(3).copyTo(tcw);

        Mat x3Dt = x3D.t();

        float z = Rcw.row(2).dot(x3Dt)+tcw.at<float>(2);
        if(z<=0)
            return false;
    }

    x3Dp = Point3f(x3D.at<float>(0), x3D.at<float>(1), x3D.at<float>(2));
    return true;
}

void LocalMapping::ComputeError(vector<Frame*> frames, const int& idx, const Point3f& pt3d, float& avg_dist, float& avg_px_error)
{
    float total_dist = 0.0;
    float total_px_error = 0.0;

    for(int i=0; i<frames.size(); i++)
    {
        Mat img = frames[i]->GetImg();
        float dist = Dist3Dpoint2Pxline(pt3d, frames[i], idx);
        total_dist += dist;
        Point2f px = frames[i]->GetVertices()[idx];
        Point2f px_repro = Cam2Px(World2Cam(pt3d, frames[i]->GetPose()), this->fx, this->fy, this->cx, this->cy);
        total_px_error += norm(px-px_repro);
        cv::circle(img, px, 2, cv::Scalar(255,0,0), -1);
        cv::circle(img, px_repro, 2, cv::Scalar(0,0,255), -1);
        imshow("img", img);
        waitKey(0);
    }
    avg_dist = total_dist/frames.size();
    avg_px_error = total_px_error/frames.size();
}

void LocalMapping::ComputeError(vector<Frame*> frames, const vector<Point3f>& pts3d, 
                                vector<float>& avg_dists, vector<float>& avg_px_errors)
{
    vector<float> total_dists(pts3d.size(), 0.0);
    vector<float> total_px_errors(pts3d.size(), 0.0);

    for(int i=0; i<frames.size(); i++)
    {
        Mat img = frames[i]->GetImg();
        for(int j=0; j<pts3d.size(); j++)
        {
            float dist = Dist3Dpoint2Pxline(pts3d[j], frames[i], j);
            total_dists[j] += dist;

            Point2f px = frames[i]->GetVertices()[j];
            Point2f px_repro = Cam2Px(World2Cam(pts3d[j], frames[i]->GetPose()), this->fx, this->fy, this->cx, this->cy);
            total_px_errors[j] += norm(px-px_repro);
            cv::circle(img, px, 2, cv::Scalar(255,0,0), -1);
            cv::circle(img, px_repro, 2, cv::Scalar(0,0,255), -1);
        }
        imshow("img", img);
        waitKey(0);
    }

    avg_dists.clear();
    avg_px_errors.clear();

    for(int i=0; i<pts3d.size(); i++)
    {
        avg_dists.push_back(total_dists[i]/frames.size());
        avg_px_errors.push_back(total_px_errors[i]/frames.size());
    }
}

float LocalMapping::Dist3Dpoint2Pxline(const cv::Point3f& x3Dp, Frame* frame, const int& index)
{
    Point3f cam_center = Converter::toCvPoint3f(frame->GetCameraCenter()); 
    Point3f px_world = Cam2World(Px2Cam(frame->GetVertices()[index], this->fx, this->fy, this->cx, this->cy), frame->GetPose().inv());

    Point3f f = px_world - cam_center;
    Point3f a = x3Dp - cam_center;
    
    float t = (f.x * a.x + f.y * a.y + f.z * a.z) / (f.x * f.x + f.y * f.y + f.z * f.z);
    Point3f A = cam_center + t*f;
    return norm(x3Dp-A);
}

void LocalMapping::RandomNumbers(const int& min, const int& max, const int& n, const int& nIter, int arr[])
{
    /* 亂數種子 */
    srand( nIter );

    int cnt = 0;

    while(cnt<n)
    {
        int x = rand() % (max - min + 1) + min;

        bool found = false;
        for(int i=0; i<cnt; i++)
        {
            if(arr[i]==x)
            {
                found = true;
                break;
            }
        }

        if(found)
            continue;

        arr[cnt] = x;
        cnt++;
    }
}


cv::Point3f LocalMapping::Px2Cam(const cv::Point2f& px, const float& fx, const float& fy, const float& cx, const float& cy)
{
    Point3f p_cam = Point3f((px.x-cx)/fx,(px.y-cy)/fy,1);
    return p_cam;
}

cv::Point2f LocalMapping::Cam2Px(const cv::Point3f& p_cam, const float& fx, const float& fy, const float& cx, const float& cy)
{
    Point2f px = Point2f(p_cam.x*fx/p_cam.z + cx, p_cam.y*fy/p_cam.z + cy);
    return px;
}

cv::Point3f LocalMapping::World2Cam(const cv::Point3f& p_world, const cv::Mat& Tcw)
{
    Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    Mat tcw = Tcw.rowRange(0,3).col(3);
    Mat p_cam_ = Rcw*Converter::toCvMat(p_world) + tcw;
    Point3f p_cam = Converter::toCvPoint3f(p_cam_);
    return p_cam;
}

cv::Point3f LocalMapping::Cam2World(const cv::Point3f& p_cam, const cv::Mat& Twc)
{
    Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
    Mat twc = Twc.rowRange(0,3).col(3);
    Mat p_world_ = Rwc*Converter::toCvMat(p_cam) + twc;
    Point3f p_world = Converter::toCvPoint3f(p_world_);
    return p_world;
}

