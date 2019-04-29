#include "optimizer.h"
#include "converter.h"
using namespace std;

void Optimizer::BundleAdjustment(const cv::Mat& Tcw1,
                                const cv::Mat& Tcw2, 
                                const std::vector<cv::Point3f>& pts3d, 
                                const std::vector<cv::Point2f>& pts1, 
                                const std::vector<cv::Point2f>& pts2, 
                                cv::Mat& New_Tcw2,
                                std::vector<cv::Point3f>& New_pts3d, 
                                int nIterations)
{
    /*步骤1：初始化g2o优化器*/
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    //linear solver
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    //linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();

    //algorithm, LM
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );
    optimizer.setAlgorithm(algorithm);


    /*步骤2：准备相机参数*/
    float fx = 712.58465;
    float fy = 713.57849;
    float cx = 613.71289;
    float cy = 386.50469;

    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );


    /*步骤3：向优化器添加顶点 set vertices: */
    //camera_pose ---> g2o::VertexSE3Expmap 
    //map_points ---> g2o::VertexSBAPointXYZ
    g2o::VertexSE3Expmap* v1 = new g2o::VertexSE3Expmap();
    v1->setId(0);        
    v1->setFixed(true);   
    v1->setEstimate(Converter::toSE3Quat(Tcw1));
    optimizer.addVertex(v1);
    
    g2o::VertexSE3Expmap* v2 = new g2o::VertexSE3Expmap();
    v2->setId(1);        
    v2->setEstimate(Converter::toSE3Quat(Tcw2));
    optimizer.addVertex(v2);

    for(int i=0; i<pts3d.size(); i++)
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( 2 + i );
        v->setMarginalized(true);
        v->setEstimate(Eigen::Vector3d(pts3d[i].x,pts3d[i].y,pts3d[i].z));
        optimizer.addVertex( v );        
    }


    /*步骤4：向优化器添加投影边边 */
    //projection relationship ---> g2o::EdgeProjectXYZ2UV
    vector<g2o::EdgeProjectXYZ2UV*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
        edge->setMeasurement( Eigen::Vector2d(pts1[i].x, pts1[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }

    // 第二帧
    for ( size_t i=0; i<pts2.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
        edge->setMeasurement( Eigen::Vector2d(pts2[i].x, pts2[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0,0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }

    /*步骤5：开始优化 */
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);


    //我们比较关心两帧之间的变换矩阵
    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    g2o::SE3Quat pose = v->estimate();
    New_Tcw2 = Converter::toCvMat(pose);

    // 以及所有特征点的位置
    New_pts3d.clear();
    for ( size_t i=0; i<pts3d.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
        Eigen::Vector3d pos = v->estimate();
        New_pts3d[i] = cv::Point3f(pos(0), pos(1), pos(2));
    }

}
    
void Optimizer::BundleAdjustment(const std::vector<cv::Mat>& Tcws, 
                                const std::vector<cv::Point3f>& pts3d, 
                                const std::vector<std::vector<cv::Point2f>>& pxs,
                                const float fx, const float cx, const float cy,
                                std::vector<cv::Mat>& New_Tcws,
                                std::vector<cv::Point3f>& New_pts3d,
                                int nIterations)
{
    int n_frames = Tcws.size();
    int n_pts3d = pts3d.size();

    /*步骤1：初始化g2o优化器*/
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    //linear solver
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();

    //algorithm, LM
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );
    optimizer.setAlgorithm(algorithm);


    /*步骤2：准备相机参数*/
    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );


    /*步骤3：向优化器添加顶点 set vertices: */
    //camera_pose ---> g2o::VertexSE3Expmap 
    //map_points ---> g2o::VertexSBAPointXYZ
    for(int i=0; i<n_frames; i++)
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);           
        v->setEstimate(Converter::toSE3Quat(Tcws[i]));
        optimizer.addVertex(v);
    }

    for(int i=0; i<n_pts3d; i++)
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( n_frames + i );
        v->setMarginalized(true);
        v->setEstimate(Eigen::Vector3d(pts3d[i].x,pts3d[i].y,pts3d[i].z));
        optimizer.addVertex( v );        
    }


    /*步骤4：向优化器添加投影边边 */
    //projection relationship ---> g2o::EdgeProjectXYZ2UV
    for(int i=0; i<n_frames; i++)
    {
        for(int j=0; j<n_pts3d; j++)
        {
            g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
            edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(j+n_frames)) );
            edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(i)) );
            edge->setMeasurement( Eigen::Vector2d(pxs[i][j].x, pxs[i][j].y ) );
            edge->setInformation( Eigen::Matrix2d::Identity() );
            edge->setParameterId(0,0);
            // 核函数
            edge->setRobustKernel( new g2o::RobustKernelHuber() );
            optimizer.addEdge( edge );
        }
    }


    /*步骤5：开始优化 */
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    /*步骤6：Update poses and 3D points */
    New_Tcws.clear();
    for(int i=0; i<n_frames; i++)
    {
        g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(i) );
        g2o::SE3Quat pose = v->estimate();
        New_Tcws.push_back(Converter::toCvMat(pose));
    }
    
    New_pts3d.clear();
    for(int i=0; i<n_pts3d; i++)
    {
        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+n_frames));
        Eigen::Vector3d pos = v->estimate();
        New_pts3d.push_back(cv::Point3f(pos(0), pos(1), pos(2)));
    }

}

void Optimizer::BundleAdjustment(const std::vector<cv::Mat>& Tcws, 
                                const cv::Point3f& pt3d, 
                                const std::vector<cv::Point2f>& pxs,
                                const float fx, const float cx, const float cy,
                                std::vector<cv::Mat>& New_Tcws,
                                cv::Point3f& New_pt3d,
                                int nIterations)
{

    int n_frames = Tcws.size();

    /*步骤1：初始化g2o优化器*/
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    //linear solver
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();

    //algorithm, LM
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );
    optimizer.setAlgorithm(algorithm);


    /*步骤2：准备相机参数*/
    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );


    /*步骤3：向优化器添加顶点 set vertices: */
    //camera_pose ---> g2o::VertexSE3Expmap 
    //map_points ---> g2o::VertexSBAPointXYZ
    for(int i=0; i<n_frames; i++)
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);           
        v->setEstimate(Converter::toSE3Quat(Tcws[i]));
        optimizer.addVertex(v);
    }

    g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
    v->setId( n_frames );
    v->setMarginalized(true);
    v->setEstimate(Eigen::Vector3d(pt3d.x, pt3d.y, pt3d.z));
    optimizer.addVertex( v );        
    

    /*步骤4：向优化器添加投影边边 */
    //projection relationship ---> g2o::EdgeProjectXYZ2UV
    for(int i=0; i<n_frames; i++)
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(n_frames)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(i)) );
        edge->setMeasurement( Eigen::Vector2d(pxs[i].x, pxs[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0,0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
    }


    /*步骤5：开始优化 */
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);


    /*步骤6：Update poses and 3D points */
    New_Tcws.clear();
    for(int i=0; i<n_frames; i++)
    {
        g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(i) );
        g2o::SE3Quat pose = v->estimate();
        New_Tcws.push_back(Converter::toCvMat(pose));
    }

    g2o::VertexSBAPointXYZ* v_output = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(n_frames));
    Eigen::Vector3d pos = v_output->estimate();
    New_pt3d = cv::Point3f(pos(0), pos(1), pos(2));
}