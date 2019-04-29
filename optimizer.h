#pragma once
#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include <opencv2/opencv.hpp>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
//#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"

class Optimizer
{
public:
    static void BundleAdjustment(const cv::Mat& Tcw1,
                                const cv::Mat& Tcw2, 
                                const std::vector<cv::Point3f>& pts3d, 
                                const std::vector<cv::Point2f>& pts1, 
                                const std::vector<cv::Point2f>& pts2, 
                                cv::Mat& New_Tcw2,
                                std::vector<cv::Point3f>& New_pts3d, 
                                int nIterations = 10);

    static void BundleAdjustment(const std::vector<cv::Mat>& Tcws, 
                                const std::vector<cv::Point3f>& pts3d, 
                                const std::vector<std::vector<cv::Point2f>>& pxs,
                                const float fx, const float cx, const float cy,
                                std::vector<cv::Mat>& New_Tcws,
                                std::vector<cv::Point3f>& New_pts3d,
                                int nIterations = 10);

    static void BundleAdjustment(const std::vector<cv::Mat>& Tcws, 
                                const cv::Point3f& pt3d, 
                                const std::vector<cv::Point2f>& pxs,
                                const float fx, const float cx, const float cy,
                                std::vector<cv::Mat>& New_Tcws,
                                cv::Point3f& New_pt3d,
                                int nIterations = 10);

};
#endif //OPTIMIZER_H