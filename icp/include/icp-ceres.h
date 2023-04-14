#ifndef ICPCERES
#define ICPCERES

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/types.h>

#include <ceres/rotation.h>
#include "eigen_quaternion.h"
#include "sophus_se3.h"

using namespace Eigen;
using namespace std;

namespace ICP_Ceres {

template <typename T>
ceres::MatrixAdapter<T, 1, 4> ColumnMajorAdapter4x3(T* pointer) {
  return ceres::MatrixAdapter<T, 1, 4>(pointer);
}
    Isometry3d solve_pts_to_plane(
        array<vector<Vector4d>,3> points,
        vector<Vector4d> planes,
        vector<int> plane_indexes,
        vector<Quaterniond> qwc,
        vector<Vector3d> twc,
        bool automaticDiffLocalParam=true
    );
}

namespace ICPCostFunctions{

struct Pts2Plane{
    const Eigen::Vector4d& p_pt;
    const Eigen::Vector4d& p_plane;

    Pts2Plane(const Eigen::Vector4d &pt, const Eigen::Vector4d &plane) :
        p_pt(pt), p_plane(plane)
    {
    }
    static ceres::CostFunction* Create(const Eigen::Vector4d &pt, const Eigen::Vector4d &plane) {
        return (new ceres::AutoDiffCostFunction<Pts2Plane, 1, 7>(new Pts2Plane(pt, plane)));
    }

    template <typename T>
    bool operator()(const T* const cam1, T* residuals) const {

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> p;
        p << T(p_pt[0]), T(p_pt[1]), T(p_pt[2]);

        // Map the T* array to an Sophus SE3 object (with appropriate Scalar type)
        Sophus::SE3Group<T> q = Eigen::Map< const Sophus::SE3Group<T> >(cam1);

        // Rotate the point using Eigen rotations
        p = q.unit_quaternion() * p + q.translation();

        // The error is the difference between the predicted and observed position.
        residuals[0] = p[0]*p_plane[0] + p[1]*p_plane[1] + p[2]*p_plane[2] + 1*p_plane[3] ;

        return true;
    }
};

}
#endif // ICPCERES

