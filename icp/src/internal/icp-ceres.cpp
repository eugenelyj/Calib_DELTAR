#include "icp-ceres.h"

#include <Eigen/Dense>
#include <math.h>
#include <unordered_map>
#include <vector>

//#include "Visualize.h"


#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/types.h>
#include <ceres/rotation.h>
#include <ceres/ceres.h>

#include <ceres/loss_function.h>

#define useLocalParam


namespace ICP_Ceres {

/*
Ceres Solving FAQ extracted from http://ceres-solver.org/solving_faqs.html:

1. For small (a few hundred parameters) or dense problems use DENSE_QR.

2. For general sparse problems (i.e., the Jacobian matrix has a substantial number of zeros) use SPARSE_NORMAL_CHOLESKY.
This requires that you have SuiteSparse or CXSparse installed.

3. For bundle adjustment problems with up to a hundred or so cameras, use DENSE_SCHUR.

4. For larger bundle adjustment problems with sparse Schur Complement/Reduced camera matrices use SPARSE_SCHUR.
This requires that you build Ceres with support for SuiteSparse, CXSparse or Eigen’s sparse linear algebra libraries.
If you do not have access to these libraries for whatever reason, ITERATIVE_SCHUR with SCHUR_JACOBI is an excellent alternative.

5. For large bundle adjustment problems (a few thousand cameras or more) use the ITERATIVE_SCHUR solver.
There are a number of preconditioner choices here. SCHUR_JACOBI offers an excellent balance of speed and accuracy.
This is also the recommended option if you are solving medium sized problems for which DENSE_SCHUR is too slow but SuiteSparse is not available.
Note: If you are solving small to medium sized problems, consider setting Solver::Options::use_explicit_schur_complement to true, it can result in a substantial performance boost.
If you are not satisfied with SCHUR_JACOBI‘s performance try CLUSTER_JACOBI and CLUSTER_TRIDIAGONAL in that order. They require that you have SuiteSparse installed.
Both of these preconditioners use a clustering algorithm. Use SINGLE_LINKAGE before CANONICAL_VIEWS.
*/
ceres::Solver::Options getOptions(){
    // Set a few options
    ceres::Solver::Options options;
    //options.use_nonmonotonic_steps = true;
    //options.preconditioner_type = ceres::IDENTITY;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 50;

//    options.preconditioner_type = ceres::SCHUR_JACOBI;
//    options.linear_solver_type = ceres::DENSE_SCHUR;
//    options.use_explicit_schur_complement=true;
//    options.max_num_iterations = 100;

    cout << "Ceres Solver getOptions()" << endl;
    cout << "Ceres preconditioner type: " << options.preconditioner_type << endl;
    cout << "Ceres linear algebra type: " << options.sparse_linear_algebra_library_type << endl;
    cout << "Ceres linear solver type: " << options.linear_solver_type << endl;

    return options;
}

ceres::Solver::Options getOptionsMedium(){
    // Set a few options
    ceres::Solver::Options options;

    #ifdef _WIN32
        options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        options.preconditioner_type = ceres::SCHUR_JACOBI;
    #else
        //options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    #endif // _WIN32

    //If you are solving small to medium sized problems, consider setting Solver::Options::use_explicit_schur_complement to true, it can result in a substantial performance boost.
    options.use_explicit_schur_complement=true;
    options.max_num_iterations = 50;

    cout << "Ceres Solver getOptionsMedium()" << endl;
    cout << "Ceres preconditioner type: " << options.preconditioner_type << endl;
    cout << "Ceres linear algebra type: " << options.sparse_linear_algebra_library_type << endl;
    cout << "Ceres linear solver type: " << options.linear_solver_type << endl;

    return options;
}

void solve(ceres::Problem &problem, bool smallProblem=false){
    ceres::Solver::Summary summary;
    ceres::Solve(smallProblem ? getOptions() : getOptionsMedium(), &problem, &summary);
    if(!smallProblem) std::cout << "Final report:\n" << summary.FullReport();
}

void isoToAngleAxis(const Isometry3d& pose, double* cam){
//    Matrix<const double,3,3> rot(pose.linear());
//    cout<<"rotation : "<<pose.linear().data()<<endl;
//    auto begin = pose.linear().data();
    RotationMatrixToAngleAxis(ColumnMajorAdapter4x3(pose.linear().data()), cam);
    Vector3d t(pose.translation());
    cam[3]=t.x();
    cam[4]=t.y();
    cam[5]=t.z();
}

Isometry3d axisAngleToIso(const double* cam){
    Isometry3d poseFinal = Isometry3d::Identity();
    Matrix3d rot;
    ceres::AngleAxisToRotationMatrix(cam,rot.data());
    poseFinal.linear() = rot;
    poseFinal.translation() = Vector3d(cam[3],cam[4],cam[5]);
    return poseFinal;//.cast<float>();
}

Isometry3d eigenQuaternionToIso(const Eigen::Quaterniond& q, const Vector3d& t){
    Isometry3d poseFinal = Isometry3d::Identity();
    poseFinal.linear() = q.toRotationMatrix();
    poseFinal.translation() = t;
    return poseFinal;//.cast<float>();
}

Sophus::SE3d isoToSophus(const Isometry3d& pose){
    return Sophus::SE3d(pose);
}

Isometry3d sophusToIso(Sophus::SE3d soph){
    //    return Isometry3d(soph.matrix());
    Isometry3d poseFinal = Isometry3d::Identity();
    poseFinal.linear() = soph.rotationMatrix();
    poseFinal.translation() = soph.translation();
    return poseFinal;
}



Isometry3d solve_pts_to_plane(
        array<vector<Vector4d>,3> points,
        vector<Vector4d> planes,
        vector<int> plane_indexes,
        vector<Quaterniond> qwc_list,
        vector<Vector3d> twc_list,
        bool autodiff
    ) {
    Sophus::SE3d soph = isoToSophus(Isometry3d::Identity());
    ceres::Problem problem;
    vector<Vector4d> pt_cam_list;
    Vector4d pt_cam;
    Quaterniond qcw;
    Vector3d tcw;
    int ct, pindex;
    for (int i = 0; i < planes.size(); ++i) {
        pindex = plane_indexes[i];
        qcw = qwc_list[i].inverse();
        tcw = -(qcw * twc_list[i]);
        for (int j = 0; j < points[pindex].size(); ++j) {
            pt_cam.head(3) = qcw * points[pindex][j].head(3) + tcw;
            pt_cam[3] = 1;
            pt_cam_list.emplace_back(pt_cam);
        }
    }
    ct = 0;
    for (int i = 0; i < planes.size(); ++i) {
        pindex = plane_indexes[i];
        for (int j = 0; j < points[pindex].size(); ++j) {
            ceres::CostFunction* cost_function = ICPCostFunctions::Pts2Plane::Create(pt_cam_list[ct], planes[i]);
            problem.AddResidualBlock(cost_function, NULL, soph.data());
            ct += 1;
        }
    }
#ifdef useLocalParam
    ceres::LocalParameterization* param = sophus_se3::getParameterization(autodiff);
    problem.SetParameterization(soph.data(),param);
#endif
    solve(problem);
    return sophusToIso(soph);
}




} //end namespace

