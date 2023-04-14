/** @author Adrian Haarbach
 *
 * Comparison of pairwise ICP.
 * variants:
 * - point to point
 * - point to plane
 * minimizers:
 * - closed form solution/1st order approximation
 * - g2o with SO3 vertices and GICP edges
 * - ceres with angle axis
 * - ceres with eigen quaternion
 */

#include "CPUTimer.h"
#include "common.h"
#include "gflags/gflags.h"
#include "icp-ceres.h"

using namespace std;

int main(int argc, char *argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    array<vector<Vector4d>, 3> points;   // rs map_points in 3 planes
    vector<Vector4d> planes;             // tof planes
    vector<int> plane_indexes;           // tof planes' index in [0, 1, 2]
    vector<Quaterniond> qwc_list;
    vector<Vector3d> twc_list;

    load_data(argv[1], points, planes, plane_indexes, qwc_list, twc_list);
    cout << qwc_list.size() << " " << planes.size() << " "
         << plane_indexes.size() << endl;
    cout << points[0].size() << " " << points[1].size() << " "
         << points[2].size() << endl;

    Isometry3d P_init =
        Translation3d(Vector3d(0, 0, 0)) * Quaterniond(Vector4d(0, 0, 0, 1));
    Isometry3d PtestCeres_Sophus;

    CPUTimer timer;

    PtestCeres_Sophus = ICP_Ceres::solve_pts_to_plane(
        points, planes, plane_indexes, qwc_list, twc_list, false);
    timer.toc("ceres SophusSE3");
    timer.printAllTimings();

    auto pt2pl_rot = PtestCeres_Sophus.rotation();
    auto pt2pl_t = PtestCeres_Sophus.translation();
    auto pl2pt_rot = pt2pl_rot.transpose();
    auto pl2pt_t = -pl2pt_rot * pt2pl_t;
    std::cout << "####\n";
    std::cout << pl2pt_rot << std::endl;
    std::cout << pl2pt_t.transpose() << std::endl;

    char tp[100] = "################";
    float metric[2];
    float dist;
    Vector4d pl, pt;
    // float pts_number = (float)(pts[0].size() + pts[1].size() +
    // pts[2].size());
    auto calib_rot = PtestCeres_Sophus.rotation();
    auto calib_t = PtestCeres_Sophus.translation();

    // compute point-to-plane mean distance
    metric[0] = 0;
    metric[1] = 0;
    Vector3d pt_cam;
    Quaterniond qcw;
    Vector3d tcw;
    int total_num = 0;
    for (int i = 0; i < planes.size(); ++i) {
        qcw = qwc_list[i].inverse();
        tcw = -(qcw * twc_list[i]);
        int pindex = plane_indexes[i];
        for (int j = 0; j < points[pindex].size(); ++j) {
            total_num += 1;
            pt_cam = qcw * points[pindex][j].head(3) + tcw;
            dist = fabs(planes[i][0] * pt_cam[0] + planes[i][1] * pt_cam[1] +
                        planes[i][2] * pt_cam[2] + planes[i][3]);
            metric[0] += dist;
            pt_cam = calib_rot * pt_cam + calib_t;
            dist = fabs(planes[i][0] * pt_cam[0] + planes[i][1] * pt_cam[1] +
                        planes[i][2] * pt_cam[2] + planes[i][3]);
            metric[1] += dist;
        }
    }
    metric[0] = metric[0] / total_num;
    metric[1] = metric[1] / total_num;
    printf("%s mean of point-to-plane %s\n", tp, tp);
    printf("before calibration: %lf\n", metric[0]);
    printf("after calibration: %lf\n", metric[1]);

    return 0;
}
