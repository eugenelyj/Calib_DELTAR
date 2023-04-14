#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using namespace std;
using namespace Eigen;

#define rad2deg2(r) (180 * (r) / M_PI)
//#define deg2rad(d) (M_PI*(d)/180)

#ifdef _WIN32
#include "dirent-win.h"
#else
#include <dirent.h>
#endif

#include <algorithm>   // std::any_of
#include <array>       // std::array
#include <memory>
#include <random>
#include <sophus/so3.hpp>
#include <vector>

static int get_plane_index(array<Vector2d, 3> time_stamp, double t){
    for(int i = 0; i < 3; i++){
        if (t>time_stamp[i][0] && t<time_stamp[i][1])
            return i;
    }
    return -1;
}

static void load_data(const std::string data_root,
                       array<vector<Vector4d>,3>& points,
                       vector<Vector4d>& planes,
                       vector<int>& plane_index,
                       vector<Quaterniond>& rs_q,
                       vector<Vector3d>& rs_twc) {

    const std::string order_path = data_root + "/plane_order.txt";
    const std::string plane_path = data_root + "/valid_tof_planes.txt";
    const std::string point_path = data_root + "/Catogoried_MapPoint.txt";
    const std::string traj_path = data_root + "/valid_traj.txt";

    // 1. build tof timestamp for tof classify
    array<Vector2d, 3> time_stamp;
    int count;
    std::ifstream file1(order_path.c_str(), std::ifstream::in);
    if (file1.fail() == true) {
        cerr << order_path << " could not be opened" << endl;
        exit(0);
    }
    for(int i = 0;i < 3; i++){
        for(int j = 0;j < 3; j++){
            int index;
            double t1, t2;
            file1 >> index >> t1 >> t2;
            time_stamp[index][0] = t1;
            time_stamp[index][1] = t2;
        }
    }

    // 2. build tof plane and plane index
    std::ifstream file2(plane_path.c_str(), std::ifstream::in);
    if (file2.fail() == true) {
        cerr << plane_path << " could not be opened" << endl;
        exit(0);
    }
    file2 >> count;
    Vector4d plane;
    double t;
    for (int j = 0; j < count; j++) {
        file2 >> t;
        file2 >> plane[0] >> plane[1] >> plane[2] >> plane[3];
        int index = get_plane_index(time_stamp, t);
        if (index != -1){ // valid time
            planes.push_back(plane);
            plane_index.push_back(index);
        }
    }

    // 3. build rs points
    std::ifstream file3(point_path.c_str(), std::ifstream::in);
    if (file3.fail() == true) {
        cerr << point_path << " could not be opened" << endl;
        exit(0);
    }
    for (int i = 0; i < 3; i++) {
        file3 >> count;
        Vector4d point;
        for (int j = 0; j < count; j++) {
            file3 >> point.x() >> point.y() >> point.z();
            point[3] = 1;
            points[i].push_back(point);
        }
    }

    // 4. build rs traj
    std::ifstream file4(traj_path.c_str(), std::ifstream::in);
    if (file4.fail() == true) {
        cerr << traj_path << " could not be opened" << endl;
        exit(0);
    }
    Quaterniond q;
    Vector3d twc;
    file4 >> count;
    for (int j = 0; j < count; j++) {
        file4 >> twc[0] >> twc[1] >> twc[2];
        file4 >> q.x() >> q.y() >> q.z() >> q.w();
        rs_q.push_back(q);
        rs_twc.push_back(twc);
    }
}


#endif   // COMMON_H
