import math
from cv2 import threshold
import numpy as np
import open3d as o3d
import sys
from pathlib import Path
import copy


FOV = 45
M_SINGLE_PLANE_THRES = 0.005
pcd_colors = [[1, 0, 0], [1, 1, 0], [0, 1, 1]]

def get_grid_on_plane(plane, grid=None):
    def get_intersection(direction):
        # let x = at, y = bt, z = ct
        # Aat + Bbt + Cct + D = 0; t = -D/(Aa + Bb + Cc)
        a, b, c = direction
        t = -D/(A*a + B*b + C*c)
        x, y, z = a*t, b*t, c*t
        return np.array([x,y,z])

    # plane:(A, B, C, D), return grid with shape [9, 9, 3]
    A, B, C, D = plane
    if grid is None:
        grid, _ = get_grid_and_center()
    else:
        grid = copy.deepcopy(grid)
    for i in range(9):
        for j in range(9):
            grid[i][j] = get_intersection(grid[i][j])
    return grid

def get_grid_and_center():
    h = 1 / math.tan(math.radians(FOV)/2)
    grid = np.zeros((9, 9, 3))
    center = np.zeros((8, 8, 3))
    for i in range(9):
        for j in range(9):
            grid[i][j] = np.array([(8-j)/8*-1 + (j/8)*1, (8-i)/8*-1 + (i/8)*1, h])
    for i in range(8):
        for j in range(8):
            x = (grid[i][j][0] + grid[i][j+1][0]) / 2
            y = (grid[i][j][1] + grid[i+1][j][1]) / 2
            center[i][j] = np.array([x, y, h])
    return grid, center

def array2string(a):
    # format control
    np.set_printoptions(threshold=sys.maxsize)
    np.set_printoptions(suppress=True)
    return np.array2string(a).replace("[", "").replace("]", "").replace("\n ", "\n").replace("\n ", "\n").replace("\n ", "\n").replace("\n ", "\n").strip() + '\n'


class ToFRaw():
    def __init__(self, path):
        # read ToF data
        # import ipdb; ipdb.set_trace()
        with open(path, mode='r') as f:
            lines = f.readlines()
        self.res = 8 if lines[0].split()[1] == '3' else 4
        self.data = dict()
        tmp_depth, tmp_status = [], []
        for line in lines:
            split_line = line.split()
            if len(split_line) < 2: continue
            if split_line[0] == 'N': # start a new frame
                if tmp_depth != [] : # save previous frame from tmp buffer
                    if len(tmp_depth) == self.res * self.res: # only append complete frame
                        tt = np.array(tmp_depth).reshape(self.res, self.res)
                        self.data[timestamp] = [tt, 
                                                np.array(tmp_status).reshape(self.res, self.res)]
                timestamp = float(split_line[2][:10+1+6])
                tmp_depth, tmp_status = [], [] # reset buffer
            elif len(split_line) == 10: # a complete zone record line
                tmp_depth.append(int(split_line[8]))
                tmp_status.append(int(split_line[5][:-1])) # delete comma
            else: # incomplete record, ignore
                break
    
    def __len__(self):
        return len(list(self.data.keys()))

class ToFSequence():
    def __init__(self, tof_data:ToFRaw) -> None:
        self.resolution = tof_data.res
        self.fov = FOV
        self.data = tof_data.data
        self.timestamps = list(self.data.keys())
        self.num_frame = len(self.timestamps)
        self.transformations = dict()
        self.grid, self.centers = get_grid_and_center()
        self.EM_record={"initial_loss":[], "final_loss":[], "plane_update_count":0}

    def generate_pcd(self, timestamp, use_tof_as_z=True):
        point_list = []
        assert self.resolution == self.centers.shape[0]
        for i in range(self.resolution):
            for j in range(self.resolution):
                direction = copy.deepcopy(self.centers[i][j])
                direction /= np.linalg.norm(direction)
                if use_tof_as_z:
                    distance = self.data[timestamp][0][i][j] * (1/direction[2])
                else:
                    distance = self.data[timestamp][0][i][j]
                point = direction * distance / 1e3
                point_list.append(point)
        xyz = np.array(point_list)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd

    def check_zone(self, grid, row, col, distance):
        c1, c2, c3, c4 = grid[row][col], grid[row][col+1], grid[row+1][col], grid[row+1][col+1]
        corners = [c1, c2, c3, c4]
        dmax, dmin = 0, math.inf
        threshold = 0.0
        
        for i, corner in enumerate(corners):
            d_cur = np.linalg.norm(corner)
            if d_cur > dmax:
                dmax = d_cur
                corner_max = corner
            if d_cur < dmin:
                dmin = d_cur
                corner_min = corner
        if distance + threshold > dmin and distance - threshold < dmax:
            return None
        elif distance > dmax:
            return corner_max / np.linalg.norm(corner_max) * distance
        else:
            return corner_min / np.linalg.norm(corner_min) * distance
    
    def solve_sphere_plane_intersection(self, c1, c4, radius, plane):
        A, B, C, D = plane
        xmin, xmax = c1[0], c4[0]
        ymin, ymax = c1[1], c4[1]
        
        def solve_intersrction(m, mode):
            # ay^2 + by + c = 0
            threshold = 0
            if mode == "givenx":
                a = B**2  / C**2 + 1
                b = 2*A*B*m / C**2 + 2*B*D / C**2
                c = (A**2/C**2 + 1) * m**2 + 2*A*D / C**2 * m + D**2/C**2 - radius**2
            elif mode == "giveny":
                a = A**2  / C**2 + 1
                b = 2*A*B*m / C**2 + 2*A*D / C**2
                c = (B**2/C**2 + 1) * m**2 + 2*B*D / C**2 * m + D**2/C**2 - radius**2
            delta = b**2-4*a*c
            result = []
            if delta <= 0: return result
            else :
                n1 = (-1*b + math.sqrt(delta) ) / (2*a)
                n2 = (-1*b - math.sqrt(delta) ) / (2*a)
                if mode == "givenx":
                    # then use y range to valify
                    backup_result.extend(((m, n1, math.sqrt(radius**2 - m**2 - n1**2)), (m, n2, math.sqrt(radius**2 - m**2 - n2**2))))
                    if n1 < ymax + threshold and n1 + threshold > ymin: result.append((m, n1, math.sqrt(radius**2 - m**2 - n1**2)))
                    if n2 < ymax + threshold and n2 + threshold > ymin: result.append((m, n2, math.sqrt(radius**2 - m**2 - n2**2)))
                elif mode == "giveny":
                    # then use x range to valify
                    backup_result.extend(((n1, m, math.sqrt(radius**2 - m**2 - n1**2)), (n2, m, math.sqrt(radius**2 - m**2 - n2**2))))
                    if n1 < xmax + threshold  and n1 + threshold > xmin: result.append((n1, m, math.sqrt(radius**2 - m**2 - n1**2)))
                    if n2 < xmax + threshold  and n2 + threshold> xmin: result.append((n2, m, math.sqrt(radius**2 - m**2 - n2**2)))
            return result

        backup_result = []

        result = solve_intersrction(xmin, "givenx")
        backup_result.extend(result)
        if result: return result[0]
        result = solve_intersrction(xmax, "givenx")
        backup_result.extend(result)
        if result: return result[0]
        result = solve_intersrction(ymin, "giveny")
        backup_result.extend(result)
        if result: return result[0]
        result = solve_intersrction(ymax, "giveny")
        backup_result.extend(result)
        if result: return result[0]

    def calculate_loss(self, plane, points):
        loss = 0
        for point in points:
            point_H = np.append(point, 1)
            loss += np.abs(np.dot(plane, point_H))
        return loss

    def fit_single_plane(self, timestamp, viz=False):
        pcd = self.generate_pcd(timestamp, use_tof_as_z=True)

        plane_model, inliers = pcd.segment_plane(distance_threshold=M_SINGLE_PLANE_THRES,
                                         ransac_n=3,
                                         num_iterations=1000)
        inlier_cloud = pcd.select_by_index(inliers)

        # grid generation test
        grid = get_grid_on_plane(plane_model, self.grid)
        plane_flag=False
        if viz:
            grid_pcd = o3d.geometry.PointCloud()
            grid_pcd.points = o3d.utility.Vector3dVector(grid.reshape(-1, 3))
            grid_pcd.paint_uniform_color([0, 1.0, 0])
            o3d.visualization.draw_geometries([pcd, grid_pcd])
        initial_loss = self.calculate_loss(plane_model, np.array(inlier_cloud.points))
        initial_plane = plane_model
        loss = initial_loss
        update_count = 1
        while loss > 5e-2 and update_count>0:
            points = np.asarray(pcd.points).reshape(8, 8, -1)
            update_count = 0
            inplane_count = 0
            for i in range(self.resolution):
                for j in range(self.resolution):
                    # depth = self.data[timestamp][0][i][j]/1e3
                    if 8*i+j not in inliers:continue
                    point = points[i][j]
                    distance = np.linalg.norm(point)
                    corner = self.check_zone(grid, i, j, distance)       
                    if corner is None: # can find a point on the plane that make 0 loss
                        new_point = self.solve_sphere_plane_intersection(c1=grid[i][j], c4=grid[i+1][j+1], 
                        radius=distance, plane=plane_model)
                        inplane_count += 1
                    else:
                        new_point = corner
                        plane_flag=True
                    if new_point is not None and np.linalg.norm(point-new_point)>1e-4:
                        update_count+=1
                        points[i][j] = new_point
            pcd.points = o3d.utility.Vector3dVector(points.reshape(-1, 3))
            inlier_cloud = pcd.select_by_index(inliers)
            plane_model, inliers = inlier_cloud.segment_plane(distance_threshold=M_SINGLE_PLANE_THRES,
                                         ransac_n=3,
                                         num_iterations=1000)
            loss = self.calculate_loss(plane_model, np.array(inlier_cloud.points))
        if plane_flag:
            print(f"Loss:{initial_loss}->{loss}")
            print(f"Plane:{initial_plane}->{plane_model}")
        self.EM_record['initial_loss'].append(initial_loss)
        self.EM_record['final_loss'].append(loss)
        if plane_flag>0: self.EM_record['plane_update_count']+=1
        return plane_model, inlier_cloud
    
    def save_tof_planes(self, save_path):
        with open(save_path, "w") as f:
            f.write(str(len(self.timestamps))+"\n")
            for ts in self.timestamps:
                plane_model, inlier_cloud = self.fit_single_plane(ts, viz=False)
                f.write(str(ts)+" ")
                f.write(array2string(plane_model))
        print(f"Totally generate {len(self.timestamps)} planes.")
        print(f"Point to plane loss: {np.array(self.EM_record['initial_loss']).mean()} -> {np.array(self.EM_record['final_loss']).mean()}")
        print(f"Plane update ratio {self.EM_record['plane_update_count']/len(self.timestamps)*100}%")
    
    def catogory_slam_mappoint(self, mappoint_path, save_path, draw_plane=False):

        mappoint = np.loadtxt(mappoint_path, skiprows=1)

        # 2.1 extract 3 planes
        mappoints_catogory = []
        colors = [[0,1,1], [1,0,0], [1,1,0]]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(mappoint)
        for i in range(3):
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.02,
                                         ransac_n=3,
                                         num_iterations=1000)
            inlier_cloud = pcd.select_by_index(inliers)
            inlier_cloud.paint_uniform_color(colors[i])
            mappoints_catogory.append({'plane':plane_model, 'inlier_cloud':inlier_cloud})
            pcd = pcd.select_by_index(inliers, invert=True)

        # 2.2  classify mappoints into three planes
        mappoint_clouds = [item['inlier_cloud'] for item in mappoints_catogory]
        planes = [item['plane'] for item in mappoints_catogory]
        self.mappoint = mappoint_clouds

        # 2.3 save mappoints
        with open(save_path, "w") as f:
            # write mappoints in three planes sequentially
            for i in range(3): 
                points = np.array(mappoint_clouds[i].points)
                plane = planes[i]
                # f.write(array2string(plane))
                f.write(str(len(points)) + '\n')
                f.write(array2string(points))
            f.close()

        if draw_plane:
            vis_list = []
            pcd_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            for pcd_index, point_cloud in enumerate(mappoint_clouds):
                point_cloud.paint_uniform_color(pcd_colors[pcd_index])
                vis_list.append(point_cloud)
            o3d.visualization.draw_geometries(vis_list)



if __name__ == '__main__':
    import argparse
    import os

    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=str)
    args = parser.parse_args()

    out_dir = args.root
    tof_file = os.listdir(f'{args.root}/tof')
    assert len(tof_file) == 1
    tof_path = f'{args.root}/tof/{tof_file[0]}'
    mappoint_path = f'{args.root}/MapPoint.txt'

    output_path = f"{out_dir}/Catogoried_MapPoint.txt"

    tof_data = ToFRaw(tof_path)
    tof_sequence = ToFSequence(tof_data)
    tof_sequence.save_tof_planes(f"{out_dir}/tof_planes.txt")
    tof_sequence.catogory_slam_mappoint(mappoint_path, output_path)
