import numpy as np

def array2string(a):
    # format control
    # np.set_printoptions(threshold=sys.maxsize)
    np.set_printoptions(suppress=True)
    return np.array2string(a).replace("[", "").replace("]", "").replace("\n ", "\n").replace("\n ", "\n").replace("\n ", "\n").replace("\n ", "\n").strip() + '\n'

def valid_tof(t):
    for i in range(3):
        t1, t2 = time_stamps[i]
        if t > t1 and t < t2:
            return True
    return False



# input: tof_planes.txt, CameraTrajectory.txt, plane_order.txt
# output: valid_traj.txt, valid_tof_planes.txt; these two are of same length, corresponding to each other
# purpose: 1. choose frames that only one main plane is visible, 
#          this is done by using the mannually recorded plane_order.txt stamps.
#          2. return rs and tof pairs whose timestamps are close enough as valid



if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=str)
    args = parser.parse_args()

    root = args.root
    traj_path = f"{args.root}/CameraTrajectory.txt"
    tof_plane_path = f"{args.root}/tof_planes.txt"
    order_path = f"{args.root}/plane_order.txt"
    out_traj_path = f"{args.root}/valid_traj.txt"
    out_tof_planes_path = f"{args.root}/valid_tof_planes.txt"

    rs_traj = np.loadtxt(traj_path)
    plane_order = np.loadtxt(order_path)
    time_stamps = plane_order[:, 1:]

    rs_times = list(rs_traj[:,0])
    rs_T_dict = {rs_times[i]:rs_traj[i, 1:] for i in range(len(rs_times))}#rs_traj[:,1:]

    tof_planes = np.loadtxt(tof_plane_path, skiprows=1)
    tof_times = list(tof_planes[:,0])
    tof_planes_dict = {tof_times[i]:tof_planes[i, 1:] for i in range(len(tof_planes))}
    tof_times = [i for i in tof_times if valid_tof(i)]

    potential_matches = [(abs(a - b), a, b)
                    for a in tof_times 
                    for b in rs_times 
                    if abs(a - b) <= 0.08]
    potential_matches.sort()
    valid_rs_timestamps = []
    valid_tof_timestamps = []
    for diff, a, b in potential_matches:
        if a in tof_times and b in rs_times:
            tof_times.remove(a)
            rs_times.remove(b)
            valid_rs_timestamps.append(b)
            valid_tof_timestamps.append(a)
    valid_rs_timestamps.sort()
    valid_tof_timestamps.sort()

    with open(out_traj_path, "w") as f:
        f.write(str(len(valid_rs_timestamps))+"\n")
        for time_stamp in valid_rs_timestamps:
            for item in rs_T_dict[time_stamp]:
                # f.write(str(time_stamp)+ " ")
                f.write(item.astype('str') + " ")
            f.write("\n")

    with open(out_tof_planes_path, "w") as f:
        f.write(str(len(valid_tof_timestamps))+"\n")
        for time_stamp in valid_tof_timestamps:
            f.write(str(time_stamp)+ " ")
            for item in tof_planes_dict[time_stamp]:
                f.write(item.astype('str') + " ")
            f.write("\n")
