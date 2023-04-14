# This repo contains both the recording programs (L5 and RealSense) and the calibration program.

**For how to setup an device suite like in our paper, please refer to the [doc](VL53L5CX_Linux_Getting_started_with_Rpi.pdf).**

## Record the L5 data/RGB-images/depth-maps
If you do not set up your own device, you can skip this step.

### Compile

**Install record_tof on your devices, e.g., an Raspberry Pi**. 
Copy the record_tof folder to your devices and install according to the README.

**Install record_realsense on your laptop**:
```
cd record_realsense
mkdir build
cmake ..
make -j2
```

### Record

On your Raspberry Pi, enter the record_tof folder, run `bin/long_record` (record data in an infinite loop) or `bin/short_record` (record within a given duration) to record **L5 data**.

On your laptop, if you compile successfully, there will be also `bin/long_record` and `bin/short_record` under the record_realsense folder, which are used to record **RGB-images/depth-maps** from RealSense.


## Calibration
If you don't record your own data for the calibration, we provide the [example data](https://drive.google.com/file/d/1hx7rsJ9X9TzlamZYH4vmeYJ6F0yUCGPv/view).

**Step 1.** Run ORB-SLAM2 on the recorded **RGB-images/depth-maps** to generate **camera trajectory** and the corresponding **mappoints**. We have modified the original ORB-SLAM2 to dump these data.

```
git submodule update --init --recursive
cd ORB_SLAM2
mkdir build
cmake ..
make -j4
cd ..
zsh run_realsense.zsh
```
**Step 2.** Run `optimize_plane.py` to generate planes observed in ToF camera and RGB camera.

```
python optimize_plane.py /path/to/data_root
```

**Step 3.** Watch the recorded data, and manually divide it into multiple fragments where the camera only observed one plane in each fragment. If you use the provided example data, it has contained the result of division which is named `plane_order.txt`. Notice that the order of the fragments should match that in the `Catogoried_MapPoint.txt`.

**Step 4.** Run `generate_valid_traj.py`. The script choose frames where the camera only observed one plane and return the pairs of realsense frames and tof frames whose timestamps are close enough.

```
python generate_valid_traj.py /path/to/data_root
```

**Step 5.** Run `pt2plane`, which optimize the calibration matrix.

```
./icp/build/pt2plane /path/to/data_root
```


## Citation

If you find this code useful, please use the following BibTeX entry.

```bibtex
@article{deltar2022,
  title={DELTAR: Depth Estimation from a Light-weight ToF Sensor and RGB Image},
  author={Li Yijin and Liu Xinyang and Dong Wenqi and Zhou han and Bao Hujun and Zhang Guofeng and Zhang Yinda and Cui Zhaopeng},
  booktitle={European Conference on Computer Vision (ECCV)},
  year={2022}
}
```
