# Pandar Rain Filter Visualizer

Command-line Interface for Pandar rain filter visualizing tool. The tool expects three directory path containing the particle labels, pointcloud images, and range images stored in the following structure and file naming:

```
particle_labels
├─first
│ ├─0.png
│ ├─1.png
│ ├─ ...
└─last
  ├─0.png
  ├─1.png
  ├─ ...

point_cloud_images
├─point_cloud_first_img_0.exr
├─point_cloud_first_img_1.exr
├─ ...
├─point_cloud_last_img_0.exr
├─point_cloud_last_img_1.exr
├─ ...

range_images
├─first_depth
│ ├─0.png
│ ├─1.png
│ ├─ ...
├─first_intensity
│ ├─0.png
│ ├─1.png
│ ├─ ...
├─first_return_type
│ ├─0.png
│ ├─1.png
│ ├─ ...
├─last_depth
│ ├─0.png
│ ├─1.png
│ ├─ ...
├─last_intensity
│ ├─0.png
│ ├─1.png
│ ├─ ...
└─last_return_type
  ├─0.png
  ├─1.png
  ├─ ...

```
## Usage

```sh
rosrun pandar_rain_filter_visualizer pandar_rain_filter_visualizer _particle_labels_path:=[particle labels path]  _point_cloud_images_path:=[point cloud images path]  _range_images_path:=[range images path]
```
Open rviz in a new terminal with the configuration file
```sh
rviz rviz -d visualizer.rviz
```

Once the program is running, the following screen appears
![Alt text](images/usage1.png?raw=true "Title")

Enter the frame number you want to skip to, then the following screen appears
![Alt text](images/usage2.png?raw=true "Title")

You can choose either to publish point clouds or step through them manually.


![Alt text](images/usage3.png?raw=true "Title")

Use Right and Left arrow keys for stepping through frames

Press CTRL+C & ENTER to stop the program.

<u>**NOTE:**</u> The terminal window should be in focus for keyboard navigation to work.

# Parameters and options

 |Param|Default Value|Description|
 |---|---|---|
 |`particle_labels_path`|None|Path of folder containing rain / noise labels|
 |`point_cloud_images_path`|None|Path of folder containing point cloud images|
 |`range_images_path`|None|Path of folder containing range images|


# Dependencies

Install `ncurses` for keyboard inputs: `sudo apt-get install libncurses5-dev`
