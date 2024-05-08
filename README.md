<h1 align="center">
   <img src="./images/Picto+STEREOLABS_Black.jpg" alt="Stereolabs" title="Stereolabs" /><br \>
   ZED X One - ROS 2
</h1>

<p align="center">
  ROS 2 packages for using Stereolabs ZED X One Camera cameras.<br>
  ROS 2 Foxy Fitzroy (Ubuntu 20.04) - ROS 2 Humble Hawksbill (Ubuntu 22.04)
</p>

<hr>

This package lets you use the ZED X One monocular cameras with ROS 2. It provides access to the following data:

  - Color stream
  - Camera settings
  - Camera dynamic controls

## Installation

### Prerequisites

- [Ubuntu 20.04 (Focal Fossa)](https://releases.ubuntu.com/focal/) or [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)
- ROS 2 Foxy Fitxroy or ROS 2 Humble Hawksbill: 
  - [Foxy on Ubuntu 20.04](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html)
  - [Humble on Ubuntu 22.04](https://docs.ros.org/en/humble/Installation/Linux-Install-Debians.html)

### Build the package

The repository requires [colcon](http://design.ros2.org/articles/build_tool.html) to build the pakcages.

**Note:** If you haven’t set up your colcon workspace yet, please follow this short [tutorial](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/). 

To build and install all the packages, open a bash terminal, clone the repository from Github, and build it:

```bash
mkdir -p ~/ros2_ws/src/ # create your workspace if it does not exist
cd ~/ros2_ws/src/ #use your current ros2 workspace folder
git clone https://github.com/stereolabs/zedxone-ros2.git
cd ..
sudo apt update
rosdep install --from-paths src --ignore-src -r -y # install dependencies
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc # automatically source the installation in every new bash (optional)
source ~/.bashrc
```

**Note:** If `rosdep` is missing you can install it with:

  ```sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool python3-rosinstall build-essential```

**Note:** The option `--symlink-install` is very important, it allows to use symlinks instead of copying files to the ROS 2 folders during the installation, where possible. Each package in ROS 2 must be installed and all the files used by the nodes must be copied into the installation folders. Using symlinks allows you to modify them in your workspace, reflecting the modification during the next executions without needing to issue a new `colcon build` command. This is true only for all the files that don't need to be compiled (Python scripts, configurations, etc.).

**Note:** If you are using a different console interface like zsh, you have to change the `source` command as follows: `echo source $(pwd)/install/local_setup.zsh >> ~/.zshrc` and `source ~/.zshrc`.

## Starting a ZED X One node

The package `zedxone_node` provides launch files and configuration files.

To start the node:

```bash
ros2 launch zedxone_node zedxone.launch.py camera_model:='<model>'
```

it is required to replace `'<model>'` with the model of ZED X One camera that you are using, i.e. `'GS'` or `'4K'`.

The launch command accepts other customization arguments. You can use the `-s` option to retrieve them all:

```bash
    'camera_name':
        The name of the camera. It can be different from the camera model and it will be used as node `namespace`.
        (default: 'zedxone')

    'camera_model':
        [REQUIRED] The model of the camera. Using a wrong camera model can disable camera features. Valid choices are: ['GS', '4K']

    'node_name':
        The name of the zed_wrapper node. All the topic will have the same prefix: `/<camera_name>/<node_name>/`
        (default: 'zedxone_node')

    'config_path':
        Path to the YAML configuration file for the camera.
        (default: '~/ros2_ws/src/zedxone-ros2/zedxone_node/share/zedxone_node/config/zedxone.yaml')

    'idx':
        The index number of the camera to be opened.
```

### ROS 2 Composition

The launch file `zedxone.launch.py` uses composition to start a `stereolabs::ZedXOneCamera` component in a ROS 2 container.

For more information concerning how to leverage ROS 2 Composition, plese refer to the [official documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html).

## Node Parameters

The package `zedxone_node` stores the YAML files containing the default parameters of the node.
The file `zedxone_foxy.yaml` is automatically used if the launch file detects the usage of the Foxy distribution.

```yaml
camera:
    model: 'GS' # Global Shutter: 'GS', 4K HDR: '4K'
    idx: 0
    resolution: 'HD1200' # 'SVGA', 'HD1080', 'HD1200', '4K'
    framerate: 15
    swap_rb: false
    pixel_format: 'COLOR_RGBA' # 'COLOR_RGBA', 'COLOR_RGB', 'RAW_BAYER'
    timeout_msec: 2000 # Camera timeout in milliseconds

    dynamic:
        auto_exposure: true # Enable Automatic Exposure
        exposure_range_min: 28 # Minimum value for Automatic Exposure
        exposure_range_max: 66000 # Maximum value for Automatic Exposure
        manual_exposure_usec: 2000 # Manual Exposure time
        auto_analog_gain: true # Enable Automatic Analog Gain
        analog_frame_gain_range_min: 0.1 # Minimum value for Automatic Analog Gain
        analog_frame_gain_range_max: 30.0 # Maximum value for Automatic Analog Gain
        manual_analog_gain_db: 1.0 # Manual Analog Gain
        auto_digital_gain: true # Enable Automatic Digital Gain
        digital_frame_gain_range_min: 1 # Minimum value for Automatic Digital Gain
        digital_frame_gain_range_max: 256 # Maximum value for Automatic Digital Gain
        manual_digital_gain_value: 128 # Manual Digital Gain [1,256]
        auto_wb: true # Enable Automatic White Balance
        manual_wb: 5000 # Manual White Balance [2800,12000]
        ae_anti_banding: 'AUTO' # Exposure anti banding - 'OFF', 'AUTO', '50Hz', '60Hz'
        color_saturation: 1.0 # Color Saturation [0.0,2.0]
        denoising: 0.5 # Image Denoising [0.0,1.0]
        exposure_compensation: 0.0 # Exposure Compensation [-2.0,2.0]
        sharpening: 1.0 # Image Sharpening [0.0,1.0]
        aec_agc_roi_x: -1 # AEC-AGC ROI top left x coordinate
        aec_agc_roi_y: -1 # AEC-AGC ROI top left y coordinate
        aec_agc_roi_w: -1 # AEC-AGC ROI width
        aec_agc_roi_h: -1 # AEC-AGC ROI height
        tone_mapping_r_gamma: 2.0 # [1.5,3.5]
        tone_mapping_g_gamma: 2.0 # [1.5,3.5]
        tone_mapping_b_gamma: 2.0 # [1.5,3.5]

debug:
    grab_verbose_level: 0 # Set the verbose level of the capture library [0-6]
    general: false # Enable the general debug log of the node
    diagnostic: false # Enable the diagnostic debug log of the node
    controls: false # Enable the control debug log of the node
```

## Stereo configuration

The packages of the [ZED ROS 2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper/) repository allow you to use a virtual Stereo Camera composed of two ZED X One devices with a custom baseline. 

Read [the ZED X One documentation](https://www.stereolabs.com/docs/get-started-with-zed-x-one/zed-x-one-stereo) for more information concerning the setup of a Virtual Stereo ZED X One camera.

## Known issues

* IMU sensor data not yet available
