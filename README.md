# gazebo_ros_retina_camera
A Gazebo ROS Plugin for the Retina Framework.

# Gazebo ROS Retina Plugin

This package provides a Gazebo ROS Camera interface to the Retina framework implemented at the University of Granada
 by Pablo Martínez-Cañada et al. [[1]](#references)[[2]](#references) has been integrated in the NRP [[3]](#references).


## Install

The path to the Retina framework directory must be available in the RETINA_INSTALL_DIR environment variable.

Clone this package into your Gazebo ROS Packages workspace and rebuild.

## Usage

This plugin can be used as a drop-in replacement for normal Gazebo camera plugins.
Both, the Gazebo ROS retina plugin and the [CameraPlugin](https://bitbucket.org/osrf/gazebo/src/666bf30ad9a3c042955b55f79cf1a5416a70d83d/plugins/CameraPlugin.cc)
use the Gazebo [CameraSensor](https://bitbucket.org/osrf/gazebo/src/666bf30ad9a3c042955b55f79cf1a5416a70d83d/gazebo/sensors/CameraSensor.cc) internally.

The following SDF snippet shows an example usage:

     <sensor name='camera' type='camera'>
       <camera name='retina'>
         <horizontal_fov>1.047</horizontal_fov>
         <image>
           <width>320</width>
           <height>240</height>
         </image>
         <clip>
           <near>0.1</near>
           <far>100</far>
         </clip>
       </camera>
       <plugin name="RetinaCameraPlugin" filename="libgazebo_ros_retina_camera.so">
	     <retinaScriptPath>my_retina_script.py</retinaScriptPath>
         <cameraName>image_raw</cameraName>
         <alwaysOn>true</alwaysOn>
         <updateRate>10</updateRate>
         <robotNamespace>/</robotNamespace>
       </plugin>
     </sensor>
     
The parameters `robotNamespace`, `cameraName` (default: "events") result in `"$robotNamespace/$cameraName/"`
as the identifier of the topic on which the untouched camera images are published.

For each module $ModuleName defined in the retina configuration script, specified with the `retinaScriptPath` element,
the output is published on two topics:
- .../retina/$ModuleName of type Image: As an R8G8B8 image in false colors.
- .../retina/$ModuleName/data of type Float64MultiArray: as a multi array of doubles (raw data)

The only required element for the plugin to work is `retinaScriptPath`.

## Please Note

When used in the NeuroRobotics Platform, the `retinaScriptPath` element is overridden by the configuration tag in the BIBI;
it is recommended to comment the element out. e.g. `<!--retinaScriptPath>my_retina_script.py</retinaScriptPath-->`

`width` and `height` values must match the ones in the retina config script.

## References

[1] Martínez-Cañada, P., Morillas, C., Pino, B., Ros, E., Pelayo, F. A Computational Framework for Realistic Retina Modeling. In International Journal of Neural Systems. Accepted for publication.</br>
[2] [COREM](https://github.com/pablomc88/COREM)</br>
[3] [retina](https://bbpcode.epfl.ch/code/#/admin/projects/neurorobotics/retina)