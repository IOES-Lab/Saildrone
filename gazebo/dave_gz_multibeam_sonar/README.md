# Dave Gazebo Multibeam Sonar Plugin

- The sensor itself is at `multibeam_sonar`
- The system plugin that will also be defined in `world` file is at `multibeam_sonar_system`
- We need both the sensor and the system plugin to be able to use the sensor in Gazebo

## 1. Launch example with `dave_sensor.launch.py`

```
ros2 launch dave_demos dave_sensor.launch.py namespace:=blueview_p900 world_name:=dave_multibeam_sonar paused:=false x:=4 z:=2.0 yaw:=3.14
```

To check the sonar image, you can open RViz2 and add a view to the sonar image topic in the displays panel ( Add → By topic → `/sensor/multibeam_sonar/sonar_image` → Image):

## Launch example with `ros_gz_bridge` to translate gazebo point cloud to ROS

```
ros2 launch dave_multibeam_sonar_demo multibeam_sonar_demo.launch.py
```

You should be able to also see the point cloud and sonar image in RViz2.

## **Changing sonar parameters**

In the sonar SDF file (`dave/models/dave_sensor_models/description`), it is possible to change the sonar **properties**, as well as the ROS 2 topics used to publish the images and point cloud.

```xml

  <spec>
    <!-- Sonar properties -->
    <verticalFOV>12</verticalFOV>
    <sonarFreq>900e3</sonarFreq>
    <bandwidth>29.9e3</bandwidth>
    <soundSpeed>1500</soundSpeed>
    <sourceLevel>220</sourceLevel>
    <maxDistance>10</maxDistance>
    <constantReflectivity>false</constantReflectivity>
    <raySkips>10</raySkips>
    <sensorGain>0.02</sensorGain>
    <plotScaler>10</plotScaler>
    <writeLog>true</writeLog>
    <debugFlag>true</debugFlag>
    <writeFrameInterval>5</writeFrameInterval>

    <!-- ROS publication topics -->
    <pointCloudTopicName>point_cloud</pointCloudTopicName>
    <sonarImageRawTopicName>sonar_image_raw</sonarImageRawTopicName>
    <sonarImageTopicName>sonar_image</sonarImageTopicName>
    <frameName>forward_sonar_optical_link</frameName>
  </spec>
```