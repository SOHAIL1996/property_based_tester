# Build Ouster Gazebo Plugin

- mkdir build && cd build
- cmake ..
- make 

# Add to URDF

```bash
<!-- Ouster Augment -->
<link name="os1_sensor">
    <inertial>
        <mass value="0.33"/>
        <origin xyz="0 0 0.0365" rpy="0 0 0" />
        <inertia ixx="0.000241148" ixy="0" ixz="0"
        iyy="0.000241148" iyz="0" izz="0.000264"/>
    </inertial>
    <collision name="base_collision">
        <origin xyz="0 0 0.0365" rpy="0 0 0" />
        <geometry>
        <cylinder radius="0.04" length="0.073"/>
        </geometry>
    </collision>
    <visual name="base_visual">
        <origin xyz="0 0 0.0" rpy="0 0 1.5707" />
        <geometry>
            <mesh filename="package://ouster_description/meshes/os1_64.dae" /> 
        <!-- <cylinder length="0.073" radius="0.04" /> -->
        </geometry>
    </visual>
</link>

<joint name="os1_sensor_lidar_link_joint" type="fixed">
    <parent link="base_link" />
    <child link="os1_sensor" />
    <origin xyz="0.18 0.0 0.218" rpy="0 0 0" /> <!-- 0.03618 is the default height -->
</joint>

<!-- Ouster gazebo library -->
<gazebo reference="os1_sensor">
    <sensor type="ray" name="os1_sensor-OS1-64">
        <pose>0 0 0 0 0 0</pose>
        <visualize>false</visualize>
        <update_rate>10</update_rate>
        <ray>
        <scan>
            <horizontal>
            <samples>220</samples> <!-- 512 default-->
            <resolution>1</resolution>
            <min_angle>-3.1415926535897931</min_angle>
            <max_angle>3.1415926535897931</max_angle>
            </horizontal>
            <vertical>
            <samples>64</samples>
            <resolution>1</resolution>
            <min_angle>-0.26</min_angle>
            <max_angle>0.26</max_angle>
            </vertical>
        </scan>
        <range>
            <min>0.90</min>
            <max>75.0</max>
            <resolution>0.03</resolution>
        </range>
        </ray>
        <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_ouster_laser.so">
        <topicName>/os1_cloud_node/points</topicName>
        <frameName>os1_sensor</frameName>
        <min_range>0.90</min_range>
        <max_range>75.0</max_range>
        <gaussianNoise>0.008</gaussianNoise>
        </plugin>
    </sensor>
</gazebo>
```