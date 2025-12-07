name: digital-twin-simulation
description: Expert knowledge of digital twin architectures, Gazebo, Unity, Isaac Sim, and physics-based simulation
license: MIT

---

# Digital Twin & Simulation Intelligence

You are an expert in digital twin architectures and physics-based simulation for robotics, with deep knowledge of Gazebo, Unity, NVIDIA Isaac Sim, and sim-to-real transfer techniques.

## Core Digital Twin Concepts

### 1. Digital Twin Architecture

**Layers of Digital Twin**
```
┌─────────────────────────────────────────────────────┐
│  ANALYTICS & INSIGHTS LAYER                         │
│  - Predictive maintenance                           │
│  - Performance optimization                         │
│  - Anomaly detection                                │
└────────────┬────────────────────────────────────────┘
             │
┌────────────┴────────────────────────────────────────┐
│  SIMULATION & MODELING LAYER                        │
│  - Physics simulation (Gazebo/Isaac/Unity)          │
│  - Behavior models                                  │
│  - What-if scenarios                                │
└────────────┬────────────────────────────────────────┘
             │
┌────────────┴────────────────────────────────────────┐
│  DATA SYNCHRONIZATION LAYER                         │
│  - State mirroring (bidirectional)                  │
│  - Event streaming (Kafka/MQTT)                     │
│  - Time synchronization                             │
└────────────┬────────────────────────────────────────┘
             │
┌────────────┴────────────────────────────────────────┐
│  PHYSICAL TWIN (ROBOT)                              │
│  - Sensors and actuators                            │
│  - Embedded control systems                         │
│  - Real-time operation                              │
└─────────────────────────────────────────────────────┘
```

**Key Components**
1. **Virtual Model**: CAD, URDF/SDF, mesh geometry
2. **Physics Engine**: Collision, contact, dynamics
3. **Sensor Simulation**: Camera, LiDAR, IMU, force/torque
4. **State Synchronization**: Real ↔ Virtual data flow
5. **Visualization**: 3D rendering, dashboards, telemetry

### 2. Gazebo Simulation Expertise

**SDF (Simulation Description Format) World**
```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="warehouse">
    <!-- Physics settings -->
    <physics name="ode_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Include robot model -->
    <include>
      <uri>model://mobile_robot</uri>
      <pose>0 0 0.1 0 0 0</pose>
    </include>
  </world>
</sdf>
```

**URDF Robot Model**
```xml
<?xml version="1.0"?>
<robot name="mobile_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="base_length" value="0.6"/>
  <xacro:property name="base_width" value="0.4"/>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="20"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Wheel macro -->
  <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="2"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0 ${reflect*base_width/2} 0" rpy="${pi/2} 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>

    <!-- Gazebo plugin for wheel control -->
    <gazebo reference="${prefix}_wheel">
      <material>Gazebo/Black</material>
    </gazebo>
  </xacro:macro>

  <!-- Instantiate wheels -->
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>

  <!-- LiDAR sensor -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.07"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.07"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo>
    <!-- Differential drive controller -->
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/mobile_robot</namespace>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>${base_width}</wheel_separation>
      <wheel_diameter>${2*wheel_radius}</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>

    <!-- LiDAR sensor plugin -->
    <plugin name="lidar" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/mobile_robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </gazebo>

  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.2</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
    </sensor>
  </gazebo>
</robot>
```

**Gazebo ROS2 Launch**
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot = get_package_share_directory('my_robot_description')

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_robot, 'worlds', 'warehouse.world')}.items()
    )

    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mobile_robot',
            '-file', os.path.join(pkg_robot, 'urdf', 'robot.urdf'),
            '-x', '0', '-y', '0', '-z', '0.1'
        ]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(os.path.join(pkg_robot, 'urdf', 'robot.urdf')).read()}]
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot
    ])
```

### 3. NVIDIA Isaac Sim Expertise

**Standalone Python Script**
```python
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Create world
world = World(stage_units_in_meters=1.0)

# Add ground plane
world.scene.add_default_ground_plane()

# Load robot from USD
assets_root_path = get_assets_root_path()
robot_usd_path = assets_root_path + "/Isaac/Robots/Franka/franka.usd"
add_reference_to_stage(usd_path=robot_usd_path, prim_path="/World/Franka")

# Get robot object
robot = world.scene.add(Robot(prim_path="/World/Franka", name="franka"))

# Reset world
world.reset()

# Simulation loop
for i in range(1000):
    world.step(render=True)

    # Get robot state
    position, orientation = robot.get_world_pose()

    # Set joint positions (simple sine wave motion)
    joint_positions = np.array([np.sin(i * 0.01)] * robot.num_dof)
    robot.set_joint_positions(joint_positions)

simulation_app.close()
```

**Isaac Sim with ROS2 Bridge**
```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS2 bridge
enable_extension("omni.isaac.ros2_bridge")

from omni.isaac.ros2_bridge import ROS2Bridge

# Create world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Add robot
from omni.isaac.franka import Franka
robot = world.scene.add(Franka(prim_path="/World/Franka", name="franka"))

# Setup ROS2 bridge
ros2_bridge = ROS2Bridge()

# Publish joint states
ros2_bridge.publish_joint_state(
    prim_path="/World/Franka",
    topic_name="/franka/joint_states"
)

# Subscribe to joint commands
ros2_bridge.subscribe_joint_command(
    prim_path="/World/Franka",
    topic_name="/franka/joint_commands"
)

# Publish camera
ros2_bridge.publish_camera(
    prim_path="/World/Camera",
    topic_name="/camera/image_raw",
    frame_id="camera_optical_frame"
)

world.reset()

# Run simulation
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

### 4. Unity Simulation with ROS

**Unity ROS Connection**
```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine;

public class LidarPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/scan";
    public float publishFrequency = 10f;

    private float timeElapsed;
    private LaserScanMsg laserScan;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);

        // Initialize laser scan message
        laserScan = new LaserScanMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg(),
            angle_min = -Mathf.PI,
            angle_max = Mathf.PI,
            angle_increment = Mathf.PI / 180f,
            time_increment = 0f,
            scan_time = 1f / publishFrequency,
            range_min = 0.2f,
            range_max = 10f,
            ranges = new float[360],
            intensities = new float[360]
        };
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 1f / publishFrequency)
        {
            // Perform raycasting for each angle
            for (int i = 0; i < 360; i++)
            {
                float angle = laserScan.angle_min + i * laserScan.angle_increment;
                Vector3 direction = Quaternion.Euler(0, angle * Mathf.Rad2Deg, 0) * transform.forward;

                RaycastHit hit;
                if (Physics.Raycast(transform.position, direction, out hit, laserScan.range_max))
                {
                    laserScan.ranges[i] = hit.distance;
                    laserScan.intensities[i] = 1.0f;
                }
                else
                {
                    laserScan.ranges[i] = float.PositiveInfinity;
                    laserScan.intensities[i] = 0f;
                }
            }

            laserScan.header.stamp = new RosMessageTypes.Std.TimeMsg
            {
                sec = (int)Time.time,
                nanosec = (uint)((Time.time % 1) * 1e9)
            };

            ros.Publish(topicName, laserScan);
            timeElapsed = 0;
        }
    }
}
```

### 5. Sim-to-Real Transfer Techniques

**Domain Randomization**
```python
# Randomize physics parameters
def randomize_physics():
    friction = np.random.uniform(0.5, 1.5)
    restitution = np.random.uniform(0, 0.3)
    mass_scale = np.random.uniform(0.9, 1.1)
    return friction, restitution, mass_scale

# Randomize visual appearance
def randomize_visuals():
    # Lighting
    light_intensity = np.random.uniform(0.5, 2.0)
    light_direction = np.random.uniform(-np.pi, np.pi, size=3)

    # Textures
    texture_scale = np.random.uniform(0.8, 1.2)
    color_tint = np.random.uniform(0.8, 1.2, size=3)

    # Camera
    camera_noise = np.random.uniform(0, 0.05)

    return light_intensity, light_direction, texture_scale, color_tint, camera_noise

# Apply randomization each episode
for episode in range(num_episodes):
    friction, restitution, mass_scale = randomize_physics()
    apply_physics_params(friction, restitution, mass_scale)

    light_intensity, light_dir, tex_scale, color, noise = randomize_visuals()
    apply_visual_params(light_intensity, light_dir, tex_scale, color, noise)

    run_episode()
```

**System Identification**
- Measure real robot parameters (mass, inertia, friction)
- Update simulation model to match
- Iteratively refine until sim matches real behavior
- Use trajectory following experiments for validation

**Calibration Workflow**
1. Perform known motions on real robot
2. Collect sensor data (joint angles, torques, end-effector pose)
3. Run same motions in simulation
4. Compare outputs, compute error metrics
5. Optimize simulation parameters to minimize error
6. Validate on new trajectories

**Reality Gap Mitigation**
- Use accurate CAD models
- Measure and match physical properties
- Model sensor noise realistically
- Include actuator dynamics and delays
- Test extensively in sim before real deployment

### 6. Digital Twin Data Synchronization

**Bidirectional State Sync**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import socket
import json

class DigitalTwinSync(Node):
    def __init__(self):
        super().__init__('digital_twin_sync')

        # Subscribe to real robot state
        self.real_state_sub = self.create_subscription(
            JointState, '/robot/joint_states',
            self.real_state_callback, 10
        )

        # Publish to simulation
        self.sim_state_pub = self.create_publisher(
            JointState, '/sim/robot/joint_commands', 10
        )

        # Socket connection to simulation
        self.sim_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sim_socket.connect(('localhost', 12345))

    def real_state_callback(self, msg):
        # Forward real robot state to simulation
        self.sim_state_pub.publish(msg)

        # Also send via socket for non-ROS simulation
        state_dict = {
            'timestamp': self.get_clock().now().to_msg(),
            'joint_names': msg.name,
            'positions': msg.position,
            'velocities': msg.velocity,
            'efforts': msg.effort
        }
        self.sim_socket.send(json.dumps(state_dict).encode() + b'\n')
```

**Time Synchronization**
```python
from rclpy.time import Time
from rclpy.clock import ClockType

class TimeSyncNode(Node):
    def __init__(self):
        super().__init__('time_sync',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # Use simulation time
        self.get_clock().use_sim_time = True

        # Subscribe to /clock topic from simulation
        self.clock_sub = self.create_subscription(
            Clock, '/clock',
            self.clock_callback, 10
        )

    def clock_callback(self, msg):
        # Simulation time is now synchronized
        sim_time = Time.from_msg(msg.clock)
        self.get_logger().info(f'Sim time: {sim_time.nanoseconds / 1e9:.3f}')
```

### 7. Performance Optimization

**Gazebo Optimization**
- Use simplified collision geometries (cylinders/boxes instead of meshes)
- Reduce physics update rate if real-time not needed
- Disable shadows and reduce rendering quality
- Use headless mode for batch experiments
- Parallel worlds with Ignition Gazebo

**Isaac Sim Optimization**
- Enable GPU physics acceleration
- Use RTX raytracing only when needed
- Simplify meshes (decimation, LOD)
- Batch simulations in headless mode
- Use distributed rendering for multiple cameras

**Unity Optimization**
- Use physics layers to reduce collision checks
- LOD (Level of Detail) for distant objects
- Occlusion culling for rendering
- Object pooling for spawned entities
- Profiler to identify bottlenecks

## Best Practices

- [ ] Model validation against real robot data
- [ ] Domain randomization for robustness
- [ ] Accurate physics parameters measured from hardware
- [ ] Sensor models include realistic noise
- [ ] Version control for simulation worlds
- [ ] Automated testing in simulation CI/CD
- [ ] Performance profiling and optimization
- [ ] Documentation of sim-to-real gaps
- [ ] Graceful degradation when sim diverges from real

## Common Pitfalls

1. **Overly Optimistic Friction**: Sim objects slide too little
2. **Missing Compliance**: Rigid links where real robot flexes
3. **Idealized Sensors**: No noise, perfect accuracy
4. **Instant Actuation**: No motor dynamics or delays
5. **Deterministic Worlds**: Same initial conditions every time
6. **Ignoring Latency**: Zero communication delays in sim

This skill empowers Claude with expert digital twin and simulation capabilities for robotics, enabling high-fidelity virtual testing and sim-to-real transfer.
