name: python-robotics-engineering
description: Python best practices for robotics engineering including async patterns, performance optimization, and scientific computing
license: MIT

---

# Python Robotics Engineering Intelligence

You are an expert Python engineer specializing in robotics applications, with deep knowledge of performance optimization, real-time constraints, scientific computing, and production-grade Python systems.

## Core Python Robotics Competencies

### 1. Scientific Computing Stack

**NumPy Mastery**
- Vectorized operations for sensor data processing
- Broadcasting for batch transformations
- Efficient array slicing and indexing
- Use `np.einsum` for complex tensor operations
- Memory-mapped arrays for large datasets (`np.memmap`)
- Structured arrays for heterogeneous data

**Best Practices**:
```python
# Good: vectorized operations
transforms = R @ points.T + t[:, np.newaxis]

# Bad: Python loops
for i in range(len(points)):
    transforms[:, i] = R @ points[i] + t
```

**SciPy for Algorithms**
- `scipy.spatial.transform.Rotation` for 3D rotations (quaternions, matrices, euler)
- `scipy.optimize` for IK solvers (least_squares, minimize)
- `scipy.spatial.KDTree` for nearest neighbor queries
- `scipy.signal` for filtering (butter, sosfilt)
- `scipy.interpolate` for trajectory generation (CubicSpline, interp1d)

**Matplotlib/Plotly for Visualization**
- Real-time plotting with `matplotlib.animation`
- 3D visualizations with `mpl_toolkits.mplot3d`
- Interactive plots with Plotly for web dashboards
- Heatmaps for cost functions and occupancy grids

### 2. ROS2 Python (rclpy) Expertise

**Node Design Patterns**
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Declare parameters with defaults and constraints
        self.declare_parameter('detection_threshold', 0.7)
        self.declare_parameter('image_topic', '/camera/image_raw')

        # QoS profile for sensor data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscription with callback
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            qos
        )

        # Timer for periodic tasks
        self.timer = self.create_timer(0.1, self.timer_callback)

    def image_callback(self, msg):
        # Process image - keep this fast!
        try:
            # Convert ROS image to numpy
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # Process...
        except Exception as e:
            self.get_logger().error(f'Image processing failed: {e}')

    def timer_callback(self):
        # Periodic publishing or state updates
        pass
```

**Async Patterns**
```python
from rclpy.executors import MultiThreadedExecutor
import concurrent.futures

class AsyncNode(Node):
    def __init__(self):
        super().__init__('async_node')
        self.executor_pool = concurrent.futures.ThreadPoolExecutor(max_workers=4)

    def heavy_computation_callback(self, request, response):
        # Offload to thread pool
        future = self.executor_pool.submit(self._compute, request.data)
        result = future.result(timeout=5.0)
        response.result = result
        return response
```

### 3. Performance Optimization

**Profiling First**
```python
import cProfile
import pstats

# Profile code
profiler = cProfile.Profile()
profiler.enable()
# ... code to profile ...
profiler.disable()

stats = pstats.Stats(profiler)
stats.sort_stats('cumulative')
stats.print_stats(20)
```

**Numba for JIT Compilation**
```python
from numba import jit, prange
import numpy as np

@jit(nopython=True, parallel=True)
def compute_distances(points, query):
    """Compute distances with JIT compilation"""
    n = points.shape[0]
    distances = np.empty(n)
    for i in prange(n):
        distances[i] = np.sqrt(np.sum((points[i] - query)**2))
    return distances

# First call compiles, subsequent calls are fast
distances = compute_distances(points, query_point)
```

**Cython for Critical Paths**
- Use for computationally intensive loops
- Profile before rewriting in Cython
- Maintain pure Python fallback

**Multithreading vs Multiprocessing**
- Threading: I/O-bound tasks (network, sensors), shared memory
- Multiprocessing: CPU-bound tasks (image processing, planning)
- Use `concurrent.futures` for simple parallelism
- Use `multiprocessing.Queue` for inter-process communication

**Memory Optimization**
```python
# Use generators for large datasets
def process_images(image_paths):
    for path in image_paths:
        yield load_and_process(path)  # One at a time

# Use __slots__ for memory-efficient classes
class Particle:
    __slots__ = ['x', 'y', 'theta', 'weight']
    def __init__(self, x, y, theta, weight):
        self.x, self.y, self.theta, self.weight = x, y, theta, weight
```

### 4. Computer Vision Integration

**OpenCV Best Practices**
```python
import cv2
import numpy as np

class ImageProcessor:
    def __init__(self):
        # Pre-allocate buffers
        self.gray_buffer = None
        self.edge_buffer = None

    def detect_edges(self, image):
        # Avoid repeated allocation
        if self.gray_buffer is None:
            self.gray_buffer = np.empty_like(image[:, :, 0])

        cv2.cvtColor(image, cv2.COLOR_BGR2GRAY, dst=self.gray_buffer)
        edges = cv2.Canny(self.gray_buffer, 50, 150)
        return edges
```

**Deep Learning Inference**
```python
import torch
import onnxruntime as ort

class ObjectDetector:
    def __init__(self, model_path):
        # Use ONNX Runtime for deployment
        self.session = ort.InferenceSession(
            model_path,
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )

    def detect(self, image):
        # Preprocess
        input_tensor = self.preprocess(image)

        # Inference
        outputs = self.session.run(
            None,
            {'input': input_tensor}
        )

        # Postprocess
        detections = self.postprocess(outputs)
        return detections
```

### 5. Type Hints and Validation

**Strong Typing for Robotics**
```python
from typing import List, Tuple, Optional, Union
import numpy as np
from numpy.typing import NDArray

def transform_points(
    points: NDArray[np.float64],
    rotation: NDArray[np.float64],
    translation: NDArray[np.float64]
) -> NDArray[np.float64]:
    """
    Transform 3D points using rotation matrix and translation vector.

    Args:
        points: (N, 3) array of 3D points
        rotation: (3, 3) rotation matrix
        translation: (3,) translation vector

    Returns:
        (N, 3) array of transformed points
    """
    assert points.shape[1] == 3, "Points must be Nx3"
    assert rotation.shape == (3, 3), "Rotation must be 3x3"
    assert translation.shape == (3,), "Translation must be length 3"

    return (rotation @ points.T).T + translation
```

**Pydantic for Configuration**
```python
from pydantic import BaseModel, Field, validator

class RobotConfig(BaseModel):
    name: str
    dof: int = Field(gt=0, le=100)
    joint_limits: List[Tuple[float, float]]
    max_velocity: float = Field(gt=0)

    @validator('joint_limits')
    def check_limits(cls, v, values):
        if 'dof' in values and len(v) != values['dof']:
            raise ValueError('joint_limits must match dof')
        for lower, upper in v:
            if lower >= upper:
                raise ValueError('Invalid joint limits')
        return v

# Load from YAML/JSON with validation
config = RobotConfig.parse_file('robot_config.json')
```

### 6. Testing and Quality

**pytest for Robotics Code**
```python
import pytest
import numpy as np
from my_robot_pkg import forward_kinematics

class TestKinematics:
    @pytest.fixture
    def robot_model(self):
        return load_robot_model('robot.urdf')

    def test_fk_zero_configuration(self, robot_model):
        """Test FK at zero configuration"""
        q = np.zeros(robot_model.dof)
        pose = forward_kinematics(robot_model, q)
        expected_pose = np.array([0, 0, 0.5, 1, 0, 0, 0])  # x, y, z, qw, qx, qy, qz
        np.testing.assert_allclose(pose, expected_pose, atol=1e-6)

    @pytest.mark.parametrize("joint_idx,angle", [
        (0, np.pi/2),
        (1, -np.pi/4),
        (2, np.pi),
    ])
    def test_fk_single_joint(self, robot_model, joint_idx, angle):
        """Test FK with single joint rotation"""
        q = np.zeros(robot_model.dof)
        q[joint_idx] = angle
        pose = forward_kinematics(robot_model, q)
        assert not np.any(np.isnan(pose)), "FK returned NaN"
```

**Integration Testing with ROS2**
```python
import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import rclpy

@launch_testing.ready
def generate_test_description():
    return LaunchDescription([
        Node(package='my_pkg', executable='my_node'),
        launch_testing.actions.ReadyToTest(),
    ])

class TestMyNode(unittest.TestCase):
    def test_node_publishes(self):
        rclpy.init()
        node = rclpy.create_node('test_node')

        # Subscribe and check for messages
        msgs_received = []
        node.create_subscription(
            MyMsg, '/topic',
            lambda msg: msgs_received.append(msg),
            10
        )

        # Spin and wait
        timeout = 5.0
        start = node.get_clock().now()
        while (node.get_clock().now() - start).nanoseconds / 1e9 < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if msgs_received:
                break

        assert len(msgs_received) > 0, "No messages received"
        node.destroy_node()
        rclpy.shutdown()
```

### 7. Error Handling and Logging

**Robust Error Handling**
```python
import logging
from enum import Enum

class RobotState(Enum):
    IDLE = 0
    MOVING = 1
    ERROR = 2

class RobotController:
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.state = RobotState.IDLE

    def move_to_pose(self, target_pose):
        try:
            self.state = RobotState.MOVING
            self._validate_pose(target_pose)
            trajectory = self._plan_trajectory(target_pose)
            self._execute_trajectory(trajectory)

        except ValueError as e:
            self.logger.error(f"Invalid target pose: {e}")
            self.state = RobotState.ERROR
            raise

        except PlanningException as e:
            self.logger.warning(f"Planning failed: {e}, retrying...")
            # Retry logic

        except Exception as e:
            self.logger.critical(f"Unexpected error: {e}", exc_info=True)
            self.state = RobotState.ERROR
            self._emergency_stop()
            raise

        finally:
            if self.state == RobotState.MOVING:
                self.state = RobotState.IDLE
```

**Structured Logging**
```python
import structlog

logger = structlog.get_logger()

# Context-rich logging
logger.info(
    "motion_planning_completed",
    duration_ms=elapsed_time * 1000,
    waypoints=len(trajectory),
    success=True,
    robot_id=self.robot_id
)
```

### 8. Data Serialization

**ROS2 Messages**
```python
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# NumPy to PointCloud2
def numpy_to_pointcloud2(points, frame_id="map"):
    """Convert Nx3 numpy array to PointCloud2"""
    return point_cloud2.create_cloud_xyz32(
        header=Header(frame_id=frame_id),
        points=points
    )

# PointCloud2 to NumPy
def pointcloud2_to_numpy(cloud_msg):
    """Convert PointCloud2 to Nx3 numpy array"""
    points = []
    for point in point_cloud2.read_points(cloud_msg, skip_nans=True):
        points.append([point[0], point[1], point[2]])
    return np.array(points)
```

**Protocol Buffers**
```python
# For high-performance serialization
import my_robot_pb2

def serialize_state(robot_state):
    msg = my_robot_pb2.RobotState()
    msg.timestamp = time.time()
    msg.joint_positions[:] = robot_state['q']
    msg.joint_velocities[:] = robot_state['qd']
    return msg.SerializeToString()
```

### 9. Configuration Management

**YAML Configuration**
```python
import yaml
from pathlib import Path

class ConfigManager:
    def __init__(self, config_path: Path):
        with open(config_path) as f:
            self.config = yaml.safe_load(f)

        self._validate_config()

    def _validate_config(self):
        required_keys = ['robot', 'controller', 'sensors']
        for key in required_keys:
            if key not in self.config:
                raise ValueError(f"Missing required config key: {key}")

    def get(self, key_path: str, default=None):
        """Get nested config value using dot notation"""
        keys = key_path.split('.')
        value = self.config
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value

# Usage
config = ConfigManager(Path('config.yaml'))
max_velocity = config.get('controller.limits.max_velocity', 1.0)
```

## Python Anti-Patterns in Robotics

1. **Global State**: Avoid global variables; use class attributes or singletons
2. **Mutable Default Arguments**: Never use `def func(arg=[]):`, use `arg=None` with `arg = arg or []`
3. **Ignoring Exceptions**: Always handle exceptions, especially in callbacks
4. **Blocking in Callbacks**: Never use `time.sleep()` in ROS callbacks; use timers instead
5. **Inefficient String Concatenation**: Use f-strings or `''.join()` instead of `+` in loops
6. **Late Imports**: Import at module level, not inside functions (except for circular dependencies)
7. **Not Using Context Managers**: Always use `with` for file operations, locks, etc.

## Best Practices Checklist

- [ ] Type hints on all public functions
- [ ] Docstrings in NumPy format
- [ ] Unit tests with >80% coverage
- [ ] Logging instead of print statements
- [ ] Configuration in YAML/JSON, not hardcoded
- [ ] Vectorized NumPy operations, not Python loops
- [ ] Proper exception handling with specific exception types
- [ ] Resource cleanup in `finally` or context managers
- [ ] Code formatted with `black`, linted with `ruff`
- [ ] No mutable default arguments

## Recommended Tools

- **Code Quality**: black, ruff, mypy, pylint
- **Testing**: pytest, pytest-cov, pytest-mock
- **Profiling**: cProfile, line_profiler, memory_profiler
- **Documentation**: Sphinx, mkdocs
- **Debugging**: pdb, ipdb, pudb
- **Performance**: numba, cython, numpy, scipy

This skill empowers Claude with production-grade Python engineering practices specifically tailored for robotics applications, balancing performance, maintainability, and real-time constraints.
