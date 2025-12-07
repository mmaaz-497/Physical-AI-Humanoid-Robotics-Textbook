name: cloud-native-robotics
description: Expert knowledge of cloud-native architectures, Kubernetes, Docker, and distributed systems for robotics
license: MIT

---

# Cloud-Native Robotics Intelligence

You are an expert in cloud-native architectures and distributed systems for robotics, with deep knowledge of containerization, orchestration, microservices, and edge-cloud hybrid deployments.

## Core Cloud-Native Concepts

### 1. Containerization with Docker

**Docker for ROS2 Applications**
```dockerfile
FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws
COPY src/ src/

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Setup entrypoint
COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "my_robot", "bringup.launch.py"]
```

**Multi-Stage Builds for Size Optimization**
```dockerfile
# Stage 1: Build
FROM ros:humble-ros-base AS builder
COPY src/ /ros2_ws/src/
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Stage 2: Runtime
FROM ros:humble-ros-base
COPY --from=builder /ros2_ws/install /ros2_ws/install
WORKDIR /ros2_ws
ENTRYPOINT ["/ros_entrypoint.sh"]
```

**Docker Compose for Multi-Container Systems**
```yaml
version: '3.8'

services:
  robot_driver:
    image: my_robot/driver:latest
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0  # Serial device
    network_mode: host  # For ROS2 DDS discovery
    privileged: true
    environment:
      - ROS_DOMAIN_ID=42

  perception:
    image: my_robot/perception:latest
    depends_on:
      - robot_driver
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]

  navigation:
    image: my_robot/navigation:latest
    depends_on:
      - perception
    volumes:
      - ./maps:/maps:ro

  cloud_bridge:
    image: my_robot/cloud_bridge:latest
    environment:
      - MQTT_BROKER=mqtt.example.com
      - ROBOT_ID=${ROBOT_ID}
```

**Best Practices**
- Use official base images (ros:humble, nvidia/cuda)
- Multi-stage builds to reduce image size
- .dockerignore to exclude unnecessary files
- Layer caching for faster builds
- Non-root user for security
- Health checks for container monitoring
- Version tags, never `:latest` in production

### 2. Kubernetes for Robot Fleet Management

**Deployment Manifest**
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: robot-fleet-manager
  labels:
    app: fleet-manager
spec:
  replicas: 3
  selector:
    matchLabels:
      app: fleet-manager
  template:
    metadata:
      labels:
        app: fleet-manager
    spec:
      containers:
      - name: manager
        image: robotics/fleet-manager:v1.2.3
        ports:
        - containerPort: 8080
        env:
        - name: DB_HOST
          valueFrom:
            configMapKeyRef:
              name: fleet-config
              key: database_host
        - name: DB_PASSWORD
          valueFrom:
            secretKeyRef:
              name: fleet-secrets
              key: db_password
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
        livenessProbe:
          httpGet:
            path: /health
            port: 8080
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /ready
            port: 8080
          initialDelaySeconds: 5
          periodSeconds: 5
```

**StatefulSet for Robots**
```yaml
apiVersion: apps/v1
kind: StatefulSet
metadata:
  name: robot-simulator
spec:
  serviceName: "robot-sim"
  replicas: 10
  selector:
    matchLabels:
      app: robot-sim
  template:
    metadata:
      labels:
        app: robot-sim
    spec:
      containers:
      - name: gazebo
        image: robotics/gazebo-sim:latest
        env:
        - name: ROBOT_ID
          valueFrom:
            fieldRef:
              fieldPath: metadata.name
        volumeMounts:
        - name: robot-data
          mountPath: /data
  volumeClaimTemplates:
  - metadata:
      name: robot-data
    spec:
      accessModes: [ "ReadWriteOnce" ]
      resources:
        requests:
          storage: 10Gi
```

**Service for ROS2 DDS**
```yaml
apiVersion: v1
kind: Service
metadata:
  name: ros2-discovery
spec:
  clusterIP: None  # Headless service for peer discovery
  selector:
    ros-domain: "42"
  ports:
  - name: dds-discovery
    port: 7400
    protocol: UDP
  - name: dds-user
    port: 7401
    protocol: UDP
```

**ConfigMap for ROS2 Parameters**
```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: robot-params
data:
  robot.yaml: |
    /**:
      ros__parameters:
        max_velocity: 1.0
        max_acceleration: 0.5
        safety_radius: 0.3
```

### 3. Edge-Cloud Hybrid Architectures

**Architecture Pattern**
```
┌─────────────────────────────────────────────────────────┐
│                      CLOUD LAYER                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ Fleet Manager│  │ Data Lake    │  │ ML Training  │  │
│  │ (K8s)        │  │ (S3/MinIO)   │  │ (GPU Cluster)│  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└────────────┬────────────────────────────────────────────┘
             │ MQTT/gRPC/REST
             │
┌────────────┴────────────────────────────────────────────┐
│                      EDGE LAYER                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ Edge K3s     │  │ Local Cache  │  │ Time-Series  │  │
│  │ Cluster      │  │ (Redis)      │  │ DB (InfluxDB)│  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└────────────┬────────────────────────────────────────────┘
             │ ROS2 DDS (local network)
             │
┌────────────┴────────────────────────────────────────────┐
│                     ROBOT LAYER                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ Perception   │  │ Planning     │  │ Control      │  │
│  │ (Docker)     │  │ (Docker)     │  │ (Docker)     │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────┘
```

**Data Synchronization Strategy**
- **Real-time**: Critical telemetry via MQTT (QoS 1)
- **Near-real-time**: Sensor data via Kafka streams
- **Batch**: Logs and diagnostics via S3 sync (hourly)
- **On-demand**: Large files (maps, models) via HTTP/gRPC

**Edge Computing with K3s**
```yaml
# K3s deployment (lightweight Kubernetes for edge)
# Install: curl -sfL https://get.k3s.io | sh -

# Deploy robot workload on edge cluster
apiVersion: apps/v1
kind: DaemonSet  # One pod per edge node
metadata:
  name: robot-edge-agent
spec:
  selector:
    matchLabels:
      app: edge-agent
  template:
    metadata:
      labels:
        app: edge-agent
    spec:
      hostNetwork: true  # Access host network for ROS2
      containers:
      - name: agent
        image: robotics/edge-agent:latest
        securityContext:
          privileged: true  # Access hardware devices
        volumeMounts:
        - name: dev
          mountPath: /dev
      volumes:
      - name: dev
        hostPath:
          path: /dev
```

### 4. Microservices Architecture for Robotics

**Service Decomposition**
```
Robot Monolith
      ↓
┌─────────────────────────────────────────────────────┐
│  Perception Service   │  Planning Service           │
│  - Camera driver      │  - Path planning            │
│  - Object detection   │  - Trajectory optimization  │
│  - Tracking           │  - Collision checking       │
├───────────────────────┼─────────────────────────────┤
│  Control Service      │  Localization Service       │
│  - Motor control      │  - SLAM                     │
│  - PID loops          │  - Particle filter          │
│  - Safety checks      │  - Map management           │
└───────────────────────┴─────────────────────────────┘
```

**gRPC for Inter-Service Communication**
```protobuf
// path_planning.proto
syntax = "proto3";

service PathPlanner {
  rpc PlanPath(PathRequest) returns (PathResponse);
  rpc ValidatePath(Path) returns (ValidationResult);
}

message PathRequest {
  Pose start = 1;
  Pose goal = 2;
  repeated Obstacle obstacles = 3;
  PlannerConfig config = 4;
}

message PathResponse {
  Path path = 1;
  double cost = 2;
  double planning_time = 3;
  string status = 4;
}
```

**Service Mesh with Istio**
```yaml
apiVersion: networking.istio.io/v1alpha3
kind: VirtualService
metadata:
  name: path-planner
spec:
  hosts:
  - path-planner
  http:
  - match:
    - headers:
        robot-priority:
          exact: "high"
    route:
    - destination:
        host: path-planner
        subset: gpu-accelerated
  - route:
    - destination:
        host: path-planner
        subset: cpu-only
      weight: 100
```

### 5. Message Brokers and Event-Driven Architecture

**MQTT for Robot-Cloud Communication**
```python
import paho.mqtt.client as mqtt
import json

class RobotCloudBridge:
    def __init__(self, robot_id, broker_host):
        self.robot_id = robot_id
        self.client = mqtt.Client(client_id=f"robot_{robot_id}")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # TLS for security
        self.client.tls_set(ca_certs="/etc/ssl/certs/ca.crt")
        self.client.username_pw_set(robot_id, "token")

        self.client.connect(broker_host, 8883, 60)

    def on_connect(self, client, userdata, flags, rc):
        # Subscribe to commands for this robot
        client.subscribe(f"robots/{self.robot_id}/commands/#")

    def on_message(self, client, userdata, msg):
        payload = json.loads(msg.payload)
        # Handle command

    def publish_telemetry(self, data):
        topic = f"robots/{self.robot_id}/telemetry"
        self.client.publish(topic, json.dumps(data), qos=1)
```

**Kafka for High-Throughput Sensor Data**
```python
from kafka import KafkaProducer
import msgpack

producer = KafkaProducer(
    bootstrap_servers=['kafka:9092'],
    value_serializer=lambda v: msgpack.packb(v),
    compression_type='lz4',
    batch_size=32768,
    linger_ms=10
)

def publish_pointcloud(robot_id, timestamp, points):
    topic = f'sensor_data.pointcloud.{robot_id}'
    message = {
        'timestamp': timestamp,
        'points': points.tobytes(),
        'shape': points.shape
    }
    producer.send(topic, value=message)
```

### 6. Observability and Monitoring

**Prometheus Metrics**
```python
from prometheus_client import Counter, Histogram, Gauge, start_http_server

# Define metrics
path_planning_requests = Counter(
    'path_planning_requests_total',
    'Total path planning requests',
    ['robot_id', 'status']
)

planning_duration = Histogram(
    'path_planning_duration_seconds',
    'Path planning duration',
    ['robot_id']
)

robot_battery = Gauge(
    'robot_battery_percentage',
    'Robot battery level',
    ['robot_id']
)

# Use in code
with planning_duration.labels(robot_id='robot_01').time():
    path = plan_path(start, goal)
    path_planning_requests.labels(robot_id='robot_01', status='success').inc()

robot_battery.labels(robot_id='robot_01').set(85.5)

# Start metrics server
start_http_server(8000)
```

**Grafana Dashboard**
```json
{
  "dashboard": {
    "title": "Robot Fleet Overview",
    "panels": [
      {
        "title": "Active Robots",
        "targets": [
          {
            "expr": "count(robot_heartbeat{status='active'})"
          }
        ]
      },
      {
        "title": "Path Planning Latency",
        "targets": [
          {
            "expr": "histogram_quantile(0.95, path_planning_duration_seconds)"
          }
        ]
      }
    ]
  }
}
```

**Distributed Tracing with OpenTelemetry**
```python
from opentelemetry import trace
from opentelemetry.exporter.jaeger import JaegerExporter
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor

# Setup tracing
trace.set_tracer_provider(TracerProvider())
jaeger_exporter = JaegerExporter(
    agent_host_name="jaeger",
    agent_port=6831,
)
trace.get_tracer_provider().add_span_processor(
    BatchSpanProcessor(jaeger_exporter)
)

tracer = trace.get_tracer(__name__)

# Instrument code
with tracer.start_as_current_span("plan_path"):
    with tracer.start_as_current_span("collision_check"):
        is_valid = check_collisions(path)
    with tracer.start_as_current_span("optimize_path"):
        optimized_path = optimize(path)
```

### 7. CI/CD for Robotics

**GitLab CI Pipeline**
```yaml
stages:
  - build
  - test
  - deploy

variables:
  DOCKER_REGISTRY: registry.example.com
  IMAGE_NAME: robotics/perception

build:
  stage: build
  script:
    - docker build -t $IMAGE_NAME:$CI_COMMIT_SHA .
    - docker push $IMAGE_NAME:$CI_COMMIT_SHA

unit_test:
  stage: test
  script:
    - docker run $IMAGE_NAME:$CI_COMMIT_SHA pytest tests/unit

integration_test:
  stage: test
  script:
    - docker-compose -f docker-compose.test.yml up --abort-on-container-exit
    - docker-compose -f docker-compose.test.yml down

simulation_test:
  stage: test
  script:
    - kubectl apply -f k8s/test-env.yaml
    - kubectl wait --for=condition=ready pod -l app=gazebo-test --timeout=300s
    - kubectl exec gazebo-test -- ros2 launch test_suite simulation_tests.launch.py

deploy_staging:
  stage: deploy
  script:
    - helm upgrade --install robot-app ./helm-chart --namespace staging --set image.tag=$CI_COMMIT_SHA
  environment:
    name: staging
  only:
    - develop

deploy_production:
  stage: deploy
  script:
    - helm upgrade --install robot-app ./helm-chart --namespace production --set image.tag=$CI_COMMIT_SHA
  environment:
    name: production
  only:
    - main
  when: manual
```

### 8. Security Best Practices

**Secrets Management**
```yaml
# Sealed Secrets (encrypted secrets in Git)
apiVersion: bitnami.com/v1alpha1
kind: SealedSecret
metadata:
  name: robot-credentials
spec:
  encryptedData:
    api-key: AgBh7...encrypted...
    db-password: AgCx9...encrypted...
```

**Network Policies**
```yaml
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: robot-isolation
spec:
  podSelector:
    matchLabels:
      tier: robot
  policyTypes:
  - Ingress
  - Egress
  ingress:
  - from:
    - podSelector:
        matchLabels:
          tier: edge-gateway
  egress:
  - to:
    - podSelector:
        matchLabels:
          tier: backend
    ports:
    - protocol: TCP
      port: 5432  # PostgreSQL
```

**Pod Security Policies**
```yaml
apiVersion: policy/v1beta1
kind: PodSecurityPolicy
metadata:
  name: restricted-robot
spec:
  privileged: false
  allowPrivilegeEscalation: false
  runAsUser:
    rule: 'MustRunAsNonRoot'
  seLinux:
    rule: 'RunAsAny'
  fsGroup:
    rule: 'RunAsAny'
  volumes:
  - 'configMap'
  - 'emptyDir'
  - 'secret'
```

### 9. Scaling Strategies

**Horizontal Pod Autoscaling**
```yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: path-planner-hpa
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: path-planner
  minReplicas: 2
  maxReplicas: 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Pods
    pods:
      metric:
        name: planning_requests_per_second
      target:
        type: AverageValue
        averageValue: "100"
```

**Vertical Pod Autoscaling**
```yaml
apiVersion: autoscaling.k8s.io/v1
kind: VerticalPodAutoscaler
metadata:
  name: perception-vpa
spec:
  targetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: perception
  updatePolicy:
    updateMode: "Auto"
```

## Best Practices Checklist

- [ ] Use multi-stage Docker builds for smaller images
- [ ] Implement health checks in all containers
- [ ] Set resource requests and limits
- [ ] Use ConfigMaps for configuration, Secrets for credentials
- [ ] Implement distributed tracing for debugging
- [ ] Monitor with Prometheus, visualize with Grafana
- [ ] Use service mesh for traffic management
- [ ] Implement CI/CD pipelines with automated testing
- [ ] Apply network policies for security
- [ ] Use autoscaling for dynamic workloads
- [ ] Backup stateful data regularly
- [ ] Document deployment procedures

This skill empowers Claude with expert cloud-native robotics architecture knowledge, enabling sophisticated distributed system design for robot fleets and edge-cloud deployments.
