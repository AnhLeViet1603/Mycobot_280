# Conveyor Sorting Robot Demo with ROS 2 + Gazebo

## 1. Project Overview

### Project Name

**ROS 2 Conveyor Sorting Robot Simulation**

### Objective

Build a complete industrial automation simulation in **ROS 2 + Gazebo** where:

* Objects move continuously on a conveyor belt.
* A camera detects each object.
* A vision node classifies the object (by color or using YOLO).
* A decision node determines whether the object should be rejected.
* A robotic actuator (servo pusher or robotic arm) removes incorrect objects from the conveyor.
* Correct objects continue to the end of the conveyor.

This project demonstrates a complete perception → decision → action pipeline commonly found in industrial automation.

---

# 2. Demo Scenario

Suppose a factory only accepts **blue cubes**.

Objects appear randomly:

* Blue Cube ✅ Keep
* Red Cube ❌ Reject
* Green Cube ❌ Reject

Workflow:

```text
Spawner
    │
    ▼
 Conveyor Belt
    │
    ▼
 Gazebo Camera
    │
    ▼
 Vision Node
    │
    ▼
 Classification
    │
    ▼
 Decision Node
    │
 ┌──┴─────────────┐
 │                │
 ▼                ▼
Keep         Reject
 │                │
 ▼                ▼
 Continue     Robot Push
```

---

# 3. Learning Goals

After completing this project you should understand:

* ROS 2 workspace
* ROS 2 Nodes
* Publishers/Subscribers
* Services
* Actions (optional)
* Parameters
* Launch files
* TF2
* URDF
* ros2_control
* Gazebo
* RViz
* OpenCV
* YOLO integration
* Robot control
* State machines

---

# 4. Recommended Technology Stack

| Component     | Technology          |
| ------------- | ------------------- |
| OS            | Ubuntu 24.04        |
| ROS           | ROS 2 Jazzy         |
| Simulator     | Gazebo Harmonic     |
| Robot         | URDF + ros2_control |
| Vision        | OpenCV              |
| AI (optional) | YOLOv8              |
| Language      | Python + C++        |
| Build         | colcon              |

---

# 5. Overall Architecture

```text
                    Gazebo

 ┌──────────────────────────────────────────────┐

        Conveyor Belt

 Cube → Cube → Cube → Cube → Cube

               ▲
               │
         Camera Sensor

 └──────────────────────────────────────────────┘

             │
             ▼

      /camera/image_raw

             │
             ▼

      Vision Node

             │

 Detection Result

             ▼

     Decision Node

             │

      Reject Command

             ▼

     Robot Controller

             │

      ros2_control

             ▼

       Robot Motion
```

---

# 6. Workspace Structure

```text
ros2_ws/

src/

    conveyor_description/
        urdf/
        meshes/

    conveyor_gazebo/
        worlds/
        launch/

    sorting_robot_description/
        urdf/
        meshes/

    sorting_robot_controller/

    object_spawner/

    vision_node/

    decision_node/

    custom_interfaces/

    bringup/
```

---

# 7. Development Roadmap

---

## Phase 1 — Build the Conveyor

### Goal

Create a working conveyor belt simulation.

Tasks

* Create conveyor model
* Apply belt motion
* Verify cubes move correctly

Deliverable

```
Cube moves continuously.
```

Estimated time

**1–2 days**

---

## Phase 2 — Spawn Objects

Create

* Red cube
* Blue cube
* Green cube

Spawn randomly every few seconds.

Deliverable

```
Random colored cubes appear automatically.
```

Estimated time

**1 day**

---

## Phase 3 — Add Camera

Install a Gazebo camera.

Publish

```
/camera/image_raw
```

Visualize using RViz.

Deliverable

```
Live camera stream.
```

Estimated time

**1 day**

---

## Phase 4 — Object Detection

### Beginner

Use OpenCV.

Detect

* red
* blue
* green

using HSV thresholds.

### Advanced

Replace OpenCV with YOLOv8.

Deliverable

```
Detected object class.
```

Published message:

```
DetectedObject.msg

string class_name

geometry_msgs/Point position
```

Estimated time

**2–4 days**

---

## Phase 5 — Decision Node

Logic:

```
if object == blue

    KEEP

else

    REJECT
```

Publish

```
/reject_object
```

Deliverable

Decision messages.

Estimated time

**1 day**

---

## Phase 6 — Robot Actuator

Two implementation options.

---

### Option A (Recommended)

Servo Pusher

```
Conveyor

----------------------

            |

            |

        Servo Arm
```

Motion

```
Idle

↓

Rotate

↓

Push

↓

Return
```

Advantages

* Simple
* Stable
* Easy ros2_control integration

Difficulty

⭐⭐☆☆☆

---

### Option B

Robotic Arm

Possible robots

* UR5
* Panda
* xArm

Motion

```
Move

↓

Reach

↓

Push

↓

Return
```

Difficulty

⭐⭐⭐⭐☆

---

## Phase 7 — Robot Control

Implement

* Joint controller
* PID tuning
* ros2_control

Topics

```
/joint_states

/joint_commands
```

Deliverable

Robot responds to commands.

Estimated time

**3–5 days**

---

## Phase 8 — Full Integration

Pipeline

```
Cube

↓

Camera

↓

Vision

↓

Classification

↓

Decision

↓

Robot

↓

Reject
```

Final demonstration

```
Blue cube

↓

Pass

----------------------

Red cube

↓

Robot pushes away

----------------------

Green cube

↓

Robot pushes away
```

---

# 8. Optional Enhancements

## Multiple Cameras

Top camera

Side camera

---

## Barcode Recognition

Instead of color

Detect

* QR
* Barcode

---

## Shape Classification

Detect

* Cube
* Cylinder
* Sphere

---

## Size Classification

Reject objects larger than threshold.

---

## Defect Detection

Use YOLO

Reject damaged products.

---

## Database Logging

Save

* timestamp
* object class
* accepted/rejected

SQLite or PostgreSQL.

---

## Dashboard

Build a dashboard showing

* Accepted count
* Rejected count
* Throughput
* Camera feed

---

# 9. Skills Demonstrated

This project showcases experience with

* ROS 2
* Gazebo
* URDF
* ros2_control
* RViz
* OpenCV
* YOLO
* TF2
* Python
* C++
* Industrial Robotics
* Computer Vision
* Robot Simulation

---

# 10. Suggested Development Timeline

| Week | Tasks                                                         |
| ---- | ------------------------------------------------------------- |
| 1    | Install ROS 2 + Gazebo, create workspace, build conveyor      |
| 2    | Spawn cubes, implement camera, visualize in RViz              |
| 3    | Implement OpenCV color detection and decision node            |
| 4    | Build servo pusher and integrate ros2_control                 |
| 5    | Complete full pipeline and perform system testing             |
| 6    | Replace OpenCV with YOLO (optional), optimize and polish demo |

---

# 11. Future Extensions

This architecture can later be expanded into:

* Multi-class product sorting
* Pick-and-place robot
* Autonomous warehouse system
* AGV + conveyor integration
* AI quality inspection
* Reinforcement Learning for robot manipulation
* Digital Twin for smart factory applications

The modular ROS 2 node structure allows each subsystem (vision, planning, control, actuation) to evolve independently without major architectural changes.
