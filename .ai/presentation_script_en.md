# Gazebo + ROS 2: Building a Simulated Sorting Robot

> **Duration:** ~10 minutes
> **Goal:** Introduce Gazebo and ROS 2, explain how they work together, run a live conveyor-sorting demo, and show how it scales to real-world robotics.

---

## Slide 1 — Title (30s)

**Talking points**
- Title: *"Gazebo + ROS 2 — A Simulated Conveyor Sorting Robot."*
- One-line pitch: "We built a robot that watches a conveyor belt, sees the color of each product, and pushes the defective ones off — entirely in simulation, with the same software you would run on real hardware."
- Set expectations: theory first (~3 min), live demo (~4 min), real-world applications and wrap-up (~3 min).

---

## Slide 2 — Agenda (30s)

**Talking points**
- What is Gazebo?
- What is ROS 2?
- How Gazebo and ROS 2 talk to each other.
- Live demo: color-based conveyor sorting.
- Real-world applications.
- Conclusion and where this goes next.

Keep this short — it is a signpost, not content.

---

## Slide 3 — What is Gazebo? (1 min)

**Talking points**
- Gazebo is an open-source **3D robot simulator**. It models physics, gravity, friction, collisions, and — crucially — **sensors**: cameras, LiDAR, IMU, plus actuators like motors and servos.
- Why simulate instead of testing on real hardware?
  - **Cost** — no expensive robot to break.
  - **Speed** — reset the world instantly, iterate in seconds.
  - **Safety** — a bug crashes a simulation, not a machine.
- The typical workflow:
  `Write code → Run in Gazebo → Verify behavior → Deploy to the real robot.`
- **In our demo, Gazebo provides:** the conveyor belt (a `ConveyorBelt` plugin that drives objects along at 0.3 m/s), an overhead camera looking straight down, the spawned cubes, and the physical pusher mechanism.

---

## Slide 4 — What is ROS 2? (1 min)

**Talking points**
- ROS 2 (Robot Operating System 2) is **not** an operating system — it is a **framework and middleware** for building robot software out of small, independent programs.
- Core building blocks:
  - **Nodes** — independent processes, each doing one job.
  - **Topics** — named streams that carry **messages** (publish/subscribe).
  - Plus services, actions, parameters, TF (transforms), and launch files.
- The key idea: **decoupling**. The camera node does not know or care who consumes its images. You can swap, add, or restart any node without touching the others.
- Our system is exactly this — a chain of four small nodes, each connected only by topics:
  `Camera → Vision → Decision → Controller → Robot.`

---

## Slide 5 — How Gazebo + ROS 2 Work Together (1 min)

**Talking points**
- Gazebo **simulates the world**; ROS 2 **runs the robot's brain**. They are separate processes — so how do they exchange data?
- The bridge: **`ros_gz_bridge`**. It translates Gazebo messages into ROS 2 messages and vice-versa.
- In our project the bridge config is tiny and explicit: it maps Gazebo's `/camera` image topic to ROS 2's `/camera/image_raw` (`GZ_TO_ROS`).
- The closed loop:
  1. **Gazebo → ROS 2:** the simulated camera produces images; the bridge hands them to ROS 2.
  2. **ROS 2 processes:** vision and decision nodes figure out what to do.
  3. **ROS 2 → Gazebo:** a command is sent back through `ros2_control` to move the pusher joint in the simulation.
- That round trip — sense, decide, actuate — is the heart of every robot, real or simulated.

---

## Slide 6 — The Demo: Problem & Architecture (1 min)

**Talking points**
- **Scenario:** a factory conveyor. Products (cubes) roll past a station. We keep the **good** ones and reject the **defective** ones.
  - Rule in our demo: **blue = keep**, everything else (red / green) = **reject and push off the belt.**
- **The pipeline — four ROS 2 nodes plus Gazebo:**

  | Stage | Node | What it does |
  |-------|------|--------------|
  | Spawn | `object_spawner` | Drops random-colored cubes at the head of the belt every ~4s (uses Gazebo's `create` service directly). |
  | See | `vision_node` | OpenCV **HSV color thresholding** on the camera image; finds the largest colored blob inside a small region of interest right under the camera; publishes a `DetectedObject`. |
  | Decide | `decision_node` | Applies the policy — accepted color is kept; anything else publishes a **reject** command (with debounce so one cube fires once). |
  | Act | `pusher_controller_node` | A small **state machine** (IDLE → WAIT → EXTEND → HOLD → RETRACT) that drives a prismatic servo pusher across the belt via `ros2_control`. |

- **Custom message:** we defined `DetectedObject` (`class_name` + `position`) — a clean, self-documenting interface between vision and decision.
- Everything launches with **one command** (`ros2 launch bringup system.launch.py`), with staggered startup so the world and pusher are ready before the first cube drops.

---

## Slide 7 — The Demo: Data Flow (Live / Video) (up to 3 min)

**Talking points — narrate the flow, not the code:**
1. A cube spawns at the head of the belt and the conveyor plugin carries it forward.
2. The overhead camera streams frames; the bridge delivers them to ROS 2 on `/camera/image_raw`.
3. `vision_node` classifies the cube's color and publishes `/detected_object`.
4. `decision_node` reads the color: **blue passes through untouched**; a red or green cube triggers `/reject_object`.
5. `pusher_controller_node` waits a short, tuned delay so the paddle meets the cube, then **extends the servo** across the belt.
6. The paddle sweeps the defective cube **off the edge** — it falls away. Good cubes continue to the end.

**Presenter notes**
- This is the highlight — spend the most time here. Point at the moving cube and the pusher, not at terminal logs.
- **Have a 30–45s backup video ready** in case Gazebo or ROS 2 is slow to start.
- If showing RViz (`rviz:=true`), you can display the camera view the robot actually "sees."

---

## Slide 8 — Gazebo in the Real World (1 min)

**Talking points**
- Simulation-first development is standard across the industry:
  - **Industrial & warehouse robots** (exactly our sorting scenario).
  - **Autonomous mobile robots** (AGVs / AMRs), **drones**, and **research robots**.
- What teams use it for:
  - Test and tune algorithms before touching expensive hardware.
  - Develop AI vision and perception with unlimited, repeatable data.
  - Validate motion and control safely.
- Our toy demo is the same pattern as a real quality-control line — just scaled down.

---

## Slide 9 — Conclusion (30s)

**Talking points**
- **Gazebo** simulates the environment and sensors; **ROS 2** is the modular brain that controls the robot; the **`ros_gz` bridge** ties them together.
- The payoff of this combination:
  - ✔ Cheaper development ✔ Faster iteration ✔ Safe testing ✔ A clean path to real hardware — the *same* ROS 2 nodes run on a real robot.
- We demonstrated a complete perception → decision → actuation loop in under 400 lines of node code across four independent packages.

---

## Slide 10 — Where This Goes Next (30s)

**Talking points**
- Our demo uses a camera, a conveyor, and a single servo. Natural extensions:
  - **YOLO / deep-learning detection** instead of HSV thresholds — recognize real objects, not just colors.
  - A **robot arm** to pick-and-place instead of a paddle.
  - **Mobile robots / AGVs** with **Navigation (Nav2) and SLAM**.
  - A full **Digital Twin** of a production line.
- Closing line: "The architecture doesn't change — we swap in smarter nodes. That is the power of ROS 2 plus Gazebo."

---

## Time Budget

| Section | Time |
|---------|------|
| Title + Agenda | 1 min |
| Gazebo | 1 min |
| ROS 2 | 1 min |
| Gazebo + ROS 2 | 1 min |
| Demo (architecture + live/video) | 4 min |
| Applications | 1 min |
| Conclusion + Next steps | 1 min |
| **Total** | **~10 min** |

## Presenter Reminders
- Don't dive into ROS 2 APIs or code line-by-line — explain the **flow**: camera → ROS 2 → control → Gazebo.
- Prefer **diagrams over text**; let the demo carry the message.
- The demo should take ~40% of the time — it is the memorable part.
- Keep a **backup video** in case the live sim is slow to launch.
