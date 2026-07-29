# Gamma.ai Slide Deck Prompt — Gazebo + ROS 2 Sorting Robot

## How to use
Paste the **Master Prompt** below into Gamma.ai's "Generate" box (paste text / outline mode). Then, if you want tighter control, feed the **per-slide prompts** as the outline. Style settings are at the bottom.

---

## Master Prompt (paste this whole block)

> Create a clean, modern **10-slide technical presentation** titled **"Gazebo + ROS 2 — A Simulated Conveyor Sorting Robot."**
> Audience: engineering students / robotics beginners. Tone: professional but approachable. Language: **English**.
> Visual style: dark, high-tech engineering look; blue/teal accent color; lots of **diagrams and flow arrows** instead of dense paragraphs; use icons for concepts (camera, gear, robot, conveyor). Keep max ~5 bullet points per slide, each short.
> The talk is 10 minutes with a live robot-simulation demo as the centerpiece. Emphasize the sense → decide → act data flow. Build the following 10 slides:

Then paste the 10 slide descriptions below as the outline.

---

## Per-Slide Prompts

### Slide 1 — Title
Title slide. Big title: **"Gazebo + ROS 2 — A Simulated Conveyor Sorting Robot."** Subtitle: *"Sense → Decide → Act: a full robotics pipeline in simulation."* Include a hero visual of a 3D conveyor belt with colored cubes and a robotic pusher. Add a small footer for presenter name / date.

### Slide 2 — Agenda
Title: **"Agenda."** A simple 6-item numbered list with icons: 1) What is Gazebo? 2) What is ROS 2? 3) How they work together 4) Live demo: color sorting 5) Real-world applications 6) Conclusion & next steps.

### Slide 3 — What is Gazebo?
Title: **"What is Gazebo?"** Define it as an open-source 3D robot simulator that models physics and sensors (camera, LiDAR, IMU, motors, servos). Show the workflow as a horizontal arrow diagram: **Write code → Run in Gazebo → Verify → Deploy to real robot.** Three benefit chips: **Cost, Speed, Safety.** Note our demo uses Gazebo for the conveyor, overhead camera, cubes, and pusher.

### Slide 4 — What is ROS 2?
Title: **"What is ROS 2?"** Key message: a **framework/middleware** for robot software — NOT an operating system. Show core concepts as labeled icons: **Nodes, Topics, Messages, Services, Parameters, TF.** Highlight the core idea: **modular, decoupled nodes connected by topics.** Small chain graphic: Camera → Vision → Decision → Controller → Robot.

### Slide 5 — Gazebo + ROS 2 Together
Title: **"How Gazebo and ROS 2 Work Together."** Center this on a **closed-loop diagram**: Gazebo (camera/sensors) → **ros_gz_bridge** → ROS 2 (processing) → command → **ros2_control** → back to Gazebo (move the pusher). Caption the loop as **Sense → Decide → Act.** Note the real bridge maps Gazebo `/camera` to ROS 2 `/camera/image_raw`.

### Slide 6 — Demo Problem & Architecture
Title: **"The Demo — Color-Based Conveyor Sorting."** State the rule prominently: **Blue = KEEP, Red/Green = REJECT (pushed off the belt).** Show a **4-stage pipeline diagram** with node names and icons:
1. **object_spawner** — drops random-colored cubes
2. **vision_node** — OpenCV HSV color detection → DetectedObject
3. **decision_node** — keep vs. reject policy → reject command
4. **pusher_controller_node** — servo state machine: IDLE → WAIT → EXTEND → HOLD → RETRACT.
Add a note: "One launch command starts the whole system."

### Slide 7 — Demo Live / Data Flow
Title: **"Live Demo — Follow the Cube."** A vertical numbered flow with arrows, meant to narrate over a live sim or video:
Cube spawns → conveyor carries it → camera streams frames → vision classifies color → decision: blue passes / red-green rejected → servo extends → defective cube pushed off the edge.
Leave a large empty area / placeholder framed as **"[ Live Simulation / Demo Video ]".** Add a small note: "Backup 30–45s video ready."

### Slide 8 — Real-World Applications
Title: **"Gazebo in the Real World."** Grid of application cards with icons: **Industrial & warehouse robots, AGV/AMR, Drones, Research robots.** Below, three use-case chips: **Test algorithms before real hardware, Develop AI vision, Validate motion safely.** Message: our demo is the same pattern as a real QC line, scaled down.

### Slide 9 — Conclusion
Title: **"Conclusion."** Two-column comparison: **Gazebo → simulates the world & sensors** | **ROS 2 → the modular robot brain**, joined by **ros_gz bridge.** Four benefit checkmarks: **Cheaper, Faster, Safer, Real-hardware ready.** One-line takeaway: "A complete perception → decision → actuation loop, built from small independent nodes."

### Slide 10 — Where This Goes Next
Title: **"Where This Goes Next."** Show extensions as an upgrade path from our current stack (Camera + Conveyor + Servo) to: **YOLO / deep-learning detection, Robot arm pick-and-place, Mobile robots / AGVs, Nav2 + SLAM, Digital Twin.** Closing statement banner: **"Same architecture — just swap in smarter nodes."** End with a Thank You / Q&A line.

---

## Global Style Settings (set in Gamma)
- **Theme:** dark, technical/engineering; accent = blue or teal.
- **Fonts:** clean sans-serif (e.g., Inter / Roboto).
- **Imagery:** 3D robotics renders, conveyor belts, colored cubes, flow diagrams with arrows; consistent icon set.
- **Density:** low text, high visual — max ~5 short bullets per slide; prefer diagrams.
- **Aspect ratio:** 16:9.
- **Consistency:** reuse the accent color for all flow arrows and highlighted keywords (KEEP / REJECT, node names).
