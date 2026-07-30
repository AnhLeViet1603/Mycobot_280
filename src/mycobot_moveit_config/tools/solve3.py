import math, random, rclpy, numpy as np
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_euler
rclpy.init()
mi=MoveItPy(node_name="s3"); rm=mi.get_robot_model(); psm=mi.get_planning_scene_monitor(); arm=mi.get_planning_component("arm")
J=["joint2_to_joint1","joint3_to_joint2","joint4_to_joint3","joint5_to_joint4","joint6_to_joint5","joint6output_to_joint6"]
READY=[0.0,-0.5,-1.2,-1.4,1.57,0.0]
BOUNDS=[(-2.9321,2.9321),(-2.4434,2.4434),(-2.6179,2.6179),(-2.6179,2.6179),(-2.7052,2.7925),(-3.14,3.14159)]
TABLE=0.40
OBJS=[(0.1543,0.1839,"box",(0.030,0.030,0.050),0.025),(0.0821,0.2255,"cyl",(0.015,0.050),0.025),
      (0.0000,0.2400,"box",(0.028,0.028,0.045),0.0225),(-0.0821,0.2255,"cyl",(0.016,0.045),0.0225),
      (-0.1543,0.1839,"box",(0.030,0.030,0.050),0.025)]
def coadd(i):
    x,y,kind,dims,hh=OBJS[i]; c=CollisionObject(); c.header.frame_id="world"; c.id=f"obj_{i}"; c.operation=CollisionObject.ADD
    pr=SolidPrimitive()
    if kind=="box": pr.type=SolidPrimitive.BOX; pr.dimensions=list(dims)
    else: pr.type=SolidPrimitive.CYLINDER; pr.dimensions=[dims[1],dims[0]]
    p=Pose(); p.position.x=x;p.position.y=y;p.position.z=TABLE+hh;p.orientation.w=1.0
    c.primitives=[pr]; c.primitive_poses=[p]; return c
def colliding(cfg):
    with psm.read_only() as s:
        r=RobotState(rm); r.set_joint_group_positions("arm",cfg); r.update(); return s.is_state_colliding(r,"arm",False)
def rs_of(c):
    r=RobotState(rm); r.set_joint_group_positions("arm",c); r.update(); return r
def plan(a,b):
    arm.set_start_state(robot_state=rs_of(a)); arm.set_goal_state(robot_state=rs_of(b)); return bool(arm.plan())
def ik(x,y,z,q,seed):
    p=Pose(); p.position.x=x;p.position.y=y;p.position.z=z
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q
    r=RobotState(rm); r.set_joint_group_positions("arm",seed); r.update()
    if r.set_from_ik("arm",p,"grasp_center",timeout=0.2): return [round(r.joint_positions[j],4) for j in J]
    return None
# scene: obj_2 and obj_4 present (neighbors), obj_3 removed
with psm.read_write() as s:
    s.apply_collision_object(coadd(0)); s.apply_collision_object(coadd(1)); s.apply_collision_object(coadd(2)); s.apply_collision_object(coadd(4)); s.current_state.update()
x,y,hh=-0.0821,0.2255,0.0225
found=None
for gz in [TABLE+hh, TABLE+hh+0.01, TABLE+hh+0.02]:
    for pitch in [0.0,0.15,-0.15,0.3,-0.3,0.5]:
        for yaw in [math.radians(a) for a in range(-180,181,15)]:
            q=quaternion_from_euler(math.pi,pitch,yaw)
            for attempt in range(6):
                seed=[random.uniform(lo,hi) for lo,hi in BOUNDS]
                c=ik(x,y,gz,q,seed)
                if c and not colliding(c) and plan(READY,c):
                    found=(gz,round(pitch,2),round(math.degrees(yaw)),c); break
            if found: break
        if found: break
    if found: break
if found:
    gz,pitch,yawd,grasp=found
    print(f"GRASP3 z={gz:.3f} pitch={pitch} yaw={yawd}deg: {grasp}",flush=True)
    # pregrasp cung nhanh
    q=quaternion_from_euler(math.pi,pitch,math.radians(yawd))
    def near(c): return max(abs(a-b) for a,b in zip(c,grasp))<1.0
    pre=None
    for dz in [0.08,0.07,0.06,0.05]:
        for attempt in range(150):
            seed=grasp if attempt==0 else [g+random.uniform(-0.3,0.3) for g in grasp]
            c=ik(x,y,gz+dz,q,seed)
            if c and near(c) and not colliding(c) and plan(READY,c) and plan(c,grasp):
                pre=c; break
        if pre: break
    print(f"PREGRASP3 (dz): {pre}",flush=True)
else:
    print("obj_3 STILL FAIL",flush=True)
import os; os._exit(0)
