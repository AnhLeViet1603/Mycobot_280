"""Sinh PREGRASP+GRASP NE HANG XOM: nap ca 5 vat lam collision obstacle; voi moi vat i, GO obj_i
ra (de ha xuong gap), tim grasp (snap tam vat, huong gan-vertical) sao cho STATE KHONG COLLIDING
voi scene (khong cham hang xom) + plan-able; pregrasp cung nhanh o tren. Re-add obj_i. Worst-case
ALL neighbors present -> config an toan bat ke thu tu gap."""
import math, random, rclpy, numpy as np
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_matrix
rclpy.init()
mi=MoveItPy(node_name="h4"); rm=mi.get_robot_model(); psm=mi.get_planning_scene_monitor(); arm=mi.get_planning_component("arm")
J=["joint2_to_joint1","joint3_to_joint2","joint4_to_joint3","joint5_to_joint4","joint6_to_joint5","joint6output_to_joint6"]
READY=[0.0,-0.5,-1.2,-1.4,1.57,0.0]
BOUNDS=[(-2.9321,2.9321),(-2.4434,2.4434),(-2.6179,2.6179),(-2.6179,2.6179),(-2.7052,2.7925),(-3.14,3.14159)]
TABLE=0.40
OBJS=[(0.1543,0.1839,"box",(0.030,0.030,0.050),0.025),
      (0.0821,0.2255,"cyl",(0.015,0.050),0.025),
      (0.0000,0.2400,"box",(0.028,0.028,0.045),0.0225),
      (-0.0821,0.2255,"cyl",(0.016,0.045),0.0225),
      (-0.1543,0.1839,"box",(0.030,0.030,0.050),0.025)]
SEED=[[1.6982,-1.2543,-1.9338,-0.0072,-2.1035,0.0118],
      [1.8556,-1.1473,-2.0085,-0.1295,-2.5114,0.1173],
      [-1.4295,1.4762,1.2789,0.2613,0.8664,-0.0559],
      [1.6789,-2.3687,0.7627,-1.2478,1.9045,0.1688],
      [-1.7927,1.3533,1.2261,1.1948,-1.2857,-0.6245]]
def co(i,op):
    x,y,kind,dims,hh=OBJS[i]
    c=CollisionObject(); c.header.frame_id="world"; c.id=f"obj_{i}"; c.operation=op
    if op==CollisionObject.ADD:
        pr=SolidPrimitive()
        if kind=="box": pr.type=SolidPrimitive.BOX; pr.dimensions=list(dims)
        else: pr.type=SolidPrimitive.CYLINDER; pr.dimensions=[dims[1],dims[0]]
        p=Pose(); p.position.x=x;p.position.y=y;p.position.z=TABLE+hh; p.orientation.w=1.0
        c.primitives=[pr]; c.primitive_poses=[p]
    return c
def scene_add(i):
    with psm.read_write() as s: s.apply_collision_object(co(i,CollisionObject.ADD)); s.current_state.update()
def scene_rm(i):
    with psm.read_write() as s: s.apply_collision_object(co(i,CollisionObject.REMOVE)); s.current_state.update()
def fkT(c):
    r=RobotState(rm); r.set_joint_group_positions("arm",c); r.update(); return np.array(r.get_global_link_transform("grasp_center"))
def colliding(cfg):
    with psm.read_only() as s:
        r=RobotState(rm); r.set_joint_group_positions("arm",cfg); r.update()
        return s.is_state_colliding(r,"arm",False)
def rs_of(c):
    r=RobotState(rm); r.set_joint_group_positions("arm",c); r.update(); return r
def plan(a,b):
    arm.set_start_state(robot_state=rs_of(a)); arm.set_goal_state(robot_state=rs_of(b)); return bool(arm.plan())
def ik(x,y,z,q,seed):
    p=Pose(); p.position.x=x;p.position.y=y;p.position.z=z
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q
    r=RobotState(rm); r.set_joint_group_positions("arm",seed); r.update()
    if r.set_from_ik("arm",p,"grasp_center",timeout=0.2):
        return [round(r.joint_positions[j],4) for j in J]
    return None

for i in range(5): scene_add(i)
DZ=0.08
GR=[None]*5; PG=[None]*5
for i,(x,y,kind,dims,hh) in enumerate(OBJS):
    scene_rm(i)                       # go vat dang gap
    q0=quaternion_from_matrix(np.vstack([np.hstack([fkT(SEED[i])[:3,:3],[[0],[0],[0]]]),[0,0,0,1]]))
    gz=TABLE+hh
    grasp=None
    # thu nhieu seed de tim nhanh IK KHONG cham hang xom
    for attempt in range(80):
        seed=SEED[i] if attempt==0 else [g+random.uniform(-0.4,0.4) for g in SEED[i]]
        c=ik(x,y,gz,q0,seed)
        if c and not colliding(c) and plan(READY,c):
            grasp=c; break
    if grasp is None:
        print(f"obj_{i}: GRASP fail (neighbor-safe)",flush=True); scene_add(i); continue
    # pregrasp cung nhanh, tren cao, khong cham
    def near(c): return max(abs(a-b) for a,b in zip(c,grasp))<1.0
    pre=None
    for dz in [DZ,0.07,0.06,0.05]:
        for attempt in range(100):
            seed=grasp if attempt==0 else [g+random.uniform(-0.3,0.3) for g in grasp]
            c=ik(x,y,gz+dz,q0,seed)
            if c and near(c) and not colliding(c) and plan(READY,c) and plan(c,grasp):
                pre=c; break
        if pre: break
    GR[i]=grasp; PG[i]=pre
    T=fkT(grasp); off=math.hypot(T[0,3]-x,T[1,3]-y)*1000
    print(f"obj_{i}: grasp xy_off={off:.1f}mm neighbor-safe, pregrasp={'OK' if pre else 'FAIL'}",flush=True)
    scene_add(i)                      # tra vat vao (worst-case cho vat sau)
print("PREGRASP_CONFIGS = [")
for c in PG: print(f"    {c},")
print("]")
print("GRASP_CONFIGS = [")
for c in GR: print(f"    {c},")
print("]",flush=True)
import os; os._exit(0)
