"""Tinh chinh grasp + sinh pregrasp cho TIEP CAN THANG DUNG (buoc 2, sau harvest.py).
DUNG grasp config gan-top-down (SEED, lay tu harvest.py) lam seed huong da biet dat duoc.
Moi vat: IK snap grasp_center vao dung (x,y, grasp_z=mat ban+nua cao vat) giu huong seed; roi
pregrasp o tren (+DZ) CUNG NHANH IK (de ha/nang thang). Verify plan ready->pregrasp->grasp.
In PREGRASP_CONFIGS/GRASP_CONFIGS de dan vao pick_and_place.py. Cap nhat SEED/OBJS ben duoi
neu doi layout (SEED = output harvest.py, OBJS = (x,y,half_height) khop FIXED_OBJECTS).

Chay (khong can Gazebo):
    python3 genparams.py
    python3 harvest3.py --ros-args --params-file pnp_params.yaml
"""
import math, random, rclpy, numpy as np
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from tf_transformations import quaternion_from_matrix
rclpy.init()
mi=MoveItPy(node_name="harvest3"); rm=mi.get_robot_model(); psm=mi.get_planning_scene_monitor()
arm=mi.get_planning_component("arm")
J=["joint2_to_joint1","joint3_to_joint2","joint4_to_joint3","joint5_to_joint4","joint6_to_joint5","joint6output_to_joint6"]
READY=[0.0,-0.5,-1.2,-1.4,1.57,0.0]
BOUNDS=[(-2.9321,2.9321),(-2.4434,2.4434),(-2.6179,2.6179),(-2.6179,2.6179),(-2.7052,2.7925),(-3.14,3.14159)]
rs=RobotState(rm)
def fkT(c):
    rs.set_joint_group_positions("arm",c); rs.update(); return np.array(rs.get_global_link_transform("grasp_center"))
def valid(c):
    rs.set_joint_group_positions("arm",c); rs.update()
    with psm.read_only() as s: return s.is_state_valid(rs,"arm")
def rs_of(c):
    r=RobotState(rm); r.set_joint_group_positions("arm",c); r.update(); return r
def plan(a,b):
    arm.set_start_state(robot_state=rs_of(a)); arm.set_goal_state(robot_state=rs_of(b)); return bool(arm.plan())
def quatT(T):
    M=np.eye(4); M[:3,:3]=T[:3,:3]; return quaternion_from_matrix(M)
def ik_to(x,y,z,q,seed):
    p=Pose(); p.position.x=x;p.position.y=y;p.position.z=z
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q
    for a in range(30):
        s=seed if a==0 else [random.uniform(lo,hi) for lo,hi in BOUNDS]
        r=RobotState(rm); r.set_joint_group_positions("arm",s); r.update()
        if r.set_from_ik("arm",p,"grasp_center",timeout=0.2):
            c=[round(r.joint_positions[j],4) for j in J]
            if valid(c): return c
    return None

TABLE=0.40; DZ=0.08
SEED=[
 [1.5503,-0.9718,-2.1387,-0.0744,-2.2512,-0.0008],
 [2.0171,-0.7109,-2.1363,-0.4596,-2.3518,0.15],
 [-1.3763,1.2789,1.6197,0.1092,0.9192,-0.0669],
 [1.7338,-2.2824,0.8249,-1.435,1.9574,0.1275],
 [-1.8443,1.1645,1.5469,1.1844,-1.3255,-0.7505],
]
OBJS=[(0.1543,0.1839,0.025),(0.0821,0.2255,0.025),(0.0000,0.2400,0.0225),
      (-0.0821,0.2255,0.0225),(-0.1543,0.1839,0.025)]
GR=[None]*5; PG=[None]*5
for i,(x,y,hh) in enumerate(OBJS):
    q=quatT(fkT(SEED[i]))            # huong da biet dat duoc tai vat nay
    grasp=None; gz=None
    for z in [TABLE+hh, TABLE+hh+0.01, TABLE+hh+0.02, TABLE+hh+0.03]:  # tu tam vat len
        c=ik_to(x,y,z,q,SEED[i])
        if c and plan(READY,c):
            grasp=c; gz=z; break
    if grasp is None:
        print(f"obj_{i}: GRASP fail",flush=True); continue
    # PREGRASP: phai CUNG NHANH voi grasp (moi khop < 1.0 rad) de ha/nang THANG, khong lat.
    # Uu tien DZ lon; giam dan neu vat bien khong voi toi cung nhanh.
    def in_branch(c): return max(abs(a-b) for a,b in zip(c,grasp)) < 1.0
    pre=None
    for dz in [DZ,0.07,0.06,0.05,0.04]:
        from geometry_msgs.msg import Pose as _P
        pp=_P(); pp.position.x=x; pp.position.y=y; pp.position.z=gz+dz
        pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=q
        best=None
        for a in range(120):
            s=[g+random.uniform(-0.3,0.3) for g in grasp] if a else grasp
            r=RobotState(rm); r.set_joint_group_positions("arm",s); r.update()
            if r.set_from_ik("arm",pp,"grasp_center",timeout=0.2):
                c=[round(r.joint_positions[j],4) for j in J]
                if in_branch(c) and valid(c) and plan(READY,c) and plan(c,grasp):
                    d=max(abs(a2-b2) for a2,b2 in zip(c,grasp))
                    if best is None or d<best[0]: best=(d,c)
        if best: pre=best[1]; break
    GR[i]=grasp; PG[i]=pre
    T=fkT(grasp); off=math.hypot(T[0,3]-x,T[1,3]-y)
    print(f"obj_{i}: grasp z={gz:.3f} xy_off={off*1000:.1f}mm pregrasp={'OK' if pre else 'FAIL'}",flush=True)
print("PREGRASP_CONFIGS = [")
for c in PG: print(f"    {c},")
print("]")
print("GRASP_CONFIGS = [")
for c in GR: print(f"    {c},")
print("]",flush=True)
import os; os._exit(0)
