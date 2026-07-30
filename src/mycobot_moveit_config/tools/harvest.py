"""Thu hoach GRASP_CONFIGS + PLACE_CONFIG bang FK: sample config ngau nhien, tim cai co
grasp_center gan target (x,y) tai z gap, tool chia xuong (tilt<TILT_MAX), valid & plan-able
tu READY, chon cai IT NGHIENG NHAT. Khong ap dat huong -> dung huong arm thuc su dat duoc.

Dung khi doi layout (R / AZ / PLACE ben duoi) -> in ra GRASP_CONFIGS/PLACE_CONFIG de dan
vao pick_and_place.py va (x,y) tuong ung vao FIXED_OBJECTS trong object_spawner.

Chay (can param MoveIt, khong can Gazebo):
    python3 genparams.py            # sinh pnp_params.yaml
    python3 harvest.py --ros-args --params-file pnp_params.yaml
"""
import math, random, rclpy, numpy as np
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
rclpy.init()
mi = MoveItPy(node_name="harvest")
rm = mi.get_robot_model(); psm = mi.get_planning_scene_monitor()
arm = mi.get_planning_component("arm")
J = ["joint2_to_joint1","joint3_to_joint2","joint4_to_joint3","joint5_to_joint4","joint6_to_joint5","joint6output_to_joint6"]
READY=[0.0,-0.5,-1.2,-1.4,1.57,0.0]
BOUNDS=[(-2.9321,2.9321),(-2.4434,2.4434),(-2.6179,2.6179),(-2.6179,2.6179),(-2.7052,2.7925),(-3.14,3.14159)]
rs=RobotState(rm)
def fk(c):
    rs.set_joint_group_positions("arm",c); rs.update()
    return np.array(rs.get_global_link_transform("grasp_center"))
def valid(c):
    rs.set_joint_group_positions("arm",c); rs.update()
    with psm.read_only() as s: return s.is_state_valid(rs,"arm")
def rs_of(c):
    r=RobotState(rm); r.set_joint_group_positions("arm",c); r.update(); return r
def plan_ok(c):
    arm.set_start_state(robot_state=rs_of(READY)); arm.set_goal_state(robot_state=rs_of(c))
    return bool(arm.plan())

R=0.24
AZ=[50,70,90,110,130]
ZTGT=0.44; TILT_MAX=math.radians(55)
targets=[(round(R*math.cos(math.radians(a)),4), round(R*math.sin(math.radians(a)),4)) for a in AZ]
PLACE=(-0.20,0.10)   # khay dat, xa ve ben trai ngoai cung pick
print(f"# R={R} AZ={AZ}  PLACE={PLACE}", flush=True)
print("targets:", targets, flush=True)
alltgt=targets+[PLACE]

def tilt_of(c):
    T=fk(c); za=T[:3,2]
    return math.acos(max(-1,min(1,-za[2])))

# 1 pass sample lon, gom ung vien cho moi target (ke ca PLACE)
POOL={i:[] for i in range(len(alltgt))}
for _ in range(150000):
    c=[random.uniform(lo,hi) for lo,hi in BOUNDS]
    T=fk(c); x,y,z=T[:3,3]; za=T[:3,2]
    if y<=0.03 or not (0.42<=z<=0.47): continue
    tilt=math.acos(max(-1,min(1,-za[2])))
    if tilt>TILT_MAX: continue
    for i,(tx,ty) in enumerate(alltgt):
        if math.hypot(x-tx,y-ty)<0.025:
            POOL[i].append((tilt,[round(v,4) for v in c]))
CONF=[None]*len(alltgt); TLT=[None]*len(alltgt)
for i in range(len(alltgt)):
    cands=sorted(POOL[i], key=lambda t:t[0])   # uu tien tilt nho nhat (thang dung nhat)
    for tilt,c in cands[:60]:
        if valid(c) and plan_ok(c):
            CONF[i]=c; TLT[i]=round(math.degrees(tilt),1); break
    print(f"  target{i} {alltgt[i]}: {len(cands)} cands -> {'tilt '+str(TLT[i])+' deg' if CONF[i] else 'NONE'}", flush=True)
print("GRASP_CONFIGS = [")
for c in CONF[:len(targets)]: print(f"    {c},")
print("]")
print(f"PLACE_CONFIG = {CONF[-1]}", flush=True)
import os; os._exit(0)
