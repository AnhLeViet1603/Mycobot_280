"""Mo phong dung mau runtime: batch-add 5 vat (1 rw); lan luot i=0..4: rm(i) (1 rw), plan cac
doan, KHONG re-add. Test thu tu tu nhien 0..4 + duong mang ra khay khi vat khac con."""
import rclpy
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
rclpy.init()
mi=MoveItPy(node_name="vrt"); rm=mi.get_robot_model(); psm=mi.get_planning_scene_monitor(); arm=mi.get_planning_component("arm")
TABLE=0.40
OBJS=[(0.1543,0.1839,"box",(0.030,0.030,0.050),0.025),(0.0821,0.2255,"cyl",(0.015,0.050),0.025),
      (0.0000,0.2400,"box",(0.028,0.028,0.045),0.0225),(-0.0821,0.2255,"cyl",(0.016,0.045),0.0225),
      (-0.1543,0.1839,"box",(0.030,0.030,0.050),0.025)]
def co(i,op):
    x,y,kind,dims,hh=OBJS[i]; c=CollisionObject(); c.header.frame_id="world"; c.id=f"obj_{i}"; c.operation=op
    if op==CollisionObject.ADD:
        pr=SolidPrimitive()
        if kind=="box": pr.type=SolidPrimitive.BOX; pr.dimensions=list(dims)
        else: pr.type=SolidPrimitive.CYLINDER; pr.dimensions=[dims[1],dims[0]]
        p=Pose(); p.position.x=x;p.position.y=y;p.position.z=TABLE+hh;p.orientation.w=1.0
        c.primitives=[pr]; c.primitive_poses=[p]
    return c
READY=[0.0,-0.5,-1.2,-1.4,1.57,0.0]; PLACE=[-0.0714,1.5998,0.8123,0.5351,1.3757,-0.1476]
PG=[[1.6982,-0.4546,-2.1149,-0.6258,-2.1035,0.0118],[1.8556,-0.3545,-2.0992,-0.8316,-2.5114,0.1173],
    [-1.4295,0.8435,1.5736,0.5994,0.8664,-0.0559],[-1.4001,0.5568,2.0431,0.5334,0.1707,-0.0014],
    [-1.7927,0.8046,1.3629,1.6066,-1.2857,-0.6244]]
GR=[[1.6982,-1.2543,-1.9338,-0.0072,-2.1035,0.0118],[1.8555,-1.1474,-2.0086,-0.1293,-2.5115,0.1173],
    [-1.4295,1.4761,1.279,0.2613,0.8664,-0.0559],[-1.4001,1.3194,1.8127,0.0012,0.1707,-0.0014],
    [-1.7927,1.3533,1.2261,1.1948,-1.2857,-0.6245]]
def rs_of(c):
    r=RobotState(rm); r.set_joint_group_positions("arm",c); r.update(); return r
def plan(a,b):
    arm.set_start_state(robot_state=rs_of(a)); arm.set_goal_state(robot_state=rs_of(b)); return bool(arm.plan())
with psm.read_write() as s:
    for i in range(5): s.apply_collision_object(co(i,CollisionObject.ADD))
    s.current_state.update()
print("scene: 5 objects added",flush=True)
allok=True
for i in range(5):
    with psm.read_write() as s:
        s.apply_collision_object(co(i,CollisionObject.REMOVE)); s.current_state.update()
    a=plan(READY,PG[i]); b=plan(PG[i],GR[i]); c=plan(GR[i],PG[i]); d=plan(PG[i],READY); e=plan(READY,PLACE); f=plan(PLACE,READY)
    ok=all([a,b,c,d,e,f]); allok&=ok
    print(f"obj_{i}: r->pg={a} pg->g={b} g->pg={c} pg->r={d} r->place={e} place->r={f}  {'OK' if ok else 'FAIL'}",flush=True)
print("ALL OK" if allok else "SOME FAIL",flush=True)
import os; os._exit(0)
