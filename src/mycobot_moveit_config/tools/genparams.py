"""Sinh file param MoveIt (giống pick_and_place.launch.py) để chạy MoveItPy standalone.
Ghi pnp_params.yaml cạnh script này -> dùng cho harvest.py."""
import os
import yaml
from moveit_configs_utils import MoveItConfigsBuilder

mc = (MoveItConfigsBuilder("mycobot_280", package_name="mycobot_moveit_config")
      .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
      .to_moveit_configs())
params = mc.to_dict()
params["use_sim_time"] = False
params["planning_pipelines"] = {"pipeline_names": ["ompl"]}
params["plan_request_params"] = {
    "planning_attempts": 1, "planning_pipeline": "ompl",
    "planner_id": "RRTConnectkConfigDefault",
    "max_velocity_scaling_factor": 0.3, "max_acceleration_scaling_factor": 0.3,
    "planning_time": 2.0,
}
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pnp_params.yaml")
with open(out, "w") as f:
    yaml.safe_dump({"/**": {"ros__parameters": params}}, f)
print("wrote", out)
