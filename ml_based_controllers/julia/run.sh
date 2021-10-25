#!/bin/bash

# catkin_ws=/home/wankun/Projects/archive/2021/acrobot_hybrid
# julia_depot=$catkin_ws/.julia
# julia_ws=$catkin_ws/src/ml_based_controllers/julia
# julia_exec=$julia_ws/iwp_idapbc.jl

# JULIA_PROJECT=$julia_ws JULIA_DEPOT_PATH=$julia_depot \
#     julia $julia_exec

echo "Starting Julia..."
julia $JULIA_PROJECT/iwp_idapbc.jl
echo ""
echo "Julia exited. Publishing zero torque command."
rostopic pub -1 /theta2_controller/command std_msgs/Float64 "data: 0. "
