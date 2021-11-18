#!/bin/bash

echo "Starting Julia with depot path '$JULIA_DEPOT_PATH'..."
julia_script="$JULIA_PROJECT/iwp_idapbc.jl"
julia --sysimage $JULIA_PROJECT/MLBasedESCSysimagePrecompile.so $julia_script
echo ""
echo "Julia exited. Publishing zero torque command."
rostopic pub -1 /theta2_controller/command std_msgs/Float64 "data: 0. "

