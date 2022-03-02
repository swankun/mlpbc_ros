#!/bin/bash

echo "Starting Julia with depot path '$JULIA_DEPOT_PATH'..."
julia_script="$JULIA_PROJECT/iwp_idapbc.jl"
julia --sysimage $JULIA_PROJECT/MLBasedESCSysImage.so $julia_script
# julia $julia_script

