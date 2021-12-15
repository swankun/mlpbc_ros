#!/usr/bin/env julia

using MLBasedESC
using LinearAlgebra

using RobotOS
@rosimport sensor_msgs.msg: JointState
@rosimport std_msgs.msg: Float64
rostypegen()
using .sensor_msgs.msg, .std_msgs.msg

import MLBasedESC.Flux
import MLBasedESC.Flux.NNlib
using MLBasedESC.Flux: Chain, Dense, elu

import MLBasedESC.DiffEqFlux
using MLBasedESC.DiffEqFlux: FastChain, FastDense

import BSON

#==============================================================================
Constants
==============================================================================#

const I1 = 0.0455
const I2 = 0.00425
const m3 = 0.183*9.81
const M⁻¹ = inv(diagm([I1, I2]))
V(q) = [ m3*(cos(q[1]) - 1.0) ]
MLBasedESC.jacobian(::typeof(V), q) = [-m3*sin(q[1]), zero(eltype(q))]
const G = [-1.0, 1.0]
const G⊥ = [1.0 1.0]
const LQR = [
    -7.409595362575457
    0.05000000000000429
    -1.1791663255097424
    -0.03665716263249201
]

#==============================================================================
IDAPBC
==============================================================================#

function load_idapbc_model()
    BSON.@load "/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia/model2.bson" θ
    Md⁻¹ = PSDMatrix(2, ()->θ[1:4])
    _, re = Chain(
        Dense(2, 10, elu; bias=false),
        Dense(10, 10, elu; bias=false),
        Dense(10, 1, square; bias=false),
    ) |> Flux.destructure
    Vd = re(θ[5:end])
    P = IDAPBCProblem(2,M⁻¹,Md⁻¹,V,Vd,G,G⊥)
end

#==============================================================================
ROS
==============================================================================#

function update_state(msg::JointState, state::Vector)
    state[1] = msg.position[1]
    state[2] = msg.position[2]
    state[3] = msg.velocity[1]
    state[4] = msg.velocity[2]
end

function compute_control(x::Vector, swingup_controller::Function)
    effort = 0.0
    q1, q2, q1dot, q2dot = x
    xbar = [
        rem2pi(q1-pi, RoundNearest)
        rem2pi(q2, RoundNearest)
        q1dot
        q2dot
    ]
    if (1-cos(q1-pi)) < (1-cosd(15)) && abs(q1dot) < 10.0
        xbar[2] = sin(q2)
        effort = -dot(LQR, xbar)        
    else
        effort = swingup_controller(xbar)
    end
    return clamp(effort, -1.25, 1.25)
end

function energy_shaping_controller(x::Vector)
    q1, q2, q1dot, q2dot = x
    w1, w2, w3, w4 = (-0.09, 0.05, 0.0011375, -0.005)
    q1bar = q1-pi
    q2bar = q2
    effort = w1*q1dot + w2*cos(q1)*q1dot + w3*q1dot^3 + w4*q2dot
    return clamp(effort, -1.0, 1.0)
end

function idapbc_controller(P::IDAPBCProblem; kv=1, umax=1.5)
    function (x::AbstractVector)
        u = controller(P, x, kv=kv)
        clamp(u, -umax, umax)
    end
end

function main()
    init_node("ida_pbc_controller")
    state = zeros(Float64,4)
    pub = Publisher{Float64Msg}("theta2_controller/command", queue_size=1)
    sub = Subscriber{JointState}("/joint_states", update_state, (state,), queue_size=1)
    prob = load_idapbc_model()
    idapbc = idapbc_controller(prob, kv=0.025, umax=0.85)
    # u = energy_shaping_controller
    loop_rate = Rate(1000.0)
    while !is_shutdown()
        header = std_msgs.msg.Header()
        header.stamp = RobotOS.now()
        effort = compute_control(state, idapbc)
        # effort = clamp(effort, -2.0, 2.0)
        gear_ratio = 1.0
        eta = 0.98
        k_tau = 0.230    # N-m/a
        current = effort / gear_ratio / k_tau / eta
        cmd = Float64Msg(current)
        publish(pub, cmd)
        rossleep(loop_rate)
    end
    safe_shutdown_hack()
end

function safe_shutdown_hack()
    run(`rostopic pub -1 /theta2_controller/command std_msgs/Float64 "data: 0. "`, wait=false);
end
Base.atexit(safe_shutdown_hack)

if !isinteractive()
    main()
end
