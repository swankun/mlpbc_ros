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

using Distributions
using LogExpFunctions: softplus

import BSON

#==============================================================================
Constants
==============================================================================#

const I1 = 0.03401
const I2 = 0.001405
const m3 = 0.1162*9.81
const M⁻¹ = inv(diagm([I1, I2]))
V(q) = [ m3*(cos(q[1]) - 1.0) ]
MLBasedESC.jacobian(::typeof(V), q) = [-m3*sin(q[1]), zero(eltype(q))]
const G = [-1.0, 1.0]
const G⊥ = [1.0 1.0]
# const LQR = [
#     -7.409595362575457
#     0.05000000000000429
#     -1.1791663255097424
#     0.03665716263249201
# ]
# const LQR = vec([-5.116704087488273 -0.03535533905932452 -0.810719867331168 -0.012768437378146015])
# const LQR = [-3.4802710899961165, -0.00707106781188502, -0.599496113258823, -0.0049664133606209405]
const LQR = [-3.8150228674227527, -0.010000000000018733, -0.6570687716257183, 5*-0.0065154336851863896]   # 1 ring; b1,b2 /= 10

#==============================================================================
IDAPBC
==============================================================================#

function load_idapbc_model(; kv=1, umax=1.5)
    weightdir = "/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia/models"
    weightfile = "neuralidapbc_1ring.bson"
    @show weightfile
    BSON.@load joinpath(weightdir, weightfile) idapbc
    θ = idapbc.θ
    Md⁻¹ = PSDMatrix(2, ()->θ[1:4])
    _, re = Chain(
        Dense(2, 8, elu; bias=false),
        Dense(8, 4, elu; bias=false),
        Dense(4, 1, square; bias=false),
    ) |> Flux.destructure
    Vd = re(θ[5:end])
    P = IDAPBCProblem(2,M⁻¹,Md⁻¹,V,Vd,G,G⊥)
    function (x::AbstractVector)
        xbar = [
            rem2pi(x[1]-pi, RoundNearest)
            x[2]
            x[3]
            x[4]
        ]
        u = controller(P, xbar, kv=kv)
        clamp(u, -umax, umax)
    end
end
function load_bayes_idapbc_model(; num_samples=0, kv=1, umax=1.5)
    @assert num_samples >= 0
    weightdir = "/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia/models"
    weightfile = "neuralidapbc_bayesian_00.bson"
    @show weightfile
    BSON.@load joinpath(weightdir, weightfile) hα
    paramlength = length(hα) ÷ 2
    psμ = first(hα, paramlength)
    psσ = last(hα, paramlength)
    Md⁻¹ = PSDMatrix(2, ()->psμ[1:4])
    Vd = FastChain(
        FastDense(2, 10, elu; bias=false),
        FastDense(10, 5, elu; bias=false),
        FastDense(5, 1, square; bias=false),
    )
    P = IDAPBCProblem(2,M⁻¹,Md⁻¹,V,Vd,G,G⊥)
    if iszero(num_samples)
        function (x::AbstractVector)
            xbar = [
                rem2pi(x[1]-pi, RoundNearest)
                rem2pi(x[2], RoundNearest)
                x[3]
                x[4]
            ]
            u = controller(P, xbar, psμ, kv=kv)
            clamp(u, -umax, umax)
        end
    else
        Q = MvNormal(psμ, softplus.(psσ))
        function (x::AbstractVector)
            xbar = [
                rem2pi(x[1]-pi, RoundNearest)
                rem2pi(x[2], RoundNearest)
                x[3]
                x[4]
            ]
            u = controller(P, xbar, rand(Q), kv=kv)
            for _ = 2:num_samples
                u += controller(P, xbar, rand(Q), kv=kv)
            end
            clamp(u / num_samples, -umax, umax)
        end
    end
end

#==============================================================================
PBC
==============================================================================#

function load_pbc_model(;umax=0.5)
    weightdir = "/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia/models"
    weightfile = "neuralpbc_02.bson"
    @show weightfile
    BSON.@load joinpath(weightdir, weightfile) ps
    Hd = FastChain(
        FastDense(6, 10, elu, bias=true),
        FastDense(10, 5, elu, bias=true),
        FastDense(5, 1, bias=true)
    )
    pbc = NeuralPBC(6,Hd)
    function (x::AbstractVector)
        xbar = [sincos(x[1]-pi)...; sincos(x[2])...; x[3]; x[4]]
        return clamp(pbc(xbar, ps) / 1, -umax, umax)
    end
end
function load_bayes_pbc_model(;num_samples::Int=0, umax=0.5)
    @assert num_samples >= 0
    weightdir = "/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia/models"
    weightfile = "neuralpbc_bayesian_00.bson"
    @show weightfile
    BSON.@load joinpath(weightdir, weightfile) hα
    Hd = FastChain(
        FastDense(6, 3, elu, bias=true),
        FastDense(3, 3, elu, bias=true),
        FastDense(3, 1, bias=true)
    )
    paramlength = length(hα) ÷ 2
    psμ = first(hα, paramlength)
    psσ = last(hα, paramlength)
    pbc = NeuralPBC(6,Hd)
    if iszero(num_samples)
        function (x::AbstractVector) 
            sq1, cq1 = sincos(x[1]-pi)
            sq2, cq2 = sincos(x[2])
            xbar = [cq1, sq1, cq2, sq2, x[3], x[4]]
            return clamp(pbc(xbar, psμ), -umax, umax)
        end
    else
        Q = MvNormal(psμ, softplus.(psσ))
        function (x::AbstractVector)
            sq1, cq1 = sincos(x[1]-pi)
            sq2, cq2 = sincos(x[2])
            xbar = [cq1, sq1, cq2, sq2, x[3], x[4]]
            effort = pbc(xbar, rand(Q))
            for _ in 2:num_samples
                effort  += pbc(xbar, rand(Q))
            end
            return clamp(effort/num_samples, -umax, umax)
        end
    end
end


#==============================================================================
Controllers
==============================================================================#

function compute_control(x::Vector, swingup_controller::Function)
    effort = 0.0
    q1, q2, q1dot, q2dot = x
    if (1-cos(q1-pi)) < (1-cosd(30)) && abs(q1dot) < 5
        xbar = [
            rem2pi(q1-pi, RoundNearest)
            rem(q2, 8*2pi, RoundNearest)
            q1dot
            q2dot
        ]
        effort = -dot(LQR, xbar)
    else
        effort = swingup_controller(x)
    end
    return clamp(effort, -1.0, 1.0)
end
function spong_linear_controller(x::Vector)
    q1, q2, q1dot, q2dot = x
    a = m3/I1
    bp = 1/I1
    br = 1/I2
    ω0 = 3/2*sqrt(a)
    ζ = 1/sqrt(2)
    α = 1/5
    kpp = -1/bp*((1+2*a*ζ)*ω0^2 + a)
    kdp = -1/a/bp*((a+2*ζ)*ω0*a + a*ω0^3)
    kdr = -1/a/br*α*ω0^3
    effort = -kpp*(q1-pi) - kdp*q1dot - kdr*q2dot
end
function energy_shaping_controller(x::Vector)
    q1, q2, q1dot, q2dot = x
    w1, w2, w3, w4 = (-0.09, 0.05, 0.0011375, -0.005)
    q1bar = q1-pi
    q2bar = q2
    effort = w1*q1dot + w2*cos(q1)*q1dot + w3*q1dot^3 + w4*q2dot
    return clamp(effort, -1.0, 1.0)
end

#==============================================================================
Main loop
==============================================================================#

function update_state!(msg::JointState, state::Vector)
    state[1] = msg.position[1]
    state[2] = msg.position[2]
    state[3] = msg.velocity[1]
    state[4] = msg.velocity[2]
end

function main()
    init_node("ida_pbc_controller")
    state = zeros(Float64,4)
    pub = Publisher{Float64Msg}("theta2_controller/command", queue_size=1)
    sub = Subscriber{JointState}("/joint_states", update_state!, (state,), queue_size=1)

    # policy = energy_shaping_controller

    policy = load_pbc_model(umax=0.3)
    # policy = load_idapbc_model(kv=0.5, umax=0.3)

    # policy = load_bayes_idapbc_model(num_samples=10, kv=0.015/15*5, umax=0.5)
    # policy = load_bayes_pbc_model(num_samples=10, umax=0.5)

    loop_rate = Rate(800.0)
    while !is_shutdown()
        header = std_msgs.msg.Header()
        header.stamp = RobotOS.now()
        effort = compute_control(state, policy)
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
