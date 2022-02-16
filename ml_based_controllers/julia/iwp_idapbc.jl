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

function load_idapbc_model(; kv=1, umax=1.5)
    weightdir = "/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia"
    weightfile = "model2.bson"
    BSON.@load joinpath(weightdir, weightfile) θ
    Md⁻¹ = PSDMatrix(2, ()->θ[1:4])
    _, re = Chain(
        Dense(2, 10, elu; bias=false),
        Dense(10, 10, elu; bias=false),
        Dense(10, 1, square; bias=false),
    ) |> Flux.destructure
    Vd = re(θ[5:end])
    P = IDAPBCProblem(2,M⁻¹,Md⁻¹,V,Vd,G,G⊥)
    function (x::AbstractVector)
        xbar = [
            rem2pi(x[1]-pi, RoundNearest)
            rem2pi(x[2], RoundNearest)
            x[3]
            x[4]
        ]
        u = controller(P, xbar, kv=kv)
        clamp(u, -umax, umax)
    end
end
function load_bayes_idapbc_model(; num_samples=0, kv=1, umax=1.5)
    # psμ = [31.81599845517258, 1.2449662349000192, -0.4429720022661034, 23.437419607082013, -2.7940005465094915, -0.13802583096399593, 0.6291068070182659, 0.7926600642950876, -0.1365413985013702, 0.050817252012090114, 0.04888089163561625, 0.19417523895265362, -0.096491413342068, -0.1192403878169369, 1.8029593380259277, -0.67375178405653, -0.21478740169920277, 0.543829691538897, -2.7182465238521805, 0.2547928617794956, 2.0910271827088183, -0.05510645904882095, -0.9788448387147795, 1.1209079402849231, 1.2562726607754247, -2.5256438458993418, 0.012521387299170373, -1.3449626173506206, -0.9237932773724693, -0.47067264955273286, -0.4312795305115486, -0.5488247035415885, 1.0727310634526028, -0.03358069021786072, -0.15390371217229382, -1.9181188776037188, 1.2136649883471229, -0.4150544284370016, -0.6807240711617653, -1.183131594601645, -0.9872535549754702, 1.7706712857317837, 0.04983172035069209, 2.170710883876701]
    # psμ = [31.035446145530003, 7.92297235649231, 0.9513960118541301, 30.990489648712924, 0.370436394714019, -0.07183599378752871, 0.08058652210763718, 0.24254854127556014, -0.9082394738527316, -0.019648974665634835, -1.2020819706581432, -0.06623617978344006, 0.1983095535425591, 0.03241303334697136, -0.21289332981319678, 0.13781321723809312, -0.018765950384975676, -0.004515077962848985, -0.0474892211588299, -0.048684315907155586, -0.022675400047259445, -0.002452329326416339, -0.08078236529032956, -0.10269796899371361, -0.24171858019186485, -0.7367573685666868, -0.25679894959053506, -0.5483947061280932, -0.3422143240573471, 0.37120182680890784, 0.4170436014134886, -0.45744632086867887, 0.0993608086033219, 0.1996203174388519, 0.09550673055558441, 0.03960278999687588, 0.41334224127478225, -0.6775026741838427, -0.3144946600570302, -0.020694801837649036, -0.29126169146190756, -0.13627580441822768, 0.7063124799579481, 0.4838517629633578, 0.03446457809055936, -0.3105565596798651, 0.12539097931585363, -0.13592507664212597, 0.05907439522324337, -0.22769834921727233, -0.044942130182178436, -0.5264265731204589, -0.4815574922583618, -0.5753957281071209, -0.5999660642411709, -0.3958299870457513, -0.4426538739890489, -0.7329915144158635, -0.264756300326867, -0.35260795843067044, -0.194082954821054, 0.05468775340553617, 0.3591746902506883, -0.1877446360493185, -0.03012944640575222, -0.4034251301424799, -0.03809584669003557, -0.14528883645979107, -0.15377177812151271, -0.5668118903419526, -0.27379126469402476, 0.155912897520785, -0.6124294021446947, 0.6246742774339318, 0.012139961807882968, -0.005880949941409778, -0.01464068370853437, -0.004903443375072603, 0.007530185288287717] 
    # psσ = randn(length(psμ))
    # weightdir = "/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia"
    weightdir = "/home/wankunsirichotiyakul/Downloads"
    weightfile = "idapbc_paramnoise_1.bson"
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
    weightdir = "/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia"
    weightfile = "neuralpbc_02.bson"
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
    # weightdir = "/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia"
    weightdir = "/home/wankunsirichotiyakul/Downloads"
    weightfile = "normalm3_with1_1_4.bson"
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
            rem2pi(q2, RoundNearest)
            q1dot
            q2dot
        ]
        effort = -dot(LQR, xbar)
    else
        effort = swingup_controller(x)
    end
    return clamp(effort, -1.5, 1.5)
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

    # policy = load_pbc_model(umax=0.25)
    # policy = load_idapbc_model(kv=0.001, umax=0.8)

    # policy = load_bayes_idapbc_model(num_samples=10, kv=0.015/10*0, umax=0.5)
    policy = load_bayes_pbc_model(num_samples=0, umax=0.25)

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
