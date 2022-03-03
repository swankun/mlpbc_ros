#!/usr/bin/env julia

using MLBasedESC
using LinearAlgebra

using RobotOS
@rosimport sensor_msgs.msg: JointState
@rosimport std_msgs.msg: Float64
rostypegen()
using .sensor_msgs.msg, .std_msgs.msg

import MLBasedESC.DiffEqFlux
import MLBasedESC.Flux.NNlib
using MLBasedESC.Flux.NNlib: elu
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

function load_idapbc_model_1ring()
    kv = 0.5
    umax = 0.3
    weightdir = "/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia/models"
    BSON.@load joinpath(weightdir, "neuralidapbc_1ring.bson") idapbc
    θ = idapbc.θ
    Vd = FastChain(
        FastDense(2, 8, elu; bias=false),
        FastDense(8, 4, elu; bias=false),
        FastDense(4, 1, square; bias=false),
    )
    Md⁻¹ = PSDMatrix(2, ()->θ[1:4])
    P = IDAPBCProblem(2,M⁻¹,Md⁻¹,V,Vd,G,G⊥)
    function (x::AbstractVector)
        xbar = [
            rem2pi(x[1]-pi, RoundNearest)
            x[2]
            x[3]
            x[4]
        ]
        u = controller(P, xbar, θ, kv=kv)
        clamp(u, -umax, umax)
    end
end
function load_idapbc_model_4rings()
    kv = 0.00125
    umax = 0.3
    weightdir = "/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia/models"
    BSON.@load joinpath(weightdir, "neuralidapbc_01.bson") θ
    Vd = FastChain(
        FastDense(2, 10, elu; bias=false),
        FastDense(10, 10, elu; bias=false),
        FastDense(10, 1, square; bias=false),
    )
    Md⁻¹ = PSDMatrix(2, ()->θ[1:4])
    P = IDAPBCProblem(2,M⁻¹,Md⁻¹,V,Vd,G,G⊥)
    function (x::AbstractVector)
        xbar = [
            rem2pi(x[1]-pi, RoundNearest)
            rem2pi(x[2], RoundNearest)
            x[3]
            x[4]
        ]
        u = controller(P, xbar, θ, kv=kv)
        clamp(u, -umax, umax)
    end
end
function load_bayes_idapbc_model(; num_samples=0, kv=1, umax=1.5)
    @assert num_samples >= 0
    weightdir = "/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia/models"
    weightfile = "neuralidapbc_bayesian_00.bson"
    @show weightfile
    # BSON.@load joinpath(weightdir, weightfile) hα
    # hα = [33.48190609221907, 0.20383988261274175, 0.23223521487182777, 20.40343440477517, 1.4799063880614844, -0.7162648627823733, -1.8769237637949865, 0.5504990209991742, -0.8393574308059634, 0.0381406011767764, 0.10700686281650341, -0.09094831057289597, -0.10952011577928039, 0.05353570109029773, -0.7223701814958867, 0.2988509735225607, -0.8314697108617063, -0.26620544064847745, -1.9583168082207263, 0.10366813047468203, 0.5135296015665923, -1.015992296117198, -0.06277426535172527, -0.07916532631325124, -0.03072745346522029, -0.44583425806959653, -0.9533842193517232, 0.4100579565399244, -0.6556218480275986, -0.5152535003275573, -1.24758860693492, 0.17775833047728967, -1.109308866710824, -0.32357202839882354, -0.20085906839351728, -0.006862017041532976, -0.9436316884785275, -0.06899043382437162, 0.3876551156669185, 0.12256555995454774, 0.8033414789285865, -1.6665054934709884, 1.881556863561647, 2.012254527134933, -0.6091315442060726, -0.5508012531185122, 1.3559226388946681, -2.2419870185346094, -3.592094553586373, -3.2301745681849154, -4.266972492514613, -3.893274169169737, -3.3993895595423202, -5.662727934311575, -5.170268087058957, -5.291975969018179, -6.074960584359582, -5.210306588271884, -2.451873089233, -4.718975297054374, -4.355915603017048, -2.8350287234074236, -3.366362232767921, -3.628468602592392, -3.373424047259034, -2.9726276934453675, -3.7368741923408946, -4.033927981795854, -3.603911099044876, -3.8871874273916283, -3.2150133655030904, -3.9768186806798327, -4.8347827488179655, -2.9448407880219216, -3.138143279160789, -5.027077384268941, -3.5575737531518565, -3.9689011543225887, -3.9402260044189448, -3.6345484038847173, -3.087341771237993, -3.4040551843339535, -4.355883391025107, -3.439414881060524, -3.578658984570861, -3.220600303180046, -2.850566621398526, -3.6623076981665403]
    hα = [32.05927632680847, -0.20693091910328307, 0.5311078502912965, 20.770436035441627, -2.0161966796742434, -0.3120495602078026, 0.36503771669499485, -0.6274171711365386, 0.2710854509560949, -0.026196580929697802, 0.09076031611168556, -0.13490300387249968, 0.25221243493197887, -0.06465500802736025, -0.1431637484532038, -0.23885715311383043, -0.29372740334970476, 0.6955641899559116, -1.6013928634411794, 0.030287135732259197, -0.2755567386286588, -0.6047399304492512, -0.14296642143350155, 0.30389601637508185, 0.08108829942249983, 0.17779937455671077, 0.41896169670987615, -0.6872986094480841, -0.13965529403238075, 0.3705327826570474, -0.4410560303685716, -0.38134878209307127, 0.4238530449199445, -0.2179308280934077, 0.03893966665062813, -0.0843034296403608, -0.08262362424534707, -0.9481537346740734, -0.0813685147462573, 0.005528697612887698, -1.8027407551606396, -1.4064347781920985, 1.2078494025688908, -1.0832884402240348, -0.7251065339068226, 0.001948851114270459, 0.04090201095714992, -1.8991487132113751, -2.542655366481858, -5.632337639105942, -3.3708862664798227, -3.277594020226039, -5.509121048996126, -5.495788261224029, -4.35197708001611, -4.970240043459795, -5.072020580781121, -4.860785929524356, -4.449111193825719, -4.301073205350313, -4.361880213416855, -2.902679317122654, -2.6147441927506065, -6.764838526571721, -4.325156979125857, -2.721073454415007, -3.3820910437595066, -2.8931473256540534, -3.1573545378654524, -3.7669194416738896, -3.2562223833271173, -3.8818575001002884, -2.8564351543757, -2.3206851445178267, -4.19046318777606, -4.408533417210085, -4.24107138949338, -3.0152700407056465, -3.9239036766148674, -3.0993073195720244, -2.9503501139230117, -2.5504277909294735, -2.720484162014896, -3.5371918921848016, -2.935512462074275, -3.6481788175322283, -4.089138424789804, -3.2544188675475905]
    paramlength = length(hα) ÷ 2
    psμ = first(hα, paramlength)
    psσ = last(hα, paramlength)
    Md⁻¹ = PSDMatrix(2, ()->psμ[1:4])
    Vd = FastChain(
        FastDense(2, 5, elu; bias=false),
        FastDense(5, 5, elu; bias=false),
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
    @info "ROS node initialized. Loading models..."

    # policy = energy_shaping_controller

    # policy = load_pbc_model(umax=0.3)
    # policy = load_idapbc_model_4rings()
    policy = load_idapbc_model_1ring()

    # policy = load_bayes_idapbc_model(num_samples=10, kv=0.00125, umax=0.3)
    # policy = load_bayes_pbc_model(num_samples=10, umax=0.5)

    effort = compute_control(zeros(4), policy)
    @info "Model loaded. Spinning ROS..."
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
    @info "Publishing zero torque."
    run(`rostopic pub -1 /theta2_controller/command std_msgs/Float64 "data: 0. "`, wait=false);
end
Base.atexit(safe_shutdown_hack)

if !isinteractive()
    @info "Julia packages loaded. Starting main()"
    main()
end

