using MLBasedESC
using LinearAlgebra

import MLBasedESC.DiffEqFlux
using MLBasedESC.DiffEqFlux: FastChain, FastDense
using MLBasedESC.Flux.NNlib: elu

using RobotOS
@rosimport sensor_msgs.msg: JointState
@rosimport std_msgs.msg: Float64
rostypegen()
using .sensor_msgs.msg, .std_msgs.msg

using Distributions
using LogExpFunctions: softplus

import BSON

const I1 = 0.0455
const I2 = 0.00425
const m3 = 0.183*9.81
const M⁻¹ = inv(diagm([I1, I2]))
V(q) = [ m3*(cos(q[1]) - 1.0) ]
MLBasedESC.jacobian(::typeof(V), q) = [-m3*sin(q[1]), zero(eltype(q))]
const G = [-1.0, 1.0]
const G⊥ = [1.0 1.0]

function build_idapbc_model()
    Md⁻¹ = PSDMatrix(2)
    Vd = FastChain(
        FastDense(2, 10, elu; bias=false),
        FastDense(10, 5, elu; bias=false),
        FastDense(5, 1, square; bias=false),
    )
    P = IDAPBCProblem(2,M⁻¹,Md⁻¹,V,Vd,G,G⊥)
    θ = randn(1000)
    function (x::AbstractVector)
        u = controller(P, x, θ, kv=0.1)
        clamp(u, -1.5, 1.5)
    end
end

function build_pbc_model(;umax=0.5)
    Hd = FastChain(
        FastDense(6, 10, elu, bias=true),
        FastDense(10, 5, elu, bias=true),
        FastDense(5, 1, bias=true)
    )
    pbc = NeuralPBC(6,Hd)
    ps = randn(1000)
    function (x::AbstractVector)
        xbar = [sincos(x[1]-pi)...; sincos(x[2])...; x[3]; x[4]]
        return clamp(pbc(xbar, ps) / 1, -umax, umax)
    end
end

init_node("dummynode")
uidapbc = build_idapbc_model()
x = [3.,0,0,0]
effort = uidapbc(x)
upbc = build_pbc_model()
effort = upbc(x)

