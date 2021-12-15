using MLBasedESC
using LinearAlgebra

import MLBasedESC.Flux
import MLBasedESC.Flux.NNlib
using MLBasedESC.Flux: Chain, Dense, elu

import MLBasedESC.DiffEqFlux
using MLBasedESC.DiffEqFlux: FastChain, FastDense

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
    Vd = Chain(
        Dense(2, 10, elu; bias=false),
        Dense(10, 10, elu; bias=false),
        Dense(10, 1, square; bias=false),
    )
    P = IDAPBCProblem(2,M⁻¹,Md⁻¹,V,Vd,G,G⊥)
end

function idapbc_controller(P::IDAPBCProblem; kv=1)
    function (x::AbstractVector)
        u = controller(P, x, kv=kv)
        clamp(u, -1.5, 1.5)
    end
end

prob = build_idapbc_model()
u = idapbc_controller(prob, kv=0.26)
x = [3.,0,0,0]
effort = u(x)
