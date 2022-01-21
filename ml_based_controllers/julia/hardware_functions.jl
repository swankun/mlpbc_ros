using Distributions, DistributionsAD
import LogExpFunctions

const I1 = 0.0455
const I2 = 0.00425
const m3 = 0.183*9.81
const b1 = 1/1000
const b2 = 5/1000
const M⁻¹ = inv(diagm([I1, I2]))
const G = [-1.0, 1.0]
const G⊥ = [1.0 1.0]

const LQR   = [
                -7.409595362575457
                0.05000000000000429
                -1.1791663255097424
                -0.03665716263249201
            ]

mutable struct Bays_params{T, TPBC}
    θh_mean         ::Vector{T}
    θh_σ            ::Vector{T}
    θα_mean         ::Vector{T}
    θα_σ            ::Vector{T}
    pbc             ::TPBC
end

function Bays_params(T)

    loaded      = BSON.load("/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia/bays_ode_hardware.bson")
    θhα         = loaded[:hα]            #a vector of means and standard deviations
    hα_size     = Int(length(θhα)/2)

    Nα          = 6
    Ns          = 0         #Note: s is not learned. 

    θh_mean     = θhα[1 : hα_size-Nα]
    θα_mean     = θhα[hα_size-Nα+1 : hα_size]
    θh_σ        = θhα[hα_size + 1 : 2hα_size - Nα]
    θα_σ        = θhα[2hα_size - Nα + 1 : 2hα_size]

    Hd          = FastChain(
                    FastDense(6, 10, elu; bias=true),
                    FastDense(10, 5, elu; bias=true),
                    FastDense(5, 1 ; bias=true),
                    )

    pbc         = MLBasedESC.NeuralPBC(Nα, Hd)
    Bays_params{T, typeof(pbc)}(θh_mean, θh_σ, θα_mean, θα_σ, pbc)
end

function getq(bp::Bays_params)

    return arraydist( map((μ, σ) -> Normal(μ, LogExpFunctions.softplus(σ)), vcat(bp.θh_mean, bp.θα_mean), vcat(bp.θh_σ, bp.θα_σ) ))

end

function map_controller(bp::Bays_params)

    p = vcat(bp.θh_mean, bp.θα_mean)

    function u_map(x::AbstractVector)
        xbar = [sin(x[1]), cos(x[1]), sin(x[2]), cos(x[2]), x[3], x[4]]
        u  = 3bp.pbc(xbar, p)/2
    end
end

function marginalized_controller(bp::Bays_params, sample_num)

    function u_marge(x)
        xbar = [sin(x[1]), cos(x[1]), sin(x[2]), cos(x[2]), x[3], x[4]]
        effort = 0.0 
            for _ in 1:sample_num
                ps = rand(getq(bp))       
                effort  += bp.pbc(xbar, ps)
            end
        return effort/sample_num
    end
end
