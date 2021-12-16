using Distributions, DistributionsAD
import LogExpFunctions

const I1    = 0.0455
const I2    = 0.00425
const m3    = 0.183*9.81
const M⁻¹   = inv(diagm([I1, I2]))
V(q)        = [ m3*(cos(q[1]) - 1.0) ]
const G     = [-1.0, 1.0]
const G⊥    = [1.0 1.0]
const Vdmax = 24.94412861168683
MLBasedESC.jacobian(::typeof(V), q) = [-m3*sin(q[1]), zero(eltype(q))]

const LQR   = [
                -7.409595362575457
                0.05000000000000429
                -1.1791663255097424
                -0.03665716263249201
            ]

mutable struct Bays_params{T, TIDAPBC}
    θv              ::Vector{T}
    θv_mean         ::Vector{T}
    θv_σ            ::Vector{T}
    wm              ::Vector{T}
    P               ::TIDAPBC
end

function Bays_params(T)

    loaded      = BSON.load("/home/wankunsirichotiyakul/Projects/rcl/mlpbc_ros/src/ml_based_controllers/julia/iwp_bw2.bson")
    θv          = loaded[:v]            #a vector of means and standard deviations
    V_size      = Int(length(θv)/2)

    θv_mean     = θv[1 : V_size]
    θv_σ        = θv[V_size + 1 : end]
    # wm          = [ 10.699861526489258, 0.0, 0.0, 6.656222343444824]
    wm          = Float32[32.14794, 0.0, 0.0, 21.512499]

    Mdinv       = PSDMatrix(2, () -> wm)
    Vd          = FastChain(
                    FastDense(2, 10, elu; bias=false),
                    FastDense(10, 10, elu; bias=false),
                    FastDense(10, 1, scaledShifthardσ; bias=false),
                    )

    P           = IDAPBCProblem(2,M⁻¹,Mdinv,V,Vd,G,G⊥)
    Bays_params{T, typeof(P)}(θv, θv_mean, θv_σ, wm, P)
end

shifted_hardσ(x) = Flux.hardσ(x - 3.0)
dshiftedhardσ(x) = abs(x-3) <= 3.0 ? 1/6 : 0.0  
MLBasedESC.derivative(:: typeof(shifted_hardσ)) = dshiftedhardσ

scaledShifthardσ(x) = Vdmax.*shifted_hardσ(x)
dscaledShiftedhardσ(x) = abs(x-3) <= 3.0 ? Vdmax/6.0 : 0.0  
MLBasedESC.derivative(:: typeof(scaledShifthardσ)) = dscaledShiftedhardσ

function map_controller(bp::Bays_params; kv=1, umax=1.5)

    p = vcat(bp.wm, bp.θv_mean)

    function u_map(x::AbstractVector)
        u       = controller(bp.P, x, p, kv=kv)
        clamp(u, -umax, umax)
    end
end

function getq(bp::Bays_params)

    return arraydist( map((μ, σ) -> Normal(μ, LogExpFunctions.softplus(σ)), bp.θv_mean, bp.θv_σ) )

end

function marginalized_controller(bp::Bays_params, sample_num, kv=1; umax=1.5)
    V_size = length(bp.θv_mean)

    function u_marge(x)
        posterior       = getq(bp)
        W               = rand(posterior, sample_num)
        input           = 0.0

        for i in 1:sample_num 
            ws          = W[:,i]
            w           = vcat(bp.wm, ws[1:V_size])
            input       += controller(bp.P, [rem2pi.(x[1:2], RoundNearest); x[3:end]], w, kv=kv)
        end

        return clamp(input/sample_num, -umax, umax)
    end
end
