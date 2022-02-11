using Distributions, DistributionsAD
import LogExpFunctions

mutable struct Bays_params{T, TPBC}
    θh_mean         ::Vector{T}
    θh_σ            ::Vector{T}
    θα_mean         ::Vector{T}
    θα_σ            ::Vector{T}
    pbc             ::TPBC
end

function Bays_params(T)

    # weights     = "/home/wankunsirichotiyakul/Downloads/normalm3_with1_1.bson"
    weights     = "/home/wankunsirichotiyakul/Downloads/nominal_neuralHd_4.bson"
    loaded      = BSON.load(weights)
    θhα         = loaded[:hα]            #a vector of means and standard deviations
    hα_size     = Int(length(θhα)/2)
    
    Nα          = 6
    Ns          = 0         #Note: s is not learned. 

    θh_mean     = θhα[1 : hα_size-Nα]
    θα_mean     = θhα[hα_size-Nα+1 : hα_size]
    θh_σ        = θhα[hα_size + 1 : 2hα_size - Nα]
    θα_σ        = θhα[2hα_size - Nα + 1 : 2hα_size]
    display(θh_mean[1:5])
    println("")
    Hd          = FastChain(
                    FastDense(6, 3, elu; bias=true),
                    FastDense(3, 3, elu; bias=true),
                    FastDense(3, 1; bias=true),
                )

    pbc         = MLBasedESC.NeuralPBC(Nα, Hd)
    Bays_params{T, typeof(pbc)}(θh_mean, θh_σ, θα_mean, θα_σ, pbc)
end

function getq(bp::Bays_params)
    m = vcat(bp.θh_mean, bp.θα_mean)
    s = LogExpFunctions.softplus.(vcat(bp.θh_σ, bp.θα_σ))
    # return arraydist(map(Normal, m, s))
    return MvNormal(m,s)
end

function map_controller(bp::Bays_params)

    p = vcat(bp.θh_mean, bp.θα_mean)

    function u_map(x::AbstractVector)
        sq1, cq1 = sincos(x[1]-pi)
        sq2, cq2 = sincos(x[2])
        xbar = [cq1, sq1, cq2, sq2, x[3], x[4]]
        u  = bp.pbc(xbar, p)
    end
end

function marginalized_controller(bp::Bays_params, sample_num)
    ps = rand(getq(bp))
    function u_marge(x)
        sq1, cq1 = sincos(x[1]-pi)
        sq2, cq2 = sincos(x[2])
        xbar = [cq1, sq1, cq2, sq2, x[3], x[4]]
        effort = bp.pbc(xbar, ps)
        for _ in 2:sample_num
            ps = rand(getq(bp))       
            effort  += bp.pbc(xbar, ps)
        end
        return effort/sample_num
    end
end
