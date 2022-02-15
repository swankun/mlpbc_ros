using Pkg
Pkg.activate("/home/wankun/.julia/dev/MLBasedESC")
using MLBasedESC
import MLBasedESC.DiffEqFlux: FastChain, FastDense
import MLBasedESC.Flux.NNlib: elu

Pkg.activate()
using BSON
using DataFrames
using CSV
using CairoMakie
using LaTeXStrings

function getdf()
    file = CSV.File("20220121/05.csv", ignoreemptyrows=true)
    return DataFrame(file)
end

function theta1(raw::DataFrame, fig::Figure=Figure())
    pos = filter!(!ismissing, raw."/joint_states/theta1/position")
    t = filter!(!ismissing, raw."/joint_states/header/stamp")
    t .-= t[1]
    lines!(t, pos .- pi)
end
function theta1dot(raw::DataFrame, fig::Figure=Figure())
    vel = filter!(!ismissing, raw."/joint_states/theta1/velocity")
    t = filter!(!ismissing, raw."/joint_states/header/stamp")
    t .-= t[1]
    lines!(t, vel)
end

function theta2(raw::DataFrame, fig::Figure=Figure())
    pos = filter!(!ismissing, raw."/joint_states/theta2/position")
    t = filter!(!ismissing, raw."/joint_states/header/stamp")
    t .-= t[1]
    lines!(t, pos)
end
function theta2dot(raw::DataFrame, fig::Figure=Figure())
    vel = filter!(!ismissing, raw."/joint_states/theta2/velocity")
    t = filter!(!ismissing, raw."/joint_states/header/stamp")
    t .-= t[1]
    lines!(t, vel)
end
function effort(raw::DataFrame, fig::Figure=Figure())
    u = filter!(!ismissing, raw."/joint_states/theta2/effort")
    t = filter!(!ismissing, raw."/joint_states/header/stamp")
    t .-= t[1]
    u *= 0.23
    u = clamp.(u, -0.25, 0.25)
    lines!(t, u)
end


function loadpbc(pspath::String)
    Hd = FastChain(
        FastDense(6, 10, elu, bias=true),
        FastDense(10, 5, elu, bias=true),
        FastDense(5, 1, bias=true)
    )
    pbc = NeuralPBC(6,Hd)
    BSON.@load pspath ps
    return pbc, ps
end

wrap(x) = [sin(x[1]); cos(x[1]); sin(x[2]); cos(x[2]); x[3]; x[4]]

function hamil(raw::DataFrame, pbc::MLBasedESC.NeuralPBC, ps, fig::Figure=Figure())
    q1 = filter!(!ismissing, raw."/joint_states/theta1/position")
    q2 = filter!(!ismissing, raw."/joint_states/theta2/position")
    q1dot = filter!(!ismissing, raw."/joint_states/theta1/velocity")
    q2dot = filter!(!ismissing, raw."/joint_states/theta2/velocity")
    t = filter!(!ismissing, raw."/joint_states/header/stamp")
    t .-= t[1]
    traj = Float64.( mapreduce(permutedims, vcat, (q1.-pi, q2, q1dot, q2dot)) )
    trajwrap = mapreduce(wrap, hcat, eachcol(traj))
    Hd = map(x->pbc.Hd(x,ps)[1], eachcol(trajwrap))
    lines!(t, Hd)
end

function evolution(raw::DataFrame)
    fig = Figure()
    recipes = (theta1, theta2, theta1dot, theta2dot)
    labels = ("q1","q2","q1dot","q2dot")
    for (f, xstr, ij) = zip(recipes, labels, Iterators.product(1:2,1:2))
        Axis(fig[ij...], title=xstr)
        f(raw, fig)
    end
    Axis(fig[3,1], title="u")
    effort(raw, fig)
    Axis(fig[3,2], title="Hd")
    hamil(raw, loadpbc("../julia/neuralpbc_02.bson")..., fig)
    save("plots/out.png", fig)
end

function q1_u(raw::DataFrame)
    fig = Figure()
    Axis(fig[1,1], title="θ_1")
    theta1(raw, fig)
    Axis(fig[2,1], title="Control effort")
    effort(raw, fig)
    save("plots/out.png", fig)
end

function q1_q2(raw::DataFrame)
    fig = Figure()
    Axis(fig[1,1], title="theta_1")
    theta1(raw, fig)
    Axis(fig[2,1], title="theta_2")
    theta2(raw, fig)
    save("plots/out.png", fig)
end