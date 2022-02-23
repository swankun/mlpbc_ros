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
using Statistics

function default_plot_theme()
    majorfontsize = 28*1.5
    minorfontsize = 24*1.5
    T = Theme(
        Axis = (
            xlabelfont="Latin Modern Roman",
            ylabelfont="Latin Modern Roman",
            xticklabelfont="Latin Modern Roman",
            yticklabelfont="Latin Modern Roman",
            titlefont="Latin Modern Roman",
            xlabelsize=minorfontsize,
            ylabelsize=minorfontsize,
            xticklabelsize=minorfontsize,
            yticklabelsize=minorfontsize,
            titlesize=majorfontsize,
            topspinevisible = false,
            rightspinevisible = false,
            xgridvisible = false,
            ygridvisible = false,
        ),
        Lines = (
            linewidth = 2.5,
        ),
        Legend = (
            labelfont = "Latin Modern Roman",
            labelsize = minorfontsize
        )
    )
    set_theme!(T)
end

function getdf(filename="20220217/idapbc_0rings_00.csv")
    file = CSV.File(filename, ignoreemptyrows=true)
    return DataFrame(file)
end

Base.iszero(tf::Tuple) = false
function trimafter!(tf::Real, t, x...)
    i0 = findfirst(τ->τ≥tf, t)
    i1 = lastindex(t)
    deleteat!(t, i0:i1)
    foreach(x) do y
        deleteat!(y, i0:i1)
    end
end
function trimafter!(tspan::Tuple, t, x...)
    t0, tf = tspan
    i0 = findlast(τ->τ≤t0, t)
    i1 = findfirst(τ->τ≥tf, t)
    i2 = lastindex(t)
    deleteat!(t, vcat(1:i0, i1:i2))
    t .-= t[1]
    foreach(x) do y
        deleteat!(y, vcat(1:i0, i1:i2))
    end
end

function theta1(raw::DataFrame, fig::Figure=Figure(); tf=0, kwargs...)
    pos = filter(!ismissing, raw."/joint_states/theta1/position")
    t = filter(!ismissing, raw."/joint_states/header/stamp")
    t .-= t[1]
    !iszero(tf) && trimafter!(tf, t, pos)
    lines!(t, pos .- pi; kwargs...)
end
function theta1dot(raw::DataFrame, fig::Figure=Figure(); tf=0, kwargs...)
    vel = filter(!ismissing, raw."/joint_states/theta1/velocity")
    t = filter(!ismissing, raw."/joint_states/header/stamp")
    t .-= t[1]
    !iszero(tf) && trimafter!(tf, t, vel)
    lines!(t, vel; kwargs...)
end

function theta2(raw::DataFrame, fig::Figure=Figure(); tf=0, kwargs...)
    pos = filter(!ismissing, raw."/joint_states/theta2/position")
    t = filter(!ismissing, raw."/joint_states/header/stamp")
    t .-= t[1]
    !iszero(tf) && trimafter!(tf, t, pos)
    lines!(t, pos; kwargs...)
end
function theta2dot(raw::DataFrame, fig::Figure=Figure(); tf=0, kwargs...)
    vel = filter(!ismissing, raw."/joint_states/theta2/velocity")
    t = filter(!ismissing, raw."/joint_states/header/stamp")
    t .-= t[1]
    !iszero(tf) && trimafter!(tf, t, vel)
    lines!(t, vel; kwargs...)
end
function effort(raw::DataFrame, fig::Figure=Figure(); tf=0, kwargs...)
    u = filter(!ismissing, raw."/joint_states/theta2/effort")
    t = filter(!ismissing, raw."/joint_states/header/stamp")
    t .-= t[1]
    !iszero(tf) && trimafter!(tf, t, u)
    u *= 0.23
    u = clamp.(u, -0.25, 0.25)
    lines!(t, u; kwargs...)
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

function hamil(raw::DataFrame, pbc::MLBasedESC.NeuralPBC, ps, fig::Figure=Figure(); tf=0, kwargs...)
    q1 = filter(!ismissing, raw."/joint_states/theta1/position")
    q2 = filter(!ismissing, raw."/joint_states/theta2/position")
    q1dot = filter(!ismissing, raw."/joint_states/theta1/velocity")
    q2dot = filter(!ismissing, raw."/joint_states/theta2/velocity")
    t = filter(!ismissing, raw."/joint_states/header/stamp")
    t .-= t[1]
    traj = Float64.( mapreduce(permutedims, vcat, (q1.-pi, q2, q1dot, q2dot)) )
    trajwrap = mapreduce(wrap, hcat, eachcol(traj))
    Hd = map(x->pbc.Hd(x,ps)[1], eachcol(trajwrap))
    !iszero(tf) && trimafter!(tf, t, Hd)
    lines!(t, Hd; kwargs...)
end


function evolution(raw::DataFrame; tf=0)
    fig = Figure()
    recipes = (theta1, theta2, theta1dot, theta2dot)
    labels = ("q1","q2","q1dot","q2dot")
    for (f, xstr, ij) = zip(recipes, labels, Iterators.product(1:2,1:2))
        Axis(fig[ij...], title=xstr)
        f(raw, fig)
    end
    Axis(fig[3,1], title="u")
    effort(raw, fig, tf=tf)
    Axis(fig[3,2], title="Hd")
    hamil(raw, loadpbc("../julia/neuralpbc_02.bson")..., fig, tf=tf)
    save("plots/out.png", fig)
end

function q1_u(raw::DataFrame; tf=0)
    fig = Figure(resolution=(1200,600))
    Axis(fig[1,1], title=L"Pendulum angle $\theta_{1}$")
    theta1(raw, fig, tf=tf, color=:black)
    Axis(fig[1,2], title=L"Control effort $u$")
    effort(raw, fig, tf=tf, color=:black)
    save("plots/out.png", fig)
end

function q1_q2(raw::DataFrame; tf=0)
    fig = Figure(resolution=(1200,400))
    Axis(fig[1,1], title=L"Pendulum angle $\theta_{1}$")
    theta1(raw, fig, tf=tf, color=:black)
    Axis(fig[1,2], title=L"Rotor angle $\theta_{2}$")
    theta2(raw, fig, tf=tf, color=:black)
    save("plots/out.png", fig)
end

function hamilq(fig::Figure=Figure(); dim=1, kwargs...)
    pbc, ps = loadpbc("../julia/neuralpbc_02.bson")
    q = range(-pi, pi, length=101)
    state(q) = begin
        y = zeros(4)
        setindex!(y, q, dim)
        return y
    end
    f(q) = pbc.Hd(wrap(state(q)), ps)[1]
    Axis(fig[1,1], title=L"H_d")
    lines!(q, f; kwargs...)
    save("plots/out.png", fig)
end

function idapbc_disc_removal()
    fig = Figure(resolution=(800,1600))
    # files = readdir("20220217")
    files = [
        "bidapbc_0rings_00.csv", 
        "bidapbc_0rings_01.csv", 
        "bidapbc_2rings_00.csv", 
        "idapbc_0rings_00.csv", 
        "idapbc_0rings_01.csv", 
        "idapbc_2rings_00.csv"
    ]
    tspan = [
        (5.1, 5.1+6.3),#(5.1,9.8),
        (7.25, 7.25+6.3),#(7.25,10.7),
        (3.8, 3.8+6.3),#(3.8,9.25),
        (8.9, 8.9+5.0),#(8.9,11.5),
        (5.4, 5.4+6.3),#(5.4,11.7),
        (3.2, 3.2+6.3)#(3.2, 4.9), #(3,4.9),
    ]
    # tspan = fill((0,13), 6)
    perf = zeros(6)
    for (index, file) in enumerate(files)
        df = getdf(joinpath("20220217", file))
        perf[index] = performance_metric(df, tspan[index])
        Axis(fig[index,1], title=file)
        theta1(df, fig, tf=tspan[index], color=:black)
        Axis(fig[index,2], title=file)
        theta1dot(df, fig, tf=tspan[index], color=:black)
    end
    @show perf
    save("plots/out.png", fig)

    perf_avg = [mean(perf[1:2]); perf[3]; mean(perf[4:5]); perf[6]]
    @show perf_avg
    fig = Figure(resolution=(800,400))
    colors = Makie.wong_colors()
    ax = Axis(fig[1,1], xticks = (1:2, ["zero disc", "two discs"]),
        title = "Dodged bars with legend")
    tbl = (x = [1, 2, 1, 2],
       height = perf_avg,
       grp = [1, 1, 2, 2],
    )
    barplot!(ax, tbl.x, tbl.height,
        dodge = tbl.grp,
        color = colors[tbl.grp]
    )
    labels = ["Bayesian IDA-PBC", "Neural IDA-PBC"]
    elements = [PolyElement(polycolor = colors[i]) for i in 1:length(labels)]

    Legend(fig[1,2], elements, labels)
    save("plots/bar.png", fig)
end

function performance_metric(df::DataFrame, tspan)
    q1 = filter(!ismissing, df."/joint_states/theta1/position")
    q1dot = filter(!ismissing, df."/joint_states/theta1/velocity")
    u = filter(!ismissing, df."/joint_states/theta2/effort")
    t = filter(!ismissing, df."/joint_states/header/stamp")
    t .-= t[1]
    trimafter!(tspan, t, q1, q1dot, u)
    @assert length(q1) == length(q1dot) == length(u)
    N = length(q1)
    loss = @. 1 - cos(q1) + q1dot^2 + 1u^2
    sum(loss) / N
end
