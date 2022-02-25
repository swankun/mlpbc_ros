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

######################################
# Constants
######################################

function publication_theme()
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
end


######################################
# Read I/O
######################################

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

function getdf(filename="20220217/idapbc_0rings_00.csv")
    # Read from file
    file = CSV.File(filename, ignoreemptyrows=true)
    df = DataFrame(file)

    # Pick relevant columns
    keep = names(df, r"/joint_states/theta*")
    filter!(s->!occursin(r"theta1/effort", s), keep)
    insert!(keep, 1, "/joint_states/header/stamp")
    select!(df, keep)
    dropmissing!(df, disallowmissing=true)

    # Rename columns
    rename!(df,
        Dict(
            Symbol("/joint_states/theta1/position")=>"q1",
            Symbol("/joint_states/theta2/position")=>"q2",
            Symbol("/joint_states/theta1/velocity")=>"q1dot",
            Symbol("/joint_states/theta2/velocity")=>"q2dot",
            Symbol("/joint_states/theta2/effort")=>"u",
            Symbol("/joint_states/header/stamp")=>"t",
        )
    )

    # Remove points during motor disabled
    subset!(df, :u => ByRow(!iszero))   

    # Apply offsets
    transform!(df, 
        :t => x->x.-first(df.t), # time shift
        :q1 => x->x.-pi, # 0 is upward
        renamecols=false
    )   

    return df
end

######################################
# Data processing
######################################

function view_swing(df::DataFrame)
    lqr_roa(q1,q1dot) = (1-cos(q1) < 1-cosd(5)) && (abs(q1dot) < 0.5)
    i0 = findfirst(x->abs(x)>0.9*pi, df.q1dot) # first "kick"
    i1 = findfirst(lqr_roa.(df.q1, df.q1dot)) # first enters LQR's RoA
    @view df[i0:i1, :]
end

function performance_metric(df::Union{DataFrame,SubDataFrame})
    N = lastindex(df, 1)
    loss = @. 1 - cos(df.q1) + 1/2*df.q1dot^2 + 1/2*df.u^2
    sum(loss)
end


######################################
# Utilities for batch processing
######################################

function process_experiment(file)
    raw = getdf(file)
    df = view_swing(raw)
    downsample = @view df[1:20:lastindex(df,1), :]
    N = length(1:20:lastindex(df,1))
    return downsample, performance_metric(downsample)
end

function process_experiments(datadir)
    files = readdir(datadir, join=true)
    res = process_experiment.(files)
    @. (first(res), last(res))
end


######################################
# Plotting
######################################

function plot_experiment(data; parent=Figure(800,300), kwargs...)
    ax1 = Axis(parent[1,1]; kwargs...)
    lines!(ax1, data.t, data.q1, color=:black)
    ax2 = Axis(parent[1,2]; kwargs...)
    lines!(ax2, data.t, data.q1dot, color=:black)
    return parent
end

function plot_experiments(data)
    N = length(data)
    fig = Figure(resolution=(800,300*N))
    foreach(enumerate(data)) do (index, df)
        plot_experiment(df, parent=fig[index, 1]; kwargs...);
    end
    return fig
end

function plot_experiments(data, titles; kwargs...)
    N = length(data)
    fig = Figure(resolution=(800,300*N))
    foreach(enumerate(data)) do (index, df)
        plot_experiment(df, parent=fig[index, 1]; title=popfirst!(titles), kwargs...);
    end
    return fig
end

function plot_scores(scores, category, group; catlabels=nothing, grplabels=nothing)
    set_theme!(publication_theme())
    scat = sort(unique(category))
    M = length(unique(group))
    catlabels = isnothing(catlabels) ? string.(scat) : catlabels
    grplabels = isnothing(grplabels) ? string.(collect(1:M)) : grplabels

    set_theme!(publication_theme())
    fig = Figure(resolution=(800,600))
    colors = Makie.wong_colors()
    ax = Axis(fig[2,1], xticks = (scat, catlabels))
    barplot!(ax, category, scores,
        dodge = group,
        color = colors[group]
    )
    elements = [PolyElement(polycolor = colors[i]) for i in 1:length(grplabels)]
    Legend(fig[1,1], elements, grplabels, 
        orientation=:horizontal,
        # tellheight = false,
        # tellwidth = false,
        # halign = :center,
        # valign = :top,
    )
    set_theme!()
    return fig
end


######################################
# Experiment-specific code
######################################

function batch_20220223(; plot_traj=true, plot_bar=true)
    datadir = "20220223"
    data, scores = process_experiments(datadir)
    if (plot_traj)
        filenames = readdir(datadir)
        fig = plot_experiments(data, filenames)
        save("plots/out.png", fig)
    end
    if (plot_bar)
        score_avg = map(mean, Iterators.partition(scores, 3))
        score_avg ./= minimum(score_avg)
        fig = plot_scores(score_avg, 
            repeat([0,1,2,3,4], 2), 
            [ones(Int,5); 2*ones(Int,5)]; 
            catlabels=["$(n) rings" for n=0:4],
            grplabels=["Bayesian", "NeuralPBC"]
        )
        save("plots/bar.png", fig)
    end
end

function batch_20220217(; plot_traj=true, plot_bar=true)
    datadir = "20220217"
    data, scores = process_experiments(datadir)
    if (plot_traj)
        filenames = readdir(datadir)
        fig = plot_experiments(data, filenames)
        save("plots/out.png", fig)
    end
    if (plot_bar)
        score_avg = [mean(scores[1:2]); scores[3]; mean(scores[4:5]); scores[6]]
        score_avg ./= minimum(score_avg)
        fig = plot_scores(score_avg, 
            [0,1,0,1],
            [1,1,2,2];
            catlabels=["$(n) rings" for n=[0,2]], 
            grplabels=["Bayesian", "NeuralIDAPBC"]
        )
        save("plots/bar.png", fig)
    end
end
