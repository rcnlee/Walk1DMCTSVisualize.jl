module Walk1DMCTSVisualize

using Plots, Walk1DMDP, POMDPToolbox, MCTS

export Walk1DMCTSVis

mutable struct Walk1DMCTSVis
    mdp::Walk1D
    s::Walk1DState
    actions::Vector{Vector{Float64}}
    next_states::Vector{Vector{Float64}}
    ns::Vector{Int}
    radii::Vector{Vector{Float64}}
end
Walk1DMCTSVis(mdp::Walk1D, s::Walk1DState) = Walk1DMCTSVis(mdp, s, Vector{Float64}[], Int[], Vector{Float64}[], Float64[])

function circle(x,y,r)
    c = Shape(Plots.partialcircle(0,2π,20,r))
    translate!(c, x, y)
    c
end

@recipe function f(vis::Walk1DMCTSVis, index::Int, h::SimHistory)
    m = vis.mdp
    xlim := (-1, (m.p.threshx+1))
    ylim := (-m.p.threshx-1, m.p.threshx+1)
	aspect_ratio := :equal 
    @series begin
        label := "actions"
        alpha := 0.25
        x = state_hist(h)[end-1].t+1
        y = vis.next_states[index]
        [circle(x,y,r) for (y,r) in zip(y, vis.radii[index])] 
    end
    @series begin
        seriestype := :line
        color := :black
        label := "thresh+"
        x = [-2.0, m.p.t_max]
        y = [m.p.threshx, m.p.threshx] 
        x, y
    end
    @series begin
        seriestype := :line
        color := :black
        label := "thresh-"
        x = [-2.0, m.p.t_max]
        y = [-m.p.threshx, -m.p.threshx] 
        x, y
    end
    @series begin
        label := "path"
        color := :blue
        x = [s.t for s in state_hist(h)[1:end-1]]
        y = [s.x for s in state_hist(h)[1:end-1]]
        x, y
    end
    @series begin
        seriestype := :scatter
        label := "current position"
        t = state_hist(h)[end-1].t
        x = state_hist(h)[end-1].x
        if abs(x) < abs(m.p.threshx)
            color := :blue
        else
            color := :red
        end
        [t], [x]
    end
end

function MCTS.notify_listener(vis::Walk1DMCTSVis,dsb::DSBPlanner,s,a,sp,r,snode,sanode,spnode)
    if s == vis.s
        tree = get(dsb.tree)
        sol = dsb.solver
        actions = [tree.a_labels[c] for c in tree.children[snode]]
        if isempty(vis.actions) || (actions != vis.actions[end])  #only push the deltas
            push!(vis.actions, actions)
            push!(vis.next_states, [tree.s_labels[tree.transitions[c][1][1]].x for c in tree.children[snode]])  #deterministic transition
            push!(vis.ns, tree.total_n[snode])
            push!(vis.radii, [sol.r0_action/tree.total_n[snode]^sol.lambda_action for action in actions])
        end
    end
end
function MCTS.notify_listener(vis::Walk1DMCTSVis,asb::ASBPlanner,s,a,sp,r,snode,sanode,spnode)
    if s == vis.s
        tree = get(asb.tree)
        sol = asb.solver
        actions = [tree.a_labels[c] for c in tree.children[snode]]
        if isempty(vis.actions) || (actions != vis.actions[end])  #only push the deltas
            push!(vis.actions, actions)
            push!(vis.next_states, [tree.s_labels[tree.transitions[c][1][1]].x for c in tree.children[snode]])  #deterministic transition
            push!(vis.ns, tree.total_n[snode])
            push!(vis.radii, [tree.a_radius[x] for x in tree.children[snode]])
        end
    end
end

end # module
