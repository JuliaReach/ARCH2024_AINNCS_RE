# # NAV
#
# The NAV benchmark has the goal of navigating a robot to a goal region while
# avoiding an obstacle.

module NAV  #jl

using ClosedLoopReachability
import MAT, OrdinaryDiffEq, Plots
using ReachabilityBase.CurrentPath: @current_path
using ReachabilityBase.Timing: print_timed
using Plots: plot, plot!, savefig

# ## Model

# There are 4 state variables ``(x_1, …, x_4)`` and two control inputs
# ``(u_1, u_2)``.

vars_idx = Dict(:states => 1:4, :controls => 5:6)

@taylorize function NAV!(dx, x, p, t)
    x₁, x₂, x₃, x₄, u₁, u₂ = x

    dx[1] = x₃ * cos(x₄)
    dx[2] = x₃ * sin(x₄)
    dx[3] = u₁
    dx[4] = u₂
    dx[5] = zero(u₁)
    dx[6] = zero(u₂)
    return dx
end;

# We are given a neural-network controller with two hidden layers of 64 resp. 32
# neurons with ReLU activations, followed by the output layer with tanh
# activations. The controller was trained in a set-based manner to improve its
# verifiable robustness by integrating reachability analysis into the training
# process [^KLA]. The controller has 4 inputs (the state variables) and 2
# outputs (``u_1, u_2``).

path = @current_path("NAV", "nn-nav-set.mat")
controller_robust = read_MAT(path; act_key="act");

# The control period is 0.2 time units.

period = 0.2;

# ## Specification

X₀ = Hyperrectangle([3.0, 3.0, 0.0, 0.0], [0.1, 0.1, 0.0, 0.0])
U₀ = ZeroSet(2);

# The control problem is:

ivp = @ivp(x' = NAV!(x), dim: 6, x(0) ∈ X₀ × U₀)
prob = ControlledPlant(ivp, controller_robust, vars_idx, period);

# The specification is to reach a goal region at ``t = 6`` time units while
# avoiding an obstacle. A sufficient condition for guaranteed verification is to
# overapproximate the result with hyperrectangles and check inclusion for a
# longer time horizon including ``t = 6``.

T = 6.0
T_warmup = 2 * period  # shorter time horizon for warm-up run

goal_states = cartesian_product(BallInf(zeros(2), 0.5), Universe(4))

unsafe_states = cartesian_product(BallInf([1.5, 1.5], 0.5), Universe(4))

# safety property
function predicate_set_avoid(R)
    return isdisjoint(overapproximate(R, Hyperrectangle), unsafe_states)
end

# reachability property
function predicate_set_reach(R)
    if T ∈ tspan(R)
        return overapproximate(R, Hyperrectangle) ⊆ goal_states
    end
    return true
end

function predicate(sol; silent::Bool=false)
    for F in sol, R in F
        if !predicate_set_avoid(R)
            silent || println("  Violation for time range $(tspan(R)).")
            return false
        end
        if !predicate_set_reach(R)
            silent || println("  Goal not fully reached.")
            return false
        end
    end
    return true
end;

# ## Analysis

# To enclose the continuous dynamics, we use a Taylor-model-based algorithm:

algorithm_plant = TMJets(abstol=1e-3, orderT=3, orderQ=1);

# To propagate sets through the neural network, we use the `DeepZ` algorithm:

algorithm_controller = DeepZ();

# The verification benchmark is given below:

function benchmark(; T=T, silent::Bool=false)
    ## Solve the controlled system:
    silent || println("Flowpipe construction:")
    res = @timed solve(prob; T=T, algorithm_controller=algorithm_controller,
                       algorithm_plant=algorithm_plant)
    sol = res.value
    silent || print_timed(res)

    ## Check the property:
    silent || println("Property checking:")
    res = @timed predicate(sol)
    silent || print_timed(res)
    if res.value
        silent || println("  The property is satisfied.")
        result = "verified"
    else
        silent || println("  The property may be violated.")
        result = "not verified"
    end

    return sol, result
end;

# Run the verification benchmark and compute some simulations:

## Run the verification benchmark:
benchmark(; T=T_warmup, silent=true)  # warm-up
res = @timed benchmark()  # benchmark
sol, result = res.value
@assert (result == "verified") "verification failed"
println("Total analysis time:")
print_timed(res)
io = isdefined(Main, :io) ? Main.io : stdout
print(io, "NAV,robust,$result,$(res.time)\n")

println("Simulation:")
res = @timed simulate(prob; T=T, trajectories=1, include_vertices=true)
sim = res.value
print_timed(res)

# ## Results

TARGET_FOLDER = isdefined(Main, :TARGET_FOLDER) ? Main.TARGET_FOLDER : @__DIR__

# Script to plot the results:

function plot_helper(vars)
    fig = plot()
    plot!(fig, project(goal_states, vars); color=:cyan, lab="goal")
    plot!(fig, project(unsafe_states, vars); color=:red, alpha=:0.2, lab="unsafe")
    plot!(fig, sol; vars=vars, color=:yellow, lw=0, alpha=1, lab="")
    plot!(fig, project(X₀, vars); c=:cornflowerblue, alpha=1, lab="X₀")
    plot_simulation!(fig, sim; vars=vars, color=:black, lab="")
    return fig
end;

# Plot the results:

vars = (1, 2)
fig = plot_helper(vars)
plot!(fig; xlab="x₁", ylab="x₂")
savefig(fig, joinpath(TARGET_FOLDER, "NAV-robust.png"))

end  #jl
nothing  #jl

# ## References

# [^KLA]: Lukas Koller, Tobias Ladner, and Matthias Althoff (2024). *Set-based
#         training for neural network verification*.
#         [arXiv:2401.14961](https://arxiv.org/pdf/2401.14961.pdf).
