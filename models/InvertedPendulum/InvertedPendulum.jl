# # Inverted Pendulum
#
# The Inverted Pendulum benchmark is a classical model of motion.

module InvertedPendulum  #jl

using ClosedLoopReachability
import OrdinaryDiffEq, Plots
using ReachabilityBase.CurrentPath: @current_path
using ReachabilityBase.Timing: print_timed
using ClosedLoopReachability: SingleEntryVector
using Plots: plot, plot!, xlims!, ylims!, lens!, bbox, savefig

# ## Model

# A ball of mass ``m`` is attached to a massless beam of length ``L``. The beam
# is actuated with a torque ``T``. We assume viscous friction with coefficient
# ``c``.
#
# The governing equation of motion can be obtained as follows:
#
# ```math
# \ddot{θ} = \dfrac{g}{L} \sin(θ) + \dfrac{1}{m L^2} (T - c \dot{θ})
# ```
# where ``θ`` is the angle that the link makes with the upward vertical axis,
# ``\dot{θ}`` is the angular velocity, and ``g`` is the gravitational
# acceleration. The state vector is ``(θ, \dot{θ})``. The model constants are
# chosen as ``m = L = 0.5``, ``c = 0``, and ``g = 1``.

vars_idx = Dict(:states => 1:2, :controls => 3)

const m = 0.5
const L = 0.5
const c = 0.0
const g = 1.0
const gL = g / L
const mL = 1 / (m * L^2)

@taylorize function InvertedPendulum!(dx, x, p, t)
    θ, θ′, T = x

    dx[1] = θ′
    dx[2] = gL * sin(θ) + mL * (T - c * θ′)
    dx[3] = zero(T)
    return dx
end;

# We are given a neural-network controller with 2 hidden layers of 25 neurons
# each and ReLU activations. The controller has 2 inputs (the state variables)
# and 1 output (``T``).

path = @current_path("InvertedPendulum", "controller_single_pendulum.nnet")
controller = read_NNet(path);

# The control period is 0.05 time units.

period = 0.05;

# ## Specification

# The uncertain initial condition is ``θ \in [1, 1.175], \dot{θ} \in [0, 0.2]``.

X₀ = Hyperrectangle(low=[1.0, 0], high=[1.175, 0.2])
U₀ = ZeroSet(1);

# The control problem is:

ivp = @ivp(x' = InvertedPendulum!(x), dim: 3, x(0) ∈ X₀ × U₀)
prob = ControlledPlant(ivp, controller, vars_idx, period);

# The safety specification is that ``θ ∈ [0, 1]`` for ``t ∈ [0.5, 1]`` (i.e.,
# the control periods ``10 ≤ k ≤ 20``). A sufficient condition for guaranteed
# verification is to overapproximate the result with hyperrectangles.

unsafe_states = UnionSet(HalfSpace(SingleEntryVector(1, 3, -1.0), -1.0),
                         HalfSpace(SingleEntryVector(1, 3, 1.0), 0.0))

function predicate_set(R)
    t = tspan(R)
    return t.hi <= 0.5 ||
           isdisjoint(overapproximate(R, Hyperrectangle), unsafe_states)
end

function predicate(sol)
    for F in sol
        t = tspan(F)
        if t.hi <= 0.5
            continue
        end
        for R in F
            if !predicate_set(R)
                return false
            end
        end
    end
    return true
end

T = 1.0
T_warmup = 2 * period;  # shorter time horizon for warm-up run

# ## Analysis

# To enclose the continuous dynamics, we use a Taylor-model-based algorithm:

algorithm_plant = TMJets(abstol=1e-9, orderT=5, orderQ=1);

# To propagate sets through the neural network, we use the `DeepZ` algorithm. We
# also use an additional splitting strategy to increase the precision.

algorithm_controller = DeepZ()
splitter = BoxSplitter([[1.1, 1.16], [0.09, 0.145, 0.18]]);

# The verification benchmark is given below:

function benchmark(; T=T, silent::Bool=false)
    ## Solve the controlled system:
    silent || println("Flowpipe construction:")
    res = @timed solve(prob; T=T, algorithm_controller=algorithm_controller,
                       algorithm_plant=algorithm_plant, splitter=splitter)
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

benchmark(T=T_warmup, silent=true)  # warm-up
res = @timed benchmark(T=T)  # benchmark
sol, result = res.value
@assert (result == "verified") "verification failed"
println("Total analysis time:")
print_timed(res)
io = isdefined(Main, :io) ? Main.io : stdout
print(io, "SinglePendulum,,$result,$(res.time)\n")

println("Simulation:")
res = @timed simulate(prob; T=T, trajectories=10, include_vertices=true)
sim = res.value
print_timed(res);

# ## Results

TARGET_FOLDER = isdefined(Main, :TARGET_FOLDER) ? Main.TARGET_FOLDER : @__DIR__

# Script to plot the results:

function plot_helper()
    vars = (0, 1)
    fig = plot(leg=:topright)
    lab = "unsafe"
    for B in unsafe_states
        unsafe_states_projected = cartesian_product(Interval(0.5, 1.0), project(B, [vars[2]]))
        plot!(fig, unsafe_states_projected; color=:red, alpha=:0.2, lab=lab)
        lab = ""
    end
    plot!(fig, sol; vars=vars, color=:yellow, lw=0, alpha=1, lab="")
    plot_simulation!(fig, sim; vars=vars, color=:black, lab="")
    initial_states_projected = cartesian_product(Singleton([0.0]), project(X₀, [vars[2]]))
    plot!(fig, initial_states_projected; c=:cornflowerblue, alpha=1, m=:none, lw=7, lab="X₀")
    xlims!(0, T)
    ylims!(0.5, 1.2)
    plot!(fig; xlab="t", ylab="θ")
    lens!(fig, [0.49, 0.52], [0.99, 1.01]; inset=(1, bbox(0.1, 0.6, 0.3, 0.3)),
          lc=:black, xticks=[0.5], yticks=[1.0], subplot=3)
    return fig
end;

# Plot the results:

fig = plot_helper()
savefig(fig, joinpath(TARGET_FOLDER, "InvertedPendulum.png"))

end  #jl
nothing  #jl
