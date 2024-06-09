# deactivate plot GUI, which is not available in Docker
ENV["GKSwstype"] = "100"

# instantiate project
import Pkg
Pkg.activate(@__DIR__)
Pkg.instantiate()

using ClosedLoopReachability
ClosedLoopReachability.LazySets.deactivate_assertions()

# create output folder and table
const TARGET_FOLDER = "results"
const RESULTS_FILE = "results.csv"

# function to run benchmarks
function main()
    if !isdir(TARGET_FOLDER)
        mkdir(TARGET_FOLDER)
    end
    global io = open(joinpath(TARGET_FOLDER, RESULTS_FILE), "w")
    print(io, "benchmark,instance,result,time\n")

    println("Running AINNCS benchmarks...")

    println("###\nRunning ACC benchmark\n###")
    include("models/ACC/ACC.jl")

    println("###\nRunning TORA benchmark\n###")
    include("models/TORA/TORA.jl")

    println("###\nRunning Unicycle benchmark\n###")
    include("models/Unicycle/Unicycle.jl")

    println("###\nRunning VerticalCAS benchmark\n###")
    include("models/VerticalCAS/VerticalCAS.jl")

    println("###\nRunning InvertedPendulum benchmark\n###")
    include("models/InvertedPendulum/InvertedPendulum.jl")

    println("###\nRunning InvertedTwoLinkPendulum benchmark\n###")
    include("models/InvertedTwoLinkPendulum/InvertedTwoLinkPendulum.jl")

    println("###\nRunning Airplane benchmark\n###")
    include("models/Airplane/Airplane.jl")

    println("###\nRunning AttitudeControl benchmark\n###")
    include("models/AttitudeControl/AttitudeControl.jl")

    println("###\nRunning Quadrotor benchmark\n###")
    include("models/Quadrotor/Quadrotor.jl")

    println("###\nRunning SpacecraftDocking benchmark\n###")
    include("models/SpacecraftDocking/SpacecraftDocking.jl")

    println("###\nRunning NAV benchmark\n###")
    include("models/NAV/NAV.jl")

    print(io, "\n")
    println("Finished running benchmarks.")
    close(io)
    nothing
end

# run benchmarks
main()
