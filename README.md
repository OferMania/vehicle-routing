# Intro
All instructions were tested on my Ubuntu 22.04.3 desktop. Your mileage may vary on other OS's.

# Getting Started
You'll need to install gcc, cmake, and some build essential tools, if you don't have these already.

```bash
sudo apt install gcc cmake
sudo apt install build-essential
```

# Building and Running

Make a release build directory then run cmake on it.

```bash
mkdir build_release
cmake -S . -B build_release/ -DCMAKE_BUILD_TYPE=Release
cmake --build build_release/
```

To run the VehicleRouting tool using an input from training/problem1.txt, simply do:

```bash
./build_release/VehicleRouting training/problem1.txt
```

# Evaluating a Training Set

Assuming you have python3 installed, once a release build is made (see previous section), you can run evaluateShared.py with this executible over your training set in the training/ directory as follows:

```bash
python3 evaluateShared.py --cmd "./build_release/VehicleRouting" --problemDir training/
```

# Debugging Problems

You can also build a debug version of VehicleRouting, which is useful to figure out the "magic sauce" behind the final output, including the debug-log.out file that is generated. Instructions are similar to those earlier, except we are making a debug build, which enables the LOGGING macro.

```bash
mkdir build_debug
cmake -S . -B build_debug/ -DCMAKE_BUILD_TYPE=Debug
cmake --build build_debug/
```

Then run the VehicleRouting tool as such:

```bash
./build_debug/VehicleRouting training/problem1.txt
```

IMPORTANT: Pay special attention to the debug-log.out file generated from this run! (It's empty for release build runs but nonempty for debug build runs)

# Design Overview

Vehicle Routing feels "similar" to TSP (Traveling Salesman Problem), which is NP-Complete. I couldn't figure out an efficient optimal solution without trying every graph combo (similar to every other NP-Complete problem), so I use a heuristic to traverse nodes.

The heuristic works as follows, at each node, we have 5 choices to make, and Probs is initialized with "weights" corresponding to which choices to make how often. (The "weights" work as follows... Suppose for a moment there's only 2 weights instead of 5 to simplify explanation. "0, 1" means always pick the latter, and "90, 10" means pick first 90% of time, latter 10% of time. I chose to use integer weights to simplify things a bit here). Those 5 choices are: nearest neighbor (ie GreedyNearest), the nearest node closer to us than HQ (ie OnwayNearest) or closest node as fallback, weighted nearest neighbor with closer nodes chosen with higher probability, uniformly random node selection, or just return to HQ.

The heuristic differs a little at the HQ node when we start. If hqGoesToRandom is true, we ignore all weights JUST at HQ and always pick a random node to go from HQ (we still use all the weights at all other nodes). If hqGoesToRandom is false, we still use the weights at HQ, but ignore the "return to HQ weight". Note: When hqGoesToRandom is true, it allows us to explore different solutions for what would normally be deterministic solutions if we start inspecting different nodes first. For example, GreedyNearest where we make the first choice from HQ to different nodes.

We build LOTS of candidate solutions in a single run, but keep the best one (ie the one with lowest score). For release mode, we build 3122 candidate solutions per run. We build 6x more candidate solutions for release mode than debug (I didn't feel like spending all day digging thru the debug log, plus release mode runs 6x faster than debug mode on my machine).

# Code Overview

src/main.cpp   ->  Where everything runs

src/graph.cpp  ->  The graph we build from the input file. It's effectively a directed graph with distances for the edges. The plans for drivers are built here also.

src/scheme.cpp ->  Parametrizes Probs to show what probabilities to do which techniques (nearest node, head to HQ, random node, etc). The logic for deciding which "scheme" to do is here, along with the selection of which next node to visit for a plan.

src/coordinate.h  ->  Coordinate struct declaration used in the graph

src/evaluate_shared.cpp  -> Sigh, I couldn't figure out CPython, so I redid some of the logic in evaluateShared.py with one main purpose: Anytime I build a list of paths (aka candidate solution) for the drivers, I want it validated & scored. main.cpp keeps the best solution built and outputs that in the end.

# If I had more time

The following is a list of things I would attempt if I had more time to work on this project.

- Unit-testing. I have gtest linked into the project, but it's not doing anything due to *_tests.cpp files being empty. I would make those files perform appropriate unit tests on the classes and behviors of the cpp files. I've shipped MVP's (minimum viable products) without unit tests before, as common in startup land at times, but often write unit tests and maybe also some integration tests as the FIRST THING immediately after MVP, to catch and fix potential issues ASAP. This has saved my hide in the past, so it's often the FIRST improvement I would make to any product.

- Parametertrizing VehicleRouting. Add options to toggle things like the number of solutions to try over which behaviors. At the moment, I'm just setting these to what appeared to be reasonable numbers when I was messing with it. I'd likely use tclap here since I like naming arguments.

- Other heuristics. Maybe trying grouping the graph into isolated clusters, in hopes of building more solutions where the number of loads given to a driver is fairly even (or at least do that better than visiting random loads).

- CPython - make one place to evaluate a solution for BOTH C++ and Python code (I don't care what language it's in), and have C++ and Python code both call it. We could using CPython or something similar to this effect. Although src/evaluate_shared.cpp is "mainly" a recoding of evaluteShared.py that runs faster because it's in C++, we really want to reduce redundant code when possible.

- Messing with optimization parameters? I know the solution is to minimize a_1 * d + a_2 * total_m with m <= a_3 (m is the minutes for any given driver, total_m is total minutes across total drivers, d is number of drivers, and a_1 = 500, a_2 = 1, and a_3 = 12 * 60). But what if we messed with a_1, a_2, or a_3? (Like maybe the cost per driver changes, so a_1 is now 600? Or the price per minute goes up due to gas prices, so a_2 is now 1.5) How does that change the load schedules in our optimal solution? Could be fun to explore!!!

- Other fun variants of the Vehicle Routing Problem: What if the loads had different sizes & weights, and you could in some cases squeeze two or more of them into a single truck? Then you could potentially save some money on gas by loading up the truck on nearby loads (especially if a few of those loads come from the same or nearby place), then dropping them all of at once (especially if the dropoff points for those loads are the same or nearby each other). I'm thinking of those Amazon trucks that stop by my block. Everything on that truck is from one warehouse and the dropoff points for the Amazon packages are all adjacent houses.
