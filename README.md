# Intro
All instructions were tested on my Ubuntu 22.04.3 desktop. Your mileage may vary on other OS's.

# Getting Started
You'll need to install gcc, cmake, and some build essential tools, if you don't have these already.

```bash
sudo apt install gcc cmake
sudo apt install build-essential
```

# Building and Running

Make a build directory then run cmake and make in that.

```bash
mkdir build
cd build
cmake ..
make
```

Two executibles are made: VehicleRouting and VehicleRoutingTests. The latter consists of unit tests made, especially over the Graph.

To run the unit tests, simply do:

```bash
./VehicleRoutingTests
```

To run the VehicleRouting tool using an input from ../training/problem1.txt, simply do:

```bash
./VehicleRouting ../training/problem1.txt
```
