#!/bin/bash
echo "Building Visualizer..."
clang++ tests/visualize.cpp tests/vision_algo.cpp -std=c++17 -I/opt/homebrew/include -L/opt/homebrew/lib -lraylib -o tests/algo_vis
if [ $? -eq 0 ]; then
    echo "Running Visualizer..."
    ./tests/algo_vis
else
    echo "Compilation Failed"
fi