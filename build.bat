@echo off

cmake -E make_directory build
cmake -E chdir build cmake -E time cmake ..
cmake -E time cmake --build build