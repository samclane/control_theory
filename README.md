# Control Systems Visualizer

## Overview

This code provides a real-time visual simulation of different control algorithms like PID, Bang-Bang, P-Only, Fuzzy Logic, and LQR. It uses the VPython library for the 3D visualization and graphs. It also has a customizable function generator for simulating the target position over time.

## Dependencies

- VPython
- NumPy
- Enum

## Features

- Real-time 3D visualization of control algorithms.
- Dynamic sliders for tuning controller parameters.
- FFT analysis for error signals.
- Noise addition for realistic simulation.
- Supports Sine, Square, and Sawtooth target functions.

## Components

### FunctionType Enum

An enumeration for the types of functions that the `FunctionGenerator` can use: SINE, SQUARE, SAWTOOTH.

### FunctionGenerator

A class that generates a function value for a given point in time (`t`). You can set parameters like amplitude, frequency, offset, noise level, and function type.

### ControlVisualizer

A class responsible for updating the visual elements in the VPython scene. It also updates the FFT analysis graph.

### Controller Classes

Different implementations of controllers are imported from the `controllers` module. These include PID, Bang-Bang, P-Only, Fuzzy Logic, and LQR controllers.

### UI Components

- Sliders for tuning controller parameters.
- Checkboxes for enabling/disabling individual controllers.
- Sliders for setting up the `FunctionGenerator`.
- Radio buttons for selecting the function type.

### Main Loop

This is where the magic happens. It updates the scene and graphs in real-time.

## How To Run

1. Make sure you've installed all dependencies.
2. Run the code.
