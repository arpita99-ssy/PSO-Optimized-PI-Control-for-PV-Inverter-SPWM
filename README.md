# PSO-Optimized-PI-Control-for-PV-Inverter-SPWM
This repository presents the design and simulation of a photovoltaic (PV) inverter system controlled using a Proportional–Integral (PI) controller, whose parameters are optimally tuned using the Particle Swarm Optimization (PSO) algorithm. The inverter employs Sinusoidal Pulse Width Modulation (SPWM) to generate high-quality AC output.

## Overview

This project presents the design and simulation of a photovoltaic (PV) inverter system controlled using a Proportional–Integral (PI) controller. The PI controller parameters are optimized using the Particle Swarm Optimization (PSO) algorithm to improve the control performance of the system.

The inverter uses Sinusoidal Pulse Width Modulation (SPWM) for switching control.

## Key Features

- Photovoltaic (PV) power generation model
- MPPT-based reference generation
- PI controller for control of the power-conversion stage
- PSO-based optimization of PI controller parameters
- SPWM-based inverter switching
- Voltage and current measurement
- MATLAB/Simulink-based simulation
- Power-electronic switching devices including IGBT/Diode

## System Structure

The overall simulation consists of:

PV Array → MPPT / Reference Generation → PI Controller → PWM / SPWM → Power Converter → Output

The `RefGen` MATLAB Function is used for reference generation based on the measured PV voltage and current.

## Software Requirements

- MATLAB
- Simulink
- Simscape Electrical
- Compatible MATLAB/Simulink version required for the Specialized Power Systems blocks used in the original model

> **Compatibility Note:** The original model uses Simscape Electrical Specialized Power Systems blocks. These blocks were removed in newer MATLAB releases, including R2025b. The model may therefore require conversion using the Simscape Electrical Specialized Power Systems Conversion Assistant before simulation in newer MATLAB versions.
Tested Environment: This project was developed and successfully simulated using MATLAB 2024.
