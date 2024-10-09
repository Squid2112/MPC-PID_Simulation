# Hybrid PID and MPC Controller Simulation for BESS Enclosure Temperature Control

## Overview

This project simulates a hybrid control system combining Proportional-Integral-Derivative (PID) and Model Predictive Control (MPC) algorithms to maintain the internal temperature of a Battery Energy Storage System (BESS) enclosure under varying ambient temperature conditions.

The simulation is implemented in Java and models the thermal dynamics of a BESS enclosure. It demonstrates how the hybrid controller effectively maintains the enclosure temperature at a desired setpoint despite external temperature fluctuations, similar to real-world scenarios.

---

## Table of Contents

- Introduction
- System Description
  - Thermal Model
  - Control Strategies
    - PID Controller
    - MPC Controller
    - Hybrid Controller
- Simulation Details
  - Ambient Temperature Variation
  - Simulation Parameters
  - Implementation
- Getting Started
  - Prerequisites
  - Running the Simulation
- Results
  - Enclosure Temperature Control
  - Control Input Behavior
- Future Enhancements
- Contributing
- License
- Acknowledgments

---

## Introduction

Maintaining optimal environmental conditions within a BESS enclosure is crucial for battery performance, safety, and longevity. This project explores the implementation of a hybrid control system that combines the simplicity and responsiveness of a PID controller with the predictive optimization capabilities of an MPC controller.

The simulation provides insights into how such a hybrid controller can manage temperature regulation in the presence of external disturbances, specifically varying ambient temperatures that mimic day-night cycles.

---

## System Description

### Thermal Model

The thermal dynamics of the BESS enclosure are modeled using a simplified first-order system based on Newton's Law of Cooling:
` u(t) = Kp * e(t) + Ki * ∫ e(τ) dτ + Kd * de(t)/dt `

Where:
- `T`: Enclosure temperature (°C)
- `T_ambient`: Ambient temperature (°C)
- `C`: Thermal capacity (J/°C)
- `τ` (tau): Thermal time constant (s)
- `K`: Heating/Cooling coefficient (W per unit control input)
- `u`: Control input (heating/cooling power)

### Control Strategies

#### PID Controller

A Proportional-Integral-Derivative (PID) controller adjusts the control input based on the current error, its integral, and derivative:
`u(t) = Kp * e(t) + Ki * ∫ e(τ) dτ + Kd * de(t)/dt`

Where:
- `e(t)`: Error at time `t` (setpoint - measurement)
- `Kp`, `Ki`, `Kd`: PID gains


#### MPC Controller

The Model Predictive Control (MPC) algorithm optimizes future control actions over a prediction horizon by minimizing a cost function, considering future system behavior and constraints.

- **Prediction Horizon**: Number of future time steps considered
- **Cost Function**: Sum of squared errors between predicted temperatures and setpoint
- **Constraints**: Limits on control inputs and system states (not implemented in this simplified version)

#### Hybrid Controller

The hybrid controller combines PID and MPC controllers:

- **PID Control**: Used when the error is within a predefined threshold, providing quick and efficient adjustments.
- **MPC Control**: Activated when the error exceeds the threshold, optimizing control actions over the prediction horizon.

---

## Simulation Details

### Ambient Temperature Variation

The ambient temperature varies sinusoidally to simulate day-night cycles:
`T_ambient(t) = T_base + A * sin( (2π * t) / T_period )`

Where:
- `T_base`: Average ambient temperature (°C)
- `A`: Amplitude of temperature fluctuation (°C)
- `T_period`: Period of fluctuation (s)

### Simulation Parameters

- Time Step (Δt): 60 seconds
- Total Simulation Time: 48 hours (172,800 seconds)
- Thermal Capacity (`C`): 5,000 J/°C
- Thermal Time Constant (`τ`): 600 seconds
- Heating/Cooling Coefficient (`K`): 1,000 W per unit control input
- Setpoint Temperature: 25°C
- PID Gains:
  - `Kp = 0.5`
  - `Ki = 0.1`
  - `Kd = 0.05`
- MPC Prediction Horizon: 10 steps
- Control Input Limits: `-1.0 ≤ u ≤ 1.0`

### Implementation

The simulation consists of the following Java classes:

- `PIDController`: Implements the PID control algorithm.
- `MPCController`: Implements the MPC algorithm with a simple optimization loop.
- `HybridController`: Determines whether to use PID or MPC based on the error threshold.
- `Simulation`: Runs the simulation loop, updating the enclosure temperature and control inputs over time.

---

## Getting Started

### Prerequisites

- Java Development Kit (JDK) 8 or higher
- An IDE or text editor for Java (e.g., Eclipse, IntelliJ IDEA, VS Code)

### Running the Simulation

1. Clone the Repository: `git clone https://github.com/Squid2112/MPC-PID_Simulation.git`
2. Navigate to the Project Directory: `cd MPC-PID_Simulation`
3. Compile the Java Files: `javac *.java`
4. Run the Simulation: `java Simulation`
5. View the Output
   > Time: 164520 s, Enclosure Temp:  18.74 °C, Ambient Temp:  19.34 °C, Control Input:   0.10<br>
   > Time: 164580 s, Enclosure Temp:  20.01 °C, Ambient Temp:  19.37 °C, Control Input:   0.10<br>
   > Time: 164640 s, Enclosure Temp:  21.15 °C, Ambient Temp:  19.41 °C, Control Input:   0.10<br>
   > Time: 164700 s, Enclosure Temp:  22.18 °C, Ambient Temp:  19.44 °C, Control Input:   0.10<br>
   > Time: 164760 s, Enclosure Temp:  23.11 °C, Ambient Temp:  19.48 °C, Control Input:   0.10<br>
6. Analyze the Results

Optionally, you can modify the code to store simulation data to a file or plot the results using a visualization tool.

---

## Results

### Enclosure Temperature Control

The hybrid controller maintains the enclosure temperature close to the setpoint despite ambient temperature fluctuations. The PID controller handles minor adjustments efficiently, while the MPC controller optimizes control actions when larger deviations occur due to significant ambient temperature changes.

### Control Input Behavior

The control input varies over time to counteract the influence of the ambient temperature. It adjusts heating or cooling actions to maintain the enclosure temperature at the desired setpoint. The MPC controller anticipates future ambient temperature changes, optimizing control inputs over the prediction horizon.

---

## Future Enhancements

- Internal Heat Generation: Model the heat produced by batteries during operation.
- Random Disturbances: Include unpredictable environmental factors.
- Advanced MPC Optimization: Implement a proper optimization solver to handle constraints and improve performance.
- Constraints Handling: Add limits on control inputs and system states.
- Multivariable Control: Extend the controller to manage other environmental variables like humidity.
- Visualization: Incorporate plotting libraries to visualize temperature and control input over time.

---

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the Repository
2. Create a New Branch: `git checkout -b feature/your-feature-name`
3. Commit Your Changes: `git commit -am 'Add some feature'`
4. Submit a Pull Request

---

## License

This project is licensed under the [MIT License](https://opensource.org/license/mit).

---

## Acknowledgments

- Inspiration: Control systems applications in battery energy storage.
- Resources: Thanks to the open-source community for providing resources and tools.
- Contributors: All who contribute to improving this simulation.

---

**Note**: This simulation is a simplified representation and may not cover all complexities of a real BESS enclosure. For deployment in real systems, consider consulting with control systems engineers and utilizing advanced modeling and optimization tools.
