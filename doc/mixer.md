# Mixer Overview

The mixer converts high-level actuator commands—body torques and collective thrust—into individual rotor speeds. Any multi-rotor layout can be described by the geometry and coefficient set supplied to `cs_mixer_init`.

## Why a mixer exists

Controllers reason in the body frame: “apply +0.1 Nm roll torque, -0.05 Nm pitch, +9.8 N thrust”. Motors operate in their own local axis and generate forces proportional to `ω²`. The mixer is the glue between these spaces. It takes the geometry of the airframe (rotor positions, spin directions, thrust axes) and computes how each rotor’s thrust contributes to the global wrench `{τx, τy, τz, T}`.

With that mapping established once, at runtime we simply multiply the desired wrench by a precomputed pseudoinverse to recover the rotor speeds that best realise it.

## Inputs captured by `cs_rotor_config_t`
- Rotor position (lever arm) in the body frame.
- Thrust axis for each motor or propeller (unit vector).
- Spin direction (+1/-1) to capture reaction torque due to drag.
- Thrust and drag coefficients that map `ω² → thrust/torque`.

## Building the allocation matrix
During `cs_mixer_init` we loop over each rotor and derive a column of the allocation matrix `B` (shape 4×N):

1. **Thrust contribution**: `F = k_t ω²` along the thrust axis. Only the axis’ z-component (relative to body frame) contributes to collective thrust.
2. **Moment from thrust**: the lever arm cross thrust axis, scaled by thrust coefficient, produces torque components.
3. **Reaction torque**: `τ_reaction = k_q ω² * direction * axis`, representing motor drag.

Assembled together, column `i` looks like:
```
B[:, i] = [τxᵢ, τyᵢ, τzᵢ, Tᵢ]ᵀ / ω²
```
so multiplying `B` by `ω²` (element-wise square) yields the total wrench.

## Pseudoinverse rationale

Given the allocation `B`, we want `B ω² = w` where `w` is the requested `{τx, τy, τz, T}`. For square quads this can be solved directly; for general N-rotor craft we compute the least-squares solution:

```
ω² ≈ B⁺ w     with   B⁺ = (B Bᵀ)⁻¹ B
```
When `B` is square and invertible this reduces to the exact inverse. For overactuated frames (hex, octo) it selects the minimum-norm solution.

We clamp negative entries to zero (since ω² cannot be negative) and take the square root to recover actual rotor speeds.

## Typical Workflow
1. Configure `cs_rotor_config_t` array for the airframe (quad, hex, coaxial, etc.).
2. Call `cs_mixer_init` once at start-up to compute `B` and `B⁺`.
3. On each control tick, supply the desired body torques and thrust to `cs_mixer_mix`.
4. Feed the resulting rotor speeds to motor models or ESC setpoints (apply saturation there if needed).
5. Optionally call `cs_mixer_eval` to recover the achieved torque/thrust from a speed vector for debugging or plant simulation.

## Relationship to controllers

The mixer is agnostic to the upstream control law. PID, LQR, or MPC all emit the same `{τx, τy, τz, T}` wrench. By slotting the mixer after the attitude loop we keep the controller design independent of airframe geometry. Changing from a “+” quad to an “X” or a coaxial octocopter simply means reinitialising the mixer with a different rotor table—no controller math changes required.

## Example

`examples/attitude_hold/main.c` shows a reference flow:
1. Define a quad geometry and create a mixer.
2. Initialises an attitude PID.
3. Generates a roll command, uses the mixer to convert it into rotor speeds.
4. Feeds those speeds into a simple rigid-body integrator to demonstrate convergence.

This example ties together the estimator output (`cs_state_t`), the controller (`cs_attitude_pid_update`), and the mixer, illustrating how the modular pieces plug together.
