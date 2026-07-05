# AM64x Motor Control Project

This repository contains two main firmware projects running on the AM64x platform for BLDC / PMSM motor control using a dual-core architecture.

---

## 1. PRU Project — TestPru

**Folder:** `TestPru`

This project runs on the PRU (Programmable Real-Time Unit).

### Purpose

* Low-level real-time PWM generation
* Deadtime insertion and switching safety control
* Timing-critical motor control tasks
* Interface to the gate driver (DRV8302)

### Key Features

* Deterministic real-time execution
* Very low latency PWM path
* Supports duty cycle updates from R5 core
* Hardware-level PWM synchronization

---

## 2. R5F FreeRTOS Project — posix_demo_am64x-sk_r5fss0-0_freertos_ti-arm-clang

**Folder:** `posix_demo_am64x-sk_r5fss0-0_freertos_ti-arm-clang`

This project runs on the ARM Cortex-R5F core using FreeRTOS.

### Purpose

* Field Oriented Control (FOC) implementation
* Current control loop (PI controller in dq frame)
* Clarke and Park transformations
* Rotor angle estimation and observer logic
* Communication with PRU PWM subsystem and ADC acquisition

### Key Features

* Real-time motor control loop execution
* Current measurement using **ADS8688 ADC**
* dq-axis PI regulation for torque control
* PWM duty cycle computation and updates via PRU interface

---

## Hardware Interfaces

### ADC — ADS8688

The system uses the **ADS8688 external ADC** for high-precision current sensing.

* 16-bit resolution SAR ADC
* SPI communication interface
* Measures phase currents and reference signals
* Used for Clarke/Park transforms and FOC feedback loop
* Requires offset calibration during startup (zero-current phase)

---

### Gate Driver — DRV8302 (IGD)

The inverter is driven using the **DRV8302 gate driver**.

* Three-phase MOSFET gate driver
* PWM input controlled by PRU subsystem
* Includes programmable deadtime protection
* Supports current sensing interface (not used internally here due to external ADC)

In this project it is responsible for:

* Driving U, V, W inverter phases
* Applying PWM signals generated from R5 control output
* Ensuring safe switching via deadtime control
* Supporting startup voltage offset calibration routines

---

## System Overview

* **PRU** handles PWM generation and deadtime insertion
* **R5F** executes FOC control algorithms
* **ADS8688** provides current feedback measurements
* **DRV8302** drives the power inverter stage
* Duty cycles are computed on R5F and transferred to PRU

---

## Notes

* Ensure PRU firmware is loaded before enabling PWM outputs
* ADC offset calibration must be completed before closed-loop control
* PI controller performance depends on stable and correctly scaled current sensing
* Floating ADC inputs will destabilize FOC operation
