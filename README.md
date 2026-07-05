# AM64x Motor Control Project

This repository contains two main firmware projects running on the AM64x platform:

---

## 1. PRU Project — TestPru

**Folder:** `TestPru`

This project runs on the PRU (Programmable Real-Time Unit).

### Purpose

* Low-level real-time PWM handling
* Deadtime generation and safety logic
* Timing-critical control tasks
* Interface with gate driver signals

### Key Features

* Deterministic real-time execution
* Minimal latency control path
* Supports PWM update from R5 core

---

## 2. R5F FreeRTOS Project — posix_demo_am64x-sk_r5fss0-0_freertos_ti-arm-clang

**Folder:** `posix_demo_am64x-sk_r5fss0-0_freertos_ti-arm-clang`

This project runs on the ARM Cortex-R5F core using FreeRTOS.

### Purpose

* Field Oriented Control (FOC)
* Current control loop (PI controller in dq frame)
* Clarke and Park transforms
* Observer and angle estimation
* Communication with PRU and ADC

### Key Features

* Real-time motor control loop
* ADC current acquisition (ADS8688)
* dq-axis PI regulation
* PWM duty updates via PRU interface

---

## System Overview

* PRU handles **PWM + deadtime generation**
* R5F handles **control algorithms**
* ADC feedback feeds into R5F control loop
* Duty cycles are transferred from R5F → PRU

---

## Notes

* Ensure PRU firmware is loaded before enabling PWM outputs
* ADC offsets must be calibrated before closed-loop operation
* PI controller requires stable current sensing for correct operation
