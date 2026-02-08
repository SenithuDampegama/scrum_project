# CDIO Scrum Robot Project

## Project Overview

This repository contains the implementation of a team-based robotics project developed under the **Conceive, Design, Implement, Operate (CDIO)** framework and managed using **Scrum** principles. The objective of the project is to design and build an autonomous walking robot capable of demonstrating multiple biologically inspired behaviours in a controlled environment.

The project was executed over **five sequential sprints**, each with a dedicated Scrum Master and a clearly defined set of deliverables. Although the development followed a logical engineering dependency order, Scrum artefacts such as sprint ownership, task tracking, and incremental delivery were maintained throughout the project.

---

## CDIO Tasks & Functional Requirements

The robot was required to satisfy the following core tasks:

* **Task 1 – Locomotion:** Enable the robot to walk using servo-driven mechanisms.
* **Task 2 – Object Avoidance:** Detect and avoid obstacles in the environment using distance sensors.
* **Task 3 – Phototaxis:** Move towards a bright light source and stop at approximately **20 cm** from it.
* **Task 4 – Predator Response:** Detect rapid motion in the environment, immediately stop movement, and deploy wings as a defensive response.

---

## Development Methodology

### CDIO Framework

The project followed the CDIO lifecycle:

* **Conceive:** System concept, behavioural definition, and mechanical feasibility analysis.
* **Design:** CAD modelling, electronics design, and system architecture definition.
* **Implement:** Hardware assembly, firmware development, and behaviour implementation.
* **Operate:** System testing, tuning, validation, and demonstration.

### Scrum Implementation

To align with Scrum methodology, the project was divided into **five sprints**, each led by a different Scrum Master. Each sprint produced tangible and reviewable outputs.

---

## Sprint Plan & Team Roles

| Sprint   | Focus Area                      | Duration | Scrum Master      |
| -------- | ------------------------------- | -------- | ----------------- |
| Sprint 1 | CAD Design                      | 3 weeks  | Dhanip Modi       |
| Sprint 2 | Electronics Integration         | 1 week   | Adwil Joshy       |
| Sprint 3 | Programming & Behaviour Control | 2 weeks  | Senithu Dampegama |
| Sprint 4 | Testing & Validation            | 1 week   | Misba Babu        |
| Sprint 5 | Documentation & Presentation    | 1 week   | Dinuli Jayaweera  |

The sprint order was selected to reflect real-world engineering dependencies, ensuring stable integration and reducing rework.

---

## Hardware Components

* Raspberry Pi Pico
* 10 × SG90 (180°) servo motors
* 2 × MG996R (360°) servo motors
* Adafruit BH1750 Light Sensor
* Adafruit PCA9685 Servo Driver
* Ultrasonic / IR distance sensors
* 9V battery and AA battery packs

---

## Software & Tools

* **Programming:** MicroPython / C++ (Pico)
* **CAD:** Fusion 360
* **Electronics Design:** Breadboard prototyping
* **Version Control:** Git & GitHub
* **Project Management:** GitHub Projects (Scrum Board)
* **Manufacturing:** 3D printing and laser cutting

---

## Repository Structure

```
cdio-scrum-robot/
├─ docs/                # Sprint documentation and reports
├─ cad/                 # CAD files and manufacturing exports
├─ electronics/         # Schematics, wiring, and BOM
├─ firmware/            # Pico firmware and control logic
├─ experiments/         # Calibration and tuning tests
└─ presentations/       # Final slides and demo media
```

---

## How to Run

1. Assemble the hardware according to the wiring diagrams in `electronics/`.
2. Flash the Raspberry Pi Pico with the firmware in `firmware/pico/`.
3. Power the system using the designated battery configuration.
4. Place the robot in a controlled environment with obstacles and a light source.

---

## Evidence & Demonstration

* Photos and videos of sprint deliverables are available in `docs/`.
* Final demonstration videos are available in `presentations/demo-videos/`.

---

## Project Management

All sprint tasks, issues, and progress tracking were managed using **GitHub Projects** following Scrum conventions. Each issue was labelled by sprint, owner, and task type.

---

## License

This project is released under the MIT License and is intended for academic and educational use only.
