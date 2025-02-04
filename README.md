# HoverCar

This repository showcases an electric “HoverCar” project, built by repurposing two hoverboards and integrating them with custom electronics and firmware. **Disclaimer:** The code in this repo is tailored to my personal setup. While you can explore or reuse parts of it, please note that it was never designed to be a general plug-and-play solution.

## Table of Contents
1. [Project Overview](#project-overview)
2. [Background and Motivation](#background-and-motivation)
3. [Initial Experiments: The Zeus Car](#initial-experiments-the-zeus-car)
4. [Scaling Up: Tank Chassis](#scaling-up-tank-chassis)
5. [Final Stage: The HoverCar](#final-stage-the-hovercar)
6. [Electronics & Components](#electronics--components)
7. [Software & Control](#software--control)
8. [Future Plans](#future-plans)
9. [Media & Demonstrations](#media--demonstrations)
10. [License](#license)

---

## Project Overview

The **HoverCar** began as a quest to build something bigger and more functional than standard hobbyist robot kits. After experimenting with small cars and tank chassis, I moved on to repurposing hoverboard parts due to their powerful motors, controllers, and batteries—all available at a relatively low cost on the secondhand market.

> **Key takeaway:** The knowledge I gained from smaller projects (Zeus car and tank chassis) carried over to this large-scale build, though I still encountered unexpected challenges in firmware flashing, wiring, signal conversion, and more.

## Background and Motivation

I'm a student of cybernetics and robotics who loves building practical projects to supplement theoretical knowledge. This project is a playground to learn more about motor control, firmware development, sensor integration, and wireless communication—while also having a blast driving around on a custom electric vehicle.

## Initial Experiments: The Zeus Car

I started with a mini-robot known as the **Zeus Car**, which came as a complete kit: battery, wheels, sensors, and an Arduino UNO R3.

- **Sensors** included: grayscale, camera, ultrasound, etc.
- **Customization:** I replaced the default app control with my own Python script, allowing me to drive it via a PS5 controller (using libraries like `pygame`).
- **Goal:** Learn how to manipulate sensor data and motor output in real-time. I succeeded in reading sensor data on my computer while controlling the car.
- **Limitations:** The motors (5V) couldn’t carry any significant load, but the project was a great introduction to microcontroller basics (PWM signals, RX/TX communication, battery management, and general wiring).

> **Pictures:** Insert any photos of the Zeus Car here showing wiring, modifications, or the controller setup.

## Scaling Up: Tank Chassis

Next, I bought a **tank chassis** equipped with 12V motors. This allowed me to explore more powerful driver components.

- **Motor drivers:** Two BTS7960 drivers to interface between the Arduino’s 5V PWM signals and the 12V supply.
- **Challenges:**
  - Interpreting the datasheet for correct wiring.
  - **Crucial Lesson:** A common ground (GND) is absolutely necessary.
  - Lots of soldering and troubleshooting to ensure the motor drivers responded correctly.

> **Pictures:** Insert tank chassis images here, highlighting motor drivers and wiring.


## Final Stage: The HoverCar

The leap from a tank chassis to a hoverboard-based design wasn’t trivial, but most of the microcontroller logic was transferable:

1. **Sourcing Hoverboards:** Found two used hoverboards for under $30 each. Their motors and built-in motor controllers were ideal, but were typically locked behind proprietary firmware.
2. **Firmware Flashing:**
   - Successfully flashed custom firmware on one hoverboard controller.
   - Had issues flashing the second, so I purchased two generic brushless motor controllers (18–55V) off AliExpress.
3. **Potentiometer vs. UART:**
   - The new controllers were potentiometer-based.
   - I needed an **Arduino Mega** for multiple UART (RX/TX) pins because each hoverboard side needed its own communication channel.
   - **DAC Setup:** The controllers expected analog signals, so I needed a digital-to-analog converter to transform PWM output into a smooth analog voltage.
   - I wrestled with address conflicts, soldering missteps, and code adjustments before getting stable motor control.
4. **Wireless Control with ESP32:**
   - I wanted to drive the HoverCar wirelessly via a PS5 controller.
   - Used an **ESP32** for Bluetooth, referencing open-source libraries and examples.
   - Forwarded the controller inputs to the Arduino for the final motor commands.

> **Videos:**

### Frame and Finishing Touches

- **Frame Construction:** Used wooden planks for a base, adding some basic weatherproofing.
- **Driving Experience:** The car can move outdoors, though it’s still evolving.
- **Practical Learning:** Building this forced me to apply many cybernetics/robotics concepts—power distribution, signal processing, sensor feedback, and more.

## Electronics & Components

1. **Arduino Mega** – multiple UART pins, central microcontroller.
2. **Hoverboard Motors & Controllers** – repurposed from used hoverboards.
3. **DACs** – smoothing out PWM signals for the pot-based controllers.
4. **ESP32** – Bluetooth module for wireless control.
5. **Batteries** – integrated hoverboard battery packs (be mindful of their voltage ratings).
6. **Miscellaneous** – BTS7960 drivers (used earlier), power cables, sensors, and more.

## Software & Control

- **Language/IDE:** Mostly C/C++ in the Arduino IDE.
- **Wireless Control:** Python + `pygame` for controller inputs, Bluetooth connectivity via ESP32.
- **Firmware:** Custom code for the flashed hoverboard controllers.
- **Motor Control Logic:** PWM signals, digital-to-analog conversion, plus sensor feedback (planned for future expansions).

## Future Plans

- **Refine Code:** Clean up the repository, optimize signal handling, and reduce wiring complexity.
- **Better Drive Experience:** Improve speed control, implement smoother acceleration, and refine steering.
- **Add Sensors:** Incorporate ultrasound, LiDAR, or camera modules for autonomous features.
- **Robot Arm:** Potentially build and mount a robotic manipulator on top for actual tasks.

## Media & Demonstrations

> **Insert final build photos and driving footage here.** Highlight the mechanical details (motors, battery placement, wiring) and show the car in motion. Short video clips or animated GIFs could be especially helpful.

## License

This project is provided under an open-source license for educational purposes. Feel free to explore and adapt any of the code, but please note that it’s highly specific to my configuration. Use at your own risk!

---

### Acknowledgments

- Thanks to open-source communities whose hoverboard firmware solutions I relied on.
- All the creators of libraries and open-source motor controller code.

**Enjoy exploring the HoverCar project!** If you have questions or suggestions, feel free to open an issue or contact me.

