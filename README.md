# FPGA Field-Oriented Control (FOC) Drive

This is a hardware-based Field-Oriented Control (FOC) drive for 3-phase BLDC motors, written entirely in VHDL. It targets the Xilinx Artix-7 (specifically the Arty A7 board) and does everything in the FPGA fabric. There are no soft cores (like MicroBlaze) and no C code running the loops. The math, scheduling, and sensor acquisitions are completely hardcoded into the datapath. 

The system runs on a 100 MHz base clock, driving a 20 kHz SVPWM output, and uses a multirate scheduler to handle the current, velocity, and position control loops seamlessly.

## Hardware Setup
To actually spin a motor with this, you'll need:
*   **FPGA:** Digilent Arty A7 (Artix-7 35T or 100T).
*   **Power Stage:** SimpleFOC Shield v2 (uses the L6234D driver).
*   **Encoder:** AS5048A Magnetic Encoder (talking over SPI).
*   **Motor:** Any standard 3-phase BLDC motor (e.g., a gimbal motor) that fits the SimpleFOC shield limits.

## How It Works (The Datapath)
The whole system is driven by events, not just blindly clocked FSMs. It avoids messy clock domain crossing by running the math on the fast 100 MHz clock and using valid/enable flags to trigger stages.

1.  **Trigger:** The center of the PWM cycle generates a sync tick.
2.  **Acquisition:** That tick triggers the XADC to read the phase currents, and simultaneously fires the SPI master to fetch the rotor angle from the AS5048A.
3.  **Math Pipeline:** Once both sensors return valid data, the data pushes through the FOC math pipeline:
    *   Electrical angle conversion.
    *   CORDIC sine/cosine generation.
    *   Clarke & Park transforms.
    *   PI loops (Current -> Velocity -> Position).
    *   Inverse Park & Clarke transforms.
    *   Space Vector PWM (SVPWM) duty cycle calculation.

## Control Loops & Scheduling
Instead of running every loop at the PWM frequency, there's an outer-loop scheduler that steps down the update rates. This mimics standard DSP motor control architectures:
*   **Current Loop:** Runs at 20 kHz (every PWM tick).
*   **Velocity Loop:** Runs at 4 kHz (every 5th tick).
*   **Position Loop:** Runs at 1 kHz (every 20th tick).

## Cool Architecture Details
If you're poking around the source code, here are a few structural things to note:

*   **Fixed-Point Everything:** Floating point is too expensive. The whole datapath uses strict fixed-point `Q` formats (like `Q3.12` for current, `Q5.12` for voltage, `Q8.20` for velocity).
*   **CORDIC Trig:** Sine and cosine are generated on the fly using the CORDIC algorithm. No giant LUTs or ROMs taking up block RAM.
*   **SVPWM vs Sinusoidal:** Standard sinusoidal PWM wastes DC bus voltage. This design uses Space Vector PWM by calculating phase min/max and injecting a zero-sequence offset, giving you about 15% more top speed out of the same power supply.
*   **2-Shunt Current Sensing:** The XADC only reads Phase A and Phase B. Phase C is calculated mathematically ($C = -(A + B)$) to save an analog channel and speed up conversion time.
*   **DSP48 Forcing:** Multipliers in the FOC transforms (Clarke/Park) have explicit pipeline registers right after them. This is intentional so Vivado absorbs the registers directly into the DSP48 blocks, which fixes routing delays and helps close timing.

## Safety & Noise Immunity
Motors are noisy. FPGAs are sensitive. A few things were built in to keep the drive from destroying itself:
*   **Hardware Watchdogs:** There are timeout counters for both the SPI encoder and the XADC. If either sensor misses 3 consecutive PWM cycles, the hardware latches a fault and immediately revokes the motor enable signal. 
*   **UART Oversampling:** The host UART doesn't just sample once per bit. It oversamples at 16x the baud rate and takes a 3-bit majority vote right in the center of the bit window to ignore EMI spikes.
*   **Continuous CRC:** Every 64-bit UART configuration packet includes a 56-bit CRC-8 hash. If the host sends a bad PID gain due to line noise, the parser silently drops it before it can corrupt the active control loop.
*   **Accumulator Guard Bits:** The PI controllers have extra MSBs (guard bits) in their accumulators to catch math overflows *before* they hit the anti-windup clamping stage. 
*   **Clock Domain Crossing (CDC):** There's a custom async FIFO for cleanly passing data between different clock domains (if you mod the design to use them). It uses Gray code pointers and double-flop synchronizers to prevent metastability.

## Building the Project
Just pull the VHDL files into a standard Vivado RTL project. Set `motor_controller_top.vhd` as the top module. You'll need to write or adapt a constraints file (`.xdc`) to map the SPI, UART, and PWM outputs to the specific PMOD or Arduino headers you are using on the Arty A7. All tuning (PID gains, limits) is done over UART at runtime—the FPGA boots in a safe, disabled state with all gains zeroed out.