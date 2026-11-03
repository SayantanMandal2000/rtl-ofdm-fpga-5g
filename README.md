# FPGA-Based OFDM Communication System with 5G Phase Compensation and Chaos Encryption

An FPGA-oriented Orthogonal Frequency Division Multiplexing (OFDM) communication system implemented in synthesizable Verilog-2001. The project integrates 5G NR phase compensation, Lorenz-based digital chaos encryption, and training-sequence-based timing synchronization into a modular OFDM transmitter and receiver.

The design is verified through RTL simulation using a self-checking testbench and channel model.

## Overview

OFDM is widely used in modern wireless communication systems such as Wi-Fi, LTE, and 5G NR. It transmits data over multiple orthogonal subcarriers, providing efficient spectrum utilization and robustness against multipath fading.

This project implements a complete baseband OFDM transceiver with:

- 128-point FFT/IFFT processing
- 16-QAM modulation and demodulation
- 32-sample cyclic prefix
- 5G NR-style 14-symbol slot structure
- Digital chaos-based data encryption
- Per-symbol phase compensation
- Training-sequence-based timing synchronization
- AXI4-Lite configuration interface
- BER and EVM-based performance verification

## Key Features

### 1. OFDM Transmitter

The transmitter generates OFDM symbols through the following processing stages:

1. PRBS-15 data generation
2. XOR-based chaos encryption
3. Gray-coded 16-QAM mapping
4. Key-dependent constellation rotation
5. 128-point IFFT
6. Cyclic-prefix insertion
7. 5G NR phase compensation
8. Valid-qualified I/Q output streaming

Each slot contains one training symbol followed by 13 data symbols.

### 2. OFDM Receiver

The receiver recovers the transmitted data through:

1. Training-sequence detection and timing synchronization
2. Cyclic-prefix removal
3. 128-point FFT processing
4. Chaotic constellation de-rotation
5. 16-QAM demapping
6. XOR-based chaos decryption
7. PRBS-based bit-error-rate measurement

### 3. 5G NR Phase Compensation

The system supports per-symbol phase correction using 14 complex phase coefficients per slot.

- Coefficients are configured through an AXI4-Lite register interface.
- An ARM processor can write the phase coefficients to the FPGA.
- A symbol-index tracking mechanism selects the corresponding coefficient.
- A pipelined complex multiplier applies phase correction to the transmitted I/Q samples.

The coefficients use Q2.14 fixed-point representation.

### 4. Chaos-Based Encryption

A fixed-point Lorenz chaotic system generates a pseudorandom key stream for physical-layer encryption.

- Lorenz system implemented using fixed-point arithmetic.
- Q12.20 state representation.
- 64-bit key-based initialization.
- FIFO-buffered key-stream generation.
- XOR encryption of data bits.
- Key-dependent 90° constellation rotations.
- Matching key-stream generation at the receiver for decryption.

A receiver with an incorrect key produces a high BER, demonstrating key sensitivity in the simulated system.

### 5. Timing Synchronization

The receiver uses a conjugate-symmetric training sequence for OFDM frame detection.

The synchronization architecture includes:

- Sliding-window correlation
- Signal-energy accumulation
- Timing-metric calculation
- Threshold-based frame detection
- Peak confirmation
- Receiver lock generation
- Data buffering for timing recovery

The synchronizer identifies the training symbol and determines the start of the OFDM data symbols.

## System Parameters

| Parameter                 | Value             |
| ------------------------- | ----------------- |
| FFT/IFFT size             | 128 points        |
| Cyclic prefix             | 32 samples        |
| Modulation                | Gray-coded 16-QAM |
| Symbols per slot          | 14                |
| Training symbols per slot | 1                 |
| Data symbols per slot     | 13                |
| I/Q sample width          | 16-bit signed     |
| System clock              | 100 MHz           |
| OFDM slot duration        | 0.5 ms            |
| Frame duration            | 10 ms             |
| Phase coefficient format  | Q2.14             |
| Chaos state format        | Q12.20            |

## System Architecture

### Transmitter

```text
PRBS-15 Data Source
        |
        v
Chaos XOR Encryption
        |
        v
16-QAM Mapper
        |
        v
Chaos-Based Constellation Rotation
        |
        v
128-Point IFFT
        |
        v
Cyclic Prefix Insertion
        |
        v
5G NR Phase Compensation
        |
        v
       TX I/Q
```

### Receiver

```text
      RX I/Q
        |
        v
Timing Synchronization
        |
        v
Cyclic Prefix Removal
        |
        v
128-Point FFT
        |
        v
Chaos-Based De-Rotation
        |
        v
16-QAM Demapper
        |
        v
Chaos XOR Decryption
        |
        v
PRBS BER Checker
```

## RTL Module Description

The design is organized into modular Verilog blocks.

| Module              | Description                                                         |
| ------------------- | ------------------------------------------------------------------- |
| `ofdm_top.v`        | Top-level OFDM transceiver and AXI4-Lite interface                  |
| `ofdm_tx.v`         | OFDM transmitter                                                    |
| `ofdm_rx.v`         | OFDM receiver                                                       |
| `fft_core.v`        | Custom radix-2 DIT FFT/IFFT core                                    |
| `twiddle_rom.v`     | FFT twiddle-factor lookup table                                     |
| `qam16.v`           | 16-QAM mapper and demapper                                          |
| `cmult.v`           | Complex multiplication                                              |
| `lorenz_chaos.v`    | Fixed-point Lorenz chaotic system                                   |
| `chaos_keystream.v` | Chaos key-stream generator                                          |
| `sync_fifo.v`       | Synchronous FIFO for key-stream buffering                           |
| `phase_comp_core.v` | Per-symbol phase compensation                                       |
| `timing_sync.v`     | Training-sequence-based timing synchronization                      |
| `ofdm_axil_regs.v`  | AXI4-Lite control and status registers                              |
| `tb_ofdm_top.v`     | Self-checking system testbench                                      |
| `channel_model.v`   | Simulated channel with phase offsets, noise, and timing uncertainty |

## AXI4-Lite Configuration

The AXI4-Lite register interface provides control and monitoring capabilities.

| Offset      | Register  | Function                                               |
| ----------- | --------- | ------------------------------------------------------ |
| `0x00`      | CTRL      | TX enable, phase compensation, chaos enable, BER clear |
| `0x04`      | STATUS    | Receiver lock status                                   |
| `0x08–0x0C` | TX_KEY    | 64-bit transmitter chaos key                           |
| `0x10–0x14` | RX_KEY    | 64-bit receiver chaos key                              |
| `0x18`      | BER_BITS  | Number of checked bits                                 |
| `0x1C`      | BER_ERRS  | Number of bit errors                                   |
| `0x20`      | RX_SLOTS  | Number of synchronized slots                           |
| `0x24`      | SYNC_PEAK | Last synchronization metric peak                       |
| `0x40–0x74` | PC_COEF   | 14 phase compensation coefficients                     |

The phase compensation coefficients are written as complex values containing cosine and sine components.

## Simulation and Verification

The design was verified using a self-checking testbench and a channel model.

The simulated channel introduces:

- Per-symbol carrier phase rotation
- Additive I/Q noise
- Unknown signal arrival time
- Noise-only samples before the OFDM frame

Four test scenarios were evaluated:

| Test | Phase Compensation | Chaos    | Receiver Key | Expected Behavior        |
| ---- | ------------------ | -------- | ------------ | ------------------------ |
| 1    | Disabled           | Enabled  | Correct      | Phase-induced errors     |
| 2    | Enabled            | Enabled  | Correct      | Successful data recovery |
| 3    | Enabled            | Enabled  | Incorrect    | High BER                 |
| 4    | Enabled            | Disabled | Not required | Successful data recovery |

## Simulation Results

| Test Case              | Bits Checked | Bit Errors |  BER |   EVM |
| ---------------------- | -----------: | ---------: | ---: | ----: |
| Phase compensation OFF |       13,280 |      6,115 | 0.46 | 25.0% |
| Phase compensation ON  |       19,936 |          0 |    0 |  1.2% |
| Incorrect receiver key |       13,280 |      6,845 | 0.52 |  1.2% |
| Chaos disabled         |       13,280 |          0 |    0 |  1.2% |

### Key Observations

- Phase compensation reduced EVM from 25% to 1.2%.
- Zero bit errors were observed across 19,936 checked bits with phase compensation enabled and the correct key.
- A one-bit receiver-key mismatch resulted in approximately 50% BER.
- The timing synchronizer achieved metric peaks of 122–126 against an ideal peak of 126.
- The receiver successfully synchronized across all tested slots.
- Cyclic-prefix insertion and training-sequence conjugate symmetry were verified.

**Note:** These results are obtained from RTL simulation. Physical FPGA board-level validation and hardware timing closure are future work.

## Tools and Technologies

- Verilog-2001
- Xilinx Vivado
- Vivado XSim
- FPGA-based digital signal processing
- AXI4-Lite
- FFT/IFFT
- 16-QAM
- Fixed-point arithmetic
- Digital chaos
- OFDM synchronization
- RTL simulation and verification

## Future Improvements

- Synthesize and implement the design on a Zynq-7020 FPGA.
- Perform hardware loopback testing.
- Replace the sequential FFT core with a pipelined FFT implementation for higher throughput.
- Add multipath channel effects and carrier-frequency offset.
- Implement channel estimation and equalization.
- Evaluate the statistical properties of the generated chaos key stream.
- Improve the encryption seed using slot and frame counters.

## References

1. Z. Tang, “OFDM Communication System Based on FPGA,” IEEE ACCTCS, 2023.
2. H. Nguyen and S. Nguyen, “FPGA-Based Implementation and Evaluation of Real-Time OFDM Phase Compensation in 5G,” ATC, 2021.
3. Y. Wang, X. Yang, and W. Hu, “FPGA Implementation of Real-Time Secure OFDM Transmission Using Digital Chaos,” ICTON, 2018.
4. X. Na and S. Gao, “An FPGA Implementation Method of OFDM Timing Synchronization Based on Conjugated-Symmetrical-Structured Training Sequence,” IEEE MAPE, 2013.
5. 3GPP TS 38.211, NR; Physical Channels and Modulation.

## Author

**Sayantan Mandal**
RTL Design Engineer | Arm, Bengaluru

**Riya Das**
Design Project | BITS Pilani, Hyderabad Campus

---

If you find this project useful, feel free to explore the RTL implementation and simulation files in this repository.
