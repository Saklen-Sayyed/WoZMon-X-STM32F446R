# WoZMon-X: Bare-Metal Memory Monitor & Debug Utility for STM32F446RE

A low-level UART-based memory monitor inspired by the original **WozMon** used in early microcomputers.  
This firmware allows real-time **read, write, and dump** access to STM32 memory regions through a UART terminal.

---

## ⚙️ Features
- Bare-metal firmware in C (no RTOS)
- Direct access to **SRAM, Flash, and System ROM** regions
- UART command interface supporting:
  - `D` → Dump memory  
  - `E` → Edit/write memory  
  - `H` → Help / memory map  
- Address parsing and hex data decoding
- Register-level configuration of:
  - USART3 (TX/RX)
  - GPIOB, GPIOC
  - TIM2 (1ms tick timer)
  - RCC (clock control)

---

## 🧠 Technologies
- **MCU**: STM32F446RE (ARM Cortex-M4)  
- **Language**: Embedded C  
- **Interfaces**: UART, GPIO, Timer  
- **Tools**: STM32CubeIDE, STM32 HAL, ST-Link

---

## 🧩 Project Structure
