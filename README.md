# STM32_BMML

A small bare-metal peripheral library for the **STM32F446RE** (Cortex-M4F), written in C11 on top of CMSIS register definitions, with no HAL or LL.

> 🚧 **Work in progress.** The API is not stable and some drivers still have known issues (see [Status](#status)). I'm building this to learn the hardware properly: DMA, interrupts and peripheral state machines.

## Design

- **Resource layer** (`Core/Resources/`): per-peripheral tables that map each instance to its RCC bit, IRQ and DMA stream/channel.
- **Acquire/release pattern:** every driver owns a static pool of handles. `*_acquire()` claims a peripheral and returns a handle, and `*_release()` frees it. A busy peripheral returns `BMML_BUSY`.
- **Shared DMA dispatcher** (`bmml_dma_shared.c`): drivers claim a DMA stream and register a callback with a context pointer, so one set of `DMAx_StreamN_IRQHandler`s serves the whole library.
- **Shared IRQ for TIM6/DAC:** one vector, split between the timer and DAC parts.
- **Unified status codes** via `bmml_status_t`.
- **Ring buffer** (power-of-two size, mask-based indexing) used by the USART driver.

## Drivers

| Driver | Features | State |
|---|---|---|
| GPIO | init, alternate function, atomic set/reset, EXTI with callbacks | usable |
| Timers | basic timers (TIM6/7), PWM on general-purpose and advanced timers | usable, being refined |
| USART | interrupt TX/RX, DMA TX, DMA RX with IDLE detection, `usart_printf` | in progress |
| SPI | master mode, interrupt and DMA transfers | in progress |
| I2C | master mode, interrupt and DMA, repeated start, retries | in progress |
| ADC | continuous/scan mode with DMA | in progress |
| DAC | waveform output via TIM6 trigger and DMA | in progress |

## Project layout

```
Core/
  Inc/          shared headers (status codes, DMA, ring buffer, utilities)
  Resources/    peripheral resource tables
  Src/          shared implementation, main.c
Drivers/
  GPIO/ TIMERS/ USART/ SPI/ I2C/ ADC/ DAC/
CMakeLists.txt
```

## Build

Requirements: `arm-none-eabi-gcc`, CMake ≥ 3.22. The project also needs the CMSIS headers, the startup file (`startup_stm32f446xx.s`) and the linker script (`Core/STM32F446XX_FLASH.ld`) from your STM32 package.

```sh
cmake -B build
cmake --build build
```

The build produces an ELF and a `.bin`, and prints the memory usage. Flash it with your usual tool, for example:

```sh
st-flash write build/STM32_BMML.bin 0x8000000
```

## Example

```c
// USART2 on PA2 (TX) / PA3 (RX), AF7
gpio_t tx = { GPIOA, 2 }, rx = { GPIOA, 3 };
gpio_init(&tx, GPIO_MODE_AF, GPIO_PULL_NONE, GPIO_SPEED_HIGH, GPIO_OTYPE_PP);
gpio_init(&rx, GPIO_MODE_AF, GPIO_PULL_UP,   GPIO_SPEED_HIGH, GPIO_OTYPE_PP);
gpio_set_alternate_func(&tx, 7);
gpio_set_alternate_func(&rx, 7);

usart_t *uart;
usart_acquire(USART2, 115200, NULL, &uart);
usart_printf(uart, "Hello from STM32_BMML\r\n");
```

## Status

This is an active work in progress. Known issues and planned work:

- [ ] Fix the I2C state machine for 1-byte and 2-byte reads and for DMA completion
- [ ] Complete ADC DMA setup (buffer length, `PAR`/`M0AR`/`NDTR`)
- [ ] Review SPI DMA/IT completion paths and the SPI4 DMA mapping
- [ ] Event types in callbacks (done / error / RX idle)
- [ ] Consistent NVIC priority policy
- [ ] Clock configuration (PLL) in `main.c`
- [ ] Hardware tests: USART loopback, SPI MISO↔MOSI, I2C with a sensor/EEPROM
- [ ] DAC second channel, ADC timer trigger mode

## License

TBD
