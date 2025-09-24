# nrf52840_uart_commands

Zephyr RTOS application for the **nRF52840 DK**.

## Structure
- `src/main.c` – (skeleton)
- `prj.conf` – Zephyr configuration
- `docs/` – documentation (commands, architecture, etc.)

## Build and Flash
```bash
west build -b nrf52840dk_nrf52840 .
west flash