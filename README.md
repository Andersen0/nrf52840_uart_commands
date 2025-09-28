# CDC-ACM LED Control for nRF52840-DK

This project is a USB CDC-ACM (virtual COM port) application for the Nordic **nRF52840-DK**, built and tested with **nRF Connect SDK v3.1.1 (Zephyr RTOS)**.  
It provides a simple text-based command interface over USB to control the development kit’s LEDs.

---

## How to Build and Run

1. **Clone and set up environment**
    ```bash
    west init -m https://github.com/Andersen0/nrf52840_uart_commands.git
    west update
    ```
2. **Build and Flash**
    ```bash
    west build -b nrf52840dk_nrf52840
    west flash
    ```
3. **Connect via USB**
* Plug the nRF52840-DK into your PC via the nRF USB microUSB port.
* A virtual COM port (`/dev/ttyACM0` on Linux, `COMx` on Windows) will appear.
* Open it with your favorite terminal program with a baudrate of 115200:
    ```bash
    screen /dev/ttyACM0 115200
    ```
## Available Commands

All commands are case-insensitive. Prompt ends with `>`.

* `HELP` - Show available commands and usage
* `CLEAR` - Clears the terminal screen and reprints the initial prompt
* `LED <1-4> <ON|OFF>` - Turns a specific LED steady ON or OFF
* `BLINK <1-4> <ms>` - Makes a specific LED blink wih a given period in milliseconds.
    - `<ms> = 0` -> LED stays STEADY ON

## Design Choices
* USB CDC-ACM (UART over USB):

    Builds upon Zephyr's library of samples. Extended to echo to terminal and handle commands as well as on-board LEDs.

* Command parser:

    Implemented with a small `command_table[]` that maps commands to handler functions for easy extension. Controls input validation by tokenizing in a generic layer.

* LED abstraction:

    Uses Zephyr’s gpio_dt_spec bindings for portability across boards.

* Blink thread:

    Separate thread handles LED blinking independently of USB input, keeping the system responsive.

## Use of AI tools:

* `Perplexity`
    - Finding resources
* `ChatGPT`
    - Clarify sample code and Zephyr error messages
    - Suggest command parsing logic and command functions
    - Provide guidance on using threads in Zephyr (e.g., `blink_thread`) 
    - Draft and refine documentation (`README`, `Application messages`)

## Future works

* Add command history and line editing (arrow key navigation, etc.).

* Implement input validation with better error messages.

* Support saving LED states across resets (using flash or settings subsystem).

* Expand commands (e.g., STATUS to report current LED states).

* Optimize USB transactions

* Restructure file format

* More LED effects