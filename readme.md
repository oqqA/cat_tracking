### Info

This is the template for projects on STM32F411 board

### Dependencies
- download [Rust compiler](https://www.rust-lang.org/tools/install)

 > for Windows users need [Microsoft C++ Build Tools](https://visualstudio.microsoft.com/ru/thank-you-downloading-visual-studio/?sku=BuildTools)


- download [dfu-util](https://sourceforge.net/projects/dfu-util/)
- run command:
    ```console
    > python3 scripts/first_start.py
    ```

### Instruction (build and upload)



1. Hold the button BOOT0
2. Connect STM32F411 to PC
3. Release the button BOOT0
4. Run command:
``` console
> python3 scripts/run.py
```

### Detals
based on: https://github.com/rust-embedded/cortex-m-quickstart

examples: https://github.com/stm32-rs/stm32f4xx-hal/tree/master/examples
or https://github.com/stm32-rs/stm32f4xx-hal/tree/cargo-bloat/examples

hal docs: https://docs.rs/stm32f4xx-hal/0.8.3/stm32f4xx_hal/

book: https://rust-embedded.github.io/book