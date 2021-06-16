import os

os.system("cargo build --release")
os.system("cargo objcopy --bin stm32f411 --target thumbv7em-none-eabihf --release -- -O binary stm32f411.bin")
# os.system("st-flash erase && st-flash write stm32f411.bin 0x8000000")
os.system("dfu-util -a 0 -s 0x08000000:leave -D stm32f411.bin")
os.remove("stm32f411.bin")