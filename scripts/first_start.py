import os

os.system("cargo install cargo-binutils")
os.system("rustup component add llvm-tools-preview")
os.system("rustup target add thumbv7em-none-eabihf")
os.system("cargo build --release")