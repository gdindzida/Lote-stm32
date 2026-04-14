# Lote-stm32
learn on the edge - stm32g431cbu6 open source hil

## Setup repository 
Install following vscode extensions: clangd, pylance, c/c++
Add to include path in workspace vscode settings: ${workspaceFolder}/modules/libopencm3/include/**

```
sudo apt install gcc-arm-none-eabi cmake ninja-build dfu-util clangd git
```

```
git submodule update --init --recursive
cd firmware/modules/libopencm3
make TARGETS=stm32/g4
cd ../gaspar
./setup_requirements.sh
cd ../..
```

To run gaspar it is necessary to activate venv
```
source venv/bin/activate
```
To deactivete
```
deactivate
```

## Build

configure
```
cmake -B build-debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=tools/arm-none-eabi-toolchain.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake --build build
```

## Update submodules

```
git submodule update --init --recursive

## Tools
Downloaded from: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
```

## Put board in dfu mode
Hold BOOT
Press RESET
Release RESET
Release BOOT

```
dfu-util -l
```

## flash
```
dfu-util -a 0 -s 0x08000000:leave -D firmware.bin
```
