cd ~/flycron/mcu
cargo build --features crystal-25mhz --release
arm-none-eabi-objcopy -O binary ../target/thumbv7em-none-eabihf/release/flycron_mcu /tmp/flycron-25.bin
cargo build --features crystal-8mhz --release
arm-none-eabi-objcopy -O binary ../target/thumbv7em-none-eabihf/release/flycron_mcu /tmp/flycron-8.bin

stty -F /dev/serial/by-id/usb-Armchair_Heavy_Industries_Flycron_44003E000151333138363230-if00 1200
sleep 1
sudo dfu-util -D /tmp/flycron-8.bin -a 0 -R -s 0x08000000:leave
sleep 1
stty -F /dev/serial/by-id/usb-Armchair_Heavy_Industries_Flycron_1F004E001351303239373335-if00 1200
sleep 1
sudo dfu-util -D /tmp/flycron-25.bin -a 0 -R -s 0x08000000:leave
sleep 1
stty -F /dev/serial/by-id/usb-Armchair_Heavy_Industries_Flycron_45006C000151333138363230-if00 1200
sleep 1
sudo dfu-util -D /tmp/flycron-8.bin -a 0 -R -s 0x08000000:leave
sleep 1
stty -F /dev/serial/by-id/usb-Armchair_Heavy_Industries_Flycron_490065000151333138363230-if00 1200
sleep 1
sudo dfu-util -D /tmp/flycron-8.bin -a 0 -R -s 0x08000000:leave
sleep 1

