
# Install PICO-SDK (or update it)
Use pico-sdk 2.2.0 (for RP2350, pico2)
------------------------------------
git clone -b master https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk/
git submodule update --init
REPLACE tinyusb with latest version(!):
cd lib
mv tinyusb tinyusb_old
git clone https://github.com/hathach/tinyusb.git
mv tinyusb-master tinyusb
cd tinyusb
python3 tools/get_deps.py rp2040

# Export PICO_SDK_PATH (e.g.)
export PICO_SDK_PATH=/Users/jean-marcharvengt/Documents/pico/pico-sdk-2.2.0 (path to pico-sdk!)

# Compile the project
Go to project dir (e.g. MCUME/MCUME_pico2):
mkdir build
cd build
pico2/2w: cmake -DPICO_PLATFORM=rp2350 -DPICO_BOARD=pico2 ..
make
