#!/bin/sh

BSP_PATH="/opt/bsp"
BSP_NAME="eross_v4"

source ${BSP_PATH}/$BSP_NAME/images/linux/sdk/environment-setup-aarch64-xilinx-linux
BSP_ROOT=${BSP_PATH}/$BSP_NAME/images/linux/sdk/sysroots/aarch64-xilinx-linux

mkdir -p build_petalinux
cd build_petalinux
cmake .. -DCMAKE_C_COMPILER="aarch64-xilinx-linux-gcc" \
      -DCMAKE_CXX_COMPILER="aarch64-xilinx-linux-g++" \
      -DBUILD_SYSROOT="${BSP_ROOT}" \
      -DBUILD_EMBEDDED=ON \
      -DBUILD_DDEBUG=ON
make -j4
