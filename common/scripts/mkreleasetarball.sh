#!/bin/bash
if [ $# -lt 1 ]; then
	echo "Usage: $0 tarball_name [product=zcu102]"
	echo " "
	echo "product   - Android build product name. Default is zcu102"
	echo "            Used to find folder with build results"
	exit -1 ;
fi

if [ $# -ge 2 ]; then
	product=$2;
else
	product=zcu102;
fi

echo "========= build release tarball $product";

if ! [ -d out/target/product/$product/boot ]; then
   echo "!!! Error: Missing out/target/product/$product";
   exit 1;
fi

tar -cvzf $1 device/xilinx/common/scripts/mksdcard.sh \
	out/target/product/$product/boot/ \
	out/target/product/$product/system.img \
