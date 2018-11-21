#!/bin/bash

if [ $# -lt 1 ]; then
	echo "Usage: $0 /dev/diskname [product=zcu102]"
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

echo "========= build SD card for product $product";

if ! [ -d out/target/product/$product/boot ]; then
   echo "!!! Error: Missing out/target/product/$product";
   exit 1;
fi

removable_disks() {
	for f in `ls /dev/disk/by-path/* | grep -v part` ; do
		diskname=$(basename `readlink $f`);
		type=`cat /sys/class/block/$diskname/device/type` ;
		size=`cat /sys/class/block/$diskname/size` ;
		issd=0 ;
		# echo "checking $diskname/$type/$size" ;
		if [ $size -ge 3906250 ]; then
			if [ $size -lt 62500000 ]; then
				issd=1 ;
			fi
		fi
		if [ "$issd" -eq "1" ]; then
			echo -n "/dev/$diskname ";
			# echo "removable disk /dev/$diskname, size $size, type $type" ;
			#echo -n -e "\tremovable? " ; cat /sys/class/block/$diskname/removable ;
		fi
	done
	echo;
}
diskname=$1
removables=`removable_disks`

for disk in $removables ; do
	echo "Found available removable disk: $disk" ;
	if [ "$diskname" = "$disk" ]; then
		matched=1 ;
		break ;
	fi
done

if [ -z "$matched" -a -z "$force" ]; then
	if [ "${diskname:5:6}" == "mmcblk" ]; then
		echo "mmcblk not seen as removable but will try it anyway"
	else
		echo "Invalid disk $diskname" ;
		exit -1;
	fi
fi

prefix='';

if [[ "$diskname" =~ "mmcblk" ]]; then
	prefix=p
fi

echo "reasonable disk $diskname, partitions ${diskname}${prefix}1..." ;

umount ${diskname}${prefix}*

echo "========= creating partition table"
parted -s ${diskname} mklabel msdos

echo "========= creating BOOT partition"
parted -s --align=optimal ${diskname} mkpart primary 4MiB 132MiB

echo "========= creating ROOT partition"
parted -s --align=optimal ${diskname} mkpart primary 132MiB 260MiB

# Making extended partition. Reserve 3GiB. 
# It will contain system and cache partitions for now.
# Additional misc. partitions (vendor, misc) should be placed
# on this extended partition after cache.
parted -s --align=optimal ${diskname} mkpart extended 260MiB 3332MiB

echo "========= creating SYSTEM partition"
parted -s --align=optimal ${diskname} mkpart logical 264MiB 2312MiB

echo "========= creating CACHE partition"
parted -s --align=optimal ${diskname} mkpart logical 2316MiB 2828MiB

echo "========= creating DATA partition"
parted -s --align=optimal ${diskname} mkpart primary 3332MiB 100%

sync
sleep 1

for n in `seq 1 6` ; do
	if ! [ -e ${diskname}${prefix}$n ] ; then
		echo "!!! Error: missing partition ${diskname}${prefix}$n" ;
		exit 1;
	fi
	sync
done

echo "========= formating BOOT partition"
mkfs.vfat -F 32 -n BOOT ${diskname}${prefix}1

echo "========= formating ROOT partition"
mkfs.ext4 -F -L ROOT ${diskname}${prefix}2

echo "========= formating SYSTEM partition"
mkfs.ext4 -F -L SYSTEM ${diskname}${prefix}5

echo "========= formating CACHE partition"
mkfs.ext4 -F -L CACHE ${diskname}${prefix}6

echo "========= formating DATA partition"
mkfs.ext4 -F -L DATA ${diskname}${prefix}4


echo "========= populating BOOT partition"
if [ -e ${diskname}${prefix}1 ]; then
	mkdir -p /tmp/$$/boot_part
	mount -t vfat ${diskname}${prefix}1 /tmp/$$/boot_part
	cp -rfv out/target/product/$product/boot/BOOT* /tmp/$$/boot_part/
	cp -rfv out/target/product/$product/boot/kernel /tmp/$$/boot_part/Image
	cp -rfv out/target/product/$product/boot/*.dtb /tmp/$$/boot_part/
	cp -rfv out/target/product/$product/boot/*.bit /tmp/$$/boot_part/
	cp -rfv out/target/product/$product/boot/uEnv.txt /tmp/$$/boot_part/uEnv.txt
	cp -rfv out/target/product/$product/boot/uramdisk.img /tmp/$$/boot_part/uramdisk.img
	sync
	umount /tmp/$$/boot_part
	rm -rf /tmp/$$/boot_part
else
   echo "!!! Error: missing BOOT partition ${diskname}${prefix}1";
   exit 1
fi

echo "========= populating SYSTEM partition"
if [ -e ${diskname}${prefix}5 ]; then
	sudo dd if=out/target/product/$product/system.img of=${diskname}${prefix}5
	sudo e2label ${diskname}${prefix}5 SYSTEM
	sudo e2fsck -f ${diskname}${prefix}5
	sudo resize2fs ${diskname}${prefix}5
else
	echo "!!! Error: missing SYSTEM partition ${diskname}${prefix}5";
	exit 1
fi
