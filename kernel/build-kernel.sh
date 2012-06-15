make mrproper -j16
rm -rf ../package/modules/*.ko
rm -f ../package/zImage.tar.md5
rm -f ../package/zImage

make viper_usa_defconfig && make -j16

# These move files to easier locations
echo
echo "Copying kernel modules to package/modules"
echo
find -name '*.ko' -exec cp -av {} ../package//modules/ \;
cp arch/arm/boot/zImage ../package/zImage

echo
echo "Finished compiling kernel"
echo
