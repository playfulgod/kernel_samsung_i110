echo "Taring zImage"
tar -H ustar -c zImage > zImage.tar
md5sum -t zImage.tar >> zImage.tar
mv zImage.tar zImage.tar.md5

echo
echo "Finished"
echo
