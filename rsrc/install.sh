git clone https://github.com/c-krit/ferox
cp -r ferox/ferox/** rsrc
rm -rf ferox
gcc -c -fPIC rsrc/ferox.c -o rsrc/ferox.o
sudo gcc rsrc/ferox.o -shared -o /usr/local/lib/libferox.so -lm
sudo cp /usr/local/lib/libferox.so /usr/lib/libferox.so
sudo ln -s /usr/lib/libferox.so /lib/libferox.so