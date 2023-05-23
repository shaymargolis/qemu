# Running:

```
sudo ./qemu-system-mipsel -S -s -cpu 24Kc -M mt7628an -bios ../flash.bin -serial /dev/ttyS0 -nographic -m 32M
```

# Compiling

```
../qemu/configure --target-list=mipsel-softmmu
```
