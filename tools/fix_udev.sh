#!/bin/sh
# Fix USB para STM32H562 OpenEMS VCP (0483:5740)
# Roda: sudo sh tools/fix_udev.sh

# 1. udev rule: autosuspend off + ModemManager ignore (persiste entre boots)
echo 'ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="5740", ATTR{power/autosuspend_delay_ms}="-1", ATTR{power/control}="on", ENV{ID_MM_DEVICE_IGNORE}="1"' > /etc/udev/rules.d/99-openems.rules
udevadm control --reload-rules 2>/dev/null

# 2. Global autosuspend fallback (imediato, não persiste entre boots)
echo -1 > /sys/module/usbcore/parameters/autosuspend

# 3. Stop ModemManager (interfere com CDC-ACM devices)
systemctl stop ModemManager 2>/dev/null
systemctl disable ModemManager 2>/dev/null

echo "USB fix applied: autosuspend off + ModemManager disabled"
cat /etc/udev/rules.d/99-openems.rules
