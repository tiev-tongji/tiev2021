#!/bin/bash


if [ ! -f "/etc/udev/rules.d/50-usbcan.rules" ]
then
    touch /etc/udev/rules.d/50-usbcan.rules
    usbcan='SUBSYSTEMS=="usb", ATTRS{idVendor}=="0471", ATTRS{idProduct}=="1200", GROUP="users", MODE="0666"'
    echo $usbcan > /etc/udev/rules.d/50-usbcan.rules
fi

if [ ! -f "/etc/udev/rules.d/50-remote-control.rules" ]
then
    touch /etc/udev/rules.d/50-remote-control.rules
    remotecontrol='SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="remote_control", MODE="0666"'
    echo $remotecontrol > /etc/udev/rules.d/50-remote-control.rules
fi

if [ ! -f "/etc/udev/rules.d/50-huace.rules" ]
then
    touch /etc/udev/rules.d/50-huace.rules
    huace='ACTION=="add", KERNELS=="16-2:1.0", SUBSYSTEMS=="usb", GROUP="users", MODE="0666", SYMLINK+="huace"'
    Ntrip='ACTION=="add", KERNELS=="16-1:1.0", SUBSYSTEMS=="usb", GROUP="users", MODE="0666", SYMLINK+="Ntrip"'
    echo $huace > /etc/udev/rules.d/50-huace.rules
    echo $Ntrip > /etc/udev/rules.d/50-huace.rules
fi

udevadm control --reload-rules
# replug the devices