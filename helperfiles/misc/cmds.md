
https://elinux.org/BeagleBoardUbuntu#eMMC:_BeagleBone_Black
##
i2c working

i2cdetect -y -r 0
i2cdetect -y -r 2   ..................  WORKS
/sys/class/i2c-dev/i2c-2 :: dev  device  name  power  subsystem  uevent


### OVERLAYS to use
BB-RELAY-4PORT-00A0.dts
BB-PWM0-00A0.dts



# deprecated :::/sys/devices/platform/bone_capemgr
############# PINS

/* the pin header uses */
		"P9.15",	/* gpio1_16 */ gpio_48
		"P9.23",	/* gpio1_17 */ gpio_49
		"P9.12",	/* gpio1_28 */ gpio_60
		"P9.27",	/* gpio3_19 */ gpio_115
		/* the hardware ip uses */
		"gpio1_16",
		"gpio1_17",
		"gpio1_28",
"gpio3_19";
cd /sys/class/gpio
echo 48 > export
echo 49 > export
echo 60 > export
echo 115 > export


cd /sys/class/gpio/gpio117
valuw and direction

######### PWM ############

cd /sys/class/pwm/pwmchip0
echo 0 > export
echo 1 > export

cd pwm-0:0 or cd pwm-0:1 
cd /sys/class/pwm/pwm0:0
# set
echo 0 > polarity 
echo 20000000 > period 
echo 1 > enable

# pwm control
echo 1000000 > duty_cycle 
echo 1 > enable

echo 2000000 > duty_cycle 
echo 1 > enable

##################################

EHRPWM0A = P9_22
EHRPWM0B = P9_21
EHRPWM1A = P9_14
EHRPWM1B = P9_16
EHRPWM2A = P8_19
EHRPWM2B = P8_13

Page 184 of the TI AM335x and AMIC110 Sitara Processors Technical Reference Manual gives the memory map for the PWM chips:

    PWM Subsystem 0: 0x48300000

        eCAP0: 0x48300100
        ePWM0: 0x48300200

    PWM Subsystem 1: 0x48302000

        eCAP1: 0x48302100
        ePWM1: 0x48302200

    PWM Subsystem 2: 0x48304000

        eCAP2: 0x48304100
        ePWM2: 0x48304180

The address of each PWM interface (posted in the question) contains the hardware address. Matching these addresses gives us:

    EHRPWM0 (ePWM0) is pwmchip1
    EHRPWM1 (ePWM1) is pwmchip3
    EHRPWM2 (ePWM2) is pwmchip6
    ECAP0 (eCAP0) is pwmchip0

Each EHRPWM chip has two PWM output channels, thus the A and B variants. They are exported by echoing a 0 or 1 to export. These channels must use the same frequency, but can have a different duty cycle.

Therefore, given this interface configuration, EHRPWM0A and EHRPWM0B are located at:

root@beaglebone:~# cd /sys/class/pwm/pwmchip1
root@beaglebone:/sys/class/pwm/pwmchip1# ls
device  export  npwm  power  subsystem  uevent  unexport

To export EHRPWM0A (P9_22):

root@beaglebone:/sys/class/pwm/pwmchip0# echo 0 > export
root@beaglebone:/sys/class/pwm/pwmchip0# ls
device  export  npwm  power  pwm-1:0  subsystem  uevent  unexport

To export EHRPWM0B (P9_21):

root@beaglebone:/sys/class/pwm/pwmchip0# echo 1 > export
root@beaglebone:/sys/class/pwm/pwmchip0# ls
device  export  npwm  power  pwm-1:1  subsystem  uevent  unexport

Note: the list of available PWM interfaces and addresses may differ from the list provided in the question, but this method will still work to determine the final pin map.

####################### OPENCV
cmake ..
cmake -LA | awk '{if(f)print} /-- Cache values/{f=1}'


cmake -DCMAKE_BUILD_TYPE=RELEASE \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DINSTALL_PYTHON_EXAMPLES=OFF\
      -DINSTALL_C_EXAMPLES=OFF \
      -DINSTALL_PYTHON_EXAMPLES=ON 
      -DBUILD_opencv_python2=ON \
      -DBUILD_opencv_python_bindings_generator=ON \
      -DPYTHON2_EXECUTABLE=/usr/bin/python2.7 \
      -DPYTHON2_INCLUDE_DIR = /usr/include/python2.7 \
      -DPYTHON2_LIBRARY = /usr/lib/arm-linux-gnueabihf/libpython2.7.so \
      -DPYTHON2_PACKAGES_PATH = lib/python2.7/dist-packages\
      -DPYTHON2_NUMPY_INCLUDE_DIRS = /usr/lib/python2.7/dist-packages/numpy/core/include/ \
      -DBUILD_EXAMPLES=OFF ..