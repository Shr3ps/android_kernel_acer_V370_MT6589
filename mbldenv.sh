#!/bin/bash
# ##########################################################
# ALPS(Android4.1 based) build environment profile setting
# ##########################################################
# Overwrite JAVA_HOME environment variable setting if already exists
export JAVA_HOME=/home/doha/jdk1.7.0_60


# Overwrite ANDROID_JAVA_HOME environment variable setting if already exists
export ANDROID_JAVA_HOME=/home/doha/jdk1.7.0_60


# Overwrite PATH environment setting for JDK & arm-eabi if already exists
export PATH=/home/doha/jdk1.7.0_60/bin:$PWD/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.6/bin:$PATH

# Add MediaTek developed Python libraries path into PYTHONPATH
if [ -z "$PYTHONPATH" ]; then
  PYTHONPATH=$PWD/mediatek/build/tools
else
  PYTHONPATH=$PWD/mediatek/build/tools:$PYTHONPATH
fi
export PYTHONPATH

