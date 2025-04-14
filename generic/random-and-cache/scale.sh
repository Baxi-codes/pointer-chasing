#!/bin/bash

if [ $# -ne 1 ]; then
    echo "Usage: $0 <cpu_frequency>"
    exit 1
fi

CPU_FREQ=$1


for cpu in /sys/devices/system/cpu/cpu*/cpufreq/; do
    echo 800000 | sudo tee $cpu/scaling_min_freq
    echo 800000 | sudo tee $cpu/scaling_max_freq
done

echo $CPU_FREQ | sudo tee /sys/devices/system/cpu/intel_uncore_frequency/package_00_die_00/min_freq_khz
echo $CPU_FREQ | sudo tee /sys/devices/system/cpu/intel_uncore_frequency/package_00_die_00/max_freq_khz

