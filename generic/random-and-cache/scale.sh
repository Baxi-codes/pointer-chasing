if [ $# -ne 1 ]; then
    echo "Usage: $0 <cpu_frequency>"
    exit 1
fi

CPU_FREQ=$1


for cpu in /sys/devices/system/cpu/cpu*/cpufreq/; do
    echo $CPU_FREQ | sudo tee $cpu/scaling_min_freq
    echo $CPU_FREQ | sudo tee $cpu/scaling_max_freq
done

