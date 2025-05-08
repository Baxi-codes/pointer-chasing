#!/bin/bash

if [[ $# -ne 3 ]]; then
  echo "Usage: $0 <min_freq_kHz> <max_freq_kHz> <step_kHz>"
  exit 1
fi
min_freq=$1
max_freq=$2
step=$3
runs=5
outfile="ubench_latency_vs_freq.csv"

sizes=(12 368 6144 4000000)
levels=("L1" "L2" "L3" "DRAM")

# Header for CSV
echo "frequency_hz,size_kb,level,throughput_ma_s,latency_s" > "$outfile"

# Function to get median
median() {
  sorted=($(printf "%s\n" "$@" | sort -n))
  mid=$((runs / 2))
  echo "${sorted[$mid]}"
}

# Loop through frequencies
for ((freq=min_freq; freq<=max_freq; freq+=step)); do
  echo "Setting CPU frequency to $freq kHz"
  #if (( freq <= 2700000 )); then
  #  sudo cpupower frequency-set -g userspace > /dev/null
  #  sudo cpupower frequency-set -f ${freq} > /dev/null
  #else
  sudo ./scale.sh "$freq"
  #fi

  for i in "${!sizes[@]}"; do
    size=${sizes[$i]}
    level=${levels[$i]}
    echo "  Running ./ubench-x64 $size ($level)"

    results=()
    for ((j=0; j<runs; j++)); do
      val=$(./ubench-x64 "$size" | awk '/MOV/ {print $2}')
      results+=("$val")
      likwid-memsweeper > /dev/null
    done

    med=$(median "${results[@]}")
    latency=$(echo "scale=15; 1 / (1000000 * $med)" | bc -l)
    printf "    → Median: %s MA/s, Latency: %.3e s\n" "$med" "$latency"

    # Log to CSV
    printf "%s,%s,%s,%s,%.3e\n" "$freq" "$size" "$level" "$med" "$latency" >> "$outfile"
  done
done
