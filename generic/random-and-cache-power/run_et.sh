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

sizes=(12 368 6144)
levels=("L1" "L2" "L3")

# Header for CSV
echo "frequency_hz,size_kb,level,throughput_ma_s,latency_s,energy_j,time_s" > "$outfile"

# Function to get median from array
median() {
  sorted=($(printf "%s\n" "$@" | sort -n))
  mid=$((runs / 2))
  echo "${sorted[$mid]}"
}

# Loop through frequencies
for ((freq=min_freq; freq<=max_freq; freq+=step)); do
  echo "Setting CPU frequency to $freq kHz"
  if (( freq <= 2700000 )); then
    sudo cpupower frequency-set -g userspace > /dev/null
    sudo cpupower frequency-set -f ${freq} > /dev/null
  else
    ./scale.sh "$freq"
  fi

  for i in "${!sizes[@]}"; do
    size=${sizes[$i]}
    level=${levels[$i]}
    echo "  Running ./ubench-x64 $size ($level)"

    energies=()
    times=()
    throughputs=()

    for ((j=0; j<runs; j++)); do
      output=$(./ubench-x64 "$size")
      energy=$(echo "$output" | awk '/Energy overflow thread running/{getline; print}')
      time=$(echo "$output" | awk '/Energy overflow thread running/{getline; getline; print}')
      throughput=$(echo "$output" | awk '/MOV/ {print $2}')
      
      energies+=("$energy")
      times+=("$time")
      throughputs+=("$throughput")

      likwid-memsweeper > /dev/null
    done

    med_energy=$(median "${energies[@]}")
    med_time=$(median "${times[@]}")
    med_throughput=$(median "${throughputs[@]}")
    latency=$(echo "scale=15; 1 / (1000000 * $med_throughput)" | bc -l)

    printf "    → Median: %s MA/s, Latency: %.3e s, Energy: %s J, Time: %s s\n" \
      "$med_throughput" "$latency" "$med_energy" "$med_time"

    # Log to CSV
    printf "%s,%s,%s,%s,%.3e,%s,%s\n" \
      "$freq" "$size" "$level" "$med_throughput" "$latency" "$med_energy" "$med_time" >> "$outfile"
  done
done

