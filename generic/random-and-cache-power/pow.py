#!/usr/bin/env python3

import subprocess
import sys
import csv
import statistics
from time import sleep

# Run a shell command and return its stripped stdout; exit on error
def run_command(cmd):
  result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
  if result.returncode != 0:
    print(f"Command failed: {cmd}\n{result.stderr}")
    sys.exit(1)
  return result.stdout.strip()

# Parse benchmark output: "<energy> <time> <power>"
def parse_metrics(output):
  parts = output.split()
  if len(parts) != 3:
    raise ValueError(f"Expected 3 values, got {len(parts)}: '{output}'")
  energy, elapsed_time, power = map(float, parts)
  return energy, elapsed_time, power

# Given a list of triples, pick the one whose power is the median
# Returns (energy, time, power)
def select_median_by_power(measurements):
  # Extract powers and find median
  powers = [m[2] for m in measurements]
  med = statistics.median(powers)
  # In case of even count, median is average; pick the measurement closest to med
  best = min(measurements, key=lambda m: abs(m[2] - med))
  return best


def main():
  if len(sys.argv) != 4:
    print(f"Usage: {sys.argv[0]} <start_freq> <end_freq> <step>")
    sys.exit(1)

  start_freq = int(sys.argv[1])
  end_freq = int(sys.argv[2])
  step = int(sys.argv[3])

  with open("freq_power_log.csv", "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Frequency", "MedianEnergy", "MedianTime", "MedianPower"])

    for freq in range(start_freq, end_freq, step):
      print(f"\nSetting frequency to {freq}...")
      run_command(f"sudo ./scale.sh {freq}")

      measurements = []  # list of (energy, time, power)
      for i in range(15):
        print(f"  Run {i+1}/15...")
        run_command("likwid-memsweeper")
        output = run_command("sudo ./ubench-x64 300000")
        try:
          e, t, p = parse_metrics(output)
        except ValueError as ex:
          print(f"Invalid benchmark output: {ex}")
          sys.exit(1)
        measurements.append((e, t, p))
        sleep(0.5)

      # Select median measurement by power
      med_e, med_t, med_p = select_median_by_power(measurements)
      print(f"Logging: {freq}, Energy={med_e:.4f}, Time={med_t:.4f}, Power={med_p:.4f}")
      writer.writerow([freq, f"{med_e:.4f}", f"{med_t:.4f}", f"{med_p:.4f}"])
      csvfile.flush()
      sleep(1)

if __name__ == "__main__":
  main()

