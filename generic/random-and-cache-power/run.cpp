#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <filesystem>
#include <fstream>

void set_frequency(int freq_khz) {
  namespace fs = std::filesystem;

  const std::string uncore_base = "/sys/devices/system/cpu/intel_uncore_frequency/package_00_die_00";
  const std::string min_freq_path = uncore_base + "/min_freq_khz";
  const std::string max_freq_path = uncore_base + "/max_freq_khz";

  // Loop over all CPU frequency directories
  for (const auto& entry : fs::directory_iterator("/sys/devices/system/cpu/")) {
    if (entry.is_directory()) {
      std::string path = entry.path().string();
      if (path.find("cpu") != std::string::npos && path.find("cpufreq") == std::string::npos) {
        std::string cpufreq_path = path + "/cpufreq";
        if (fs::exists(cpufreq_path)) {
          std::ofstream(cpufreq_path + "/scaling_min_freq") << "800000";
          std::ofstream(cpufreq_path + "/scaling_max_freq") << "800000";
        }
      }
    }
  }

  // Set uncore frequency
  std::ofstream(min_freq_path) << std::to_string(freq_khz);
  std::ofstream(max_freq_path) << std::to_string(freq_khz);
}

const std::array<int, 4> sizes = {4000000};
const std::array<std::string, 4> levels = {"DRAM"};
const int runs = 5;
const std::string outfile = "ubench_power_vs_freq.csv";

double median(std::vector<double>& values) {
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

double run_benchmark(int size) {
  std::string cmd = "./ubench-x64 " + std::to_string(size);
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    std::cerr << "Failed to run command: " << cmd << "\n";
    return -1.0;
  }

  char buffer[128];
  std::string result;
  while (fgets(buffer, sizeof(buffer), pipe)) {
    result += buffer;
  }
  pclose(pipe);

  try {
    return std::stod(result);
  } catch (...) {
    std::cerr << "Failed to parse output: " << result << "\n";
    return -1.0;
  }
}

int main(int argc, char* argv[]) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <min_freq_kHz> <max_freq_kHz> <step_kHz>\n";
    return 1;
  }

  int min_freq = std::stoi(argv[1]);
  int max_freq = std::stoi(argv[2]);
  int step = std::stoi(argv[3]);

  std::ofstream csv(outfile);
  csv << "frequency_hz,size_kb,level,power_watts\n";

  for (int freq = min_freq; freq <= max_freq; freq += step) {
    std::cout << "Setting CPU frequency to " << freq << " kHz\n";
    set_frequency(freq);

    for (size_t i = 0; i < sizes.size(); ++i) {
      int size = sizes[i];
      const std::string& level = levels[i];
      std::cout << "  Running ./ubench-x64 " << size << " (" << level << ")\n";

      std::vector<double> powers;
      for (int j = 0; j < runs; ++j) {
        double power = run_benchmark(size);
        if (power < 0) {
          std::cerr << "  Skipping due to error.\n";
          continue;
        }
        powers.push_back(power);
        std::system("likwid-memsweeper > /dev/null");
      }

      if (powers.size() < runs) {
        std::cerr << "  Not enough valid runs for size " << size << "\n";
        continue;
      }

      double med_power = median(powers);
      std::cout << "    → Median Power: " << med_power << " W\n";

      csv << freq << "," << size << "," << level << "," << med_power << "\n";
    }
  }

  csv.close();
  return 0;
}
