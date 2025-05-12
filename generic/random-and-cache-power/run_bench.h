#include <atomic>
#include <cassert>
#include <chrono>
#include <cpufreq-bindings/cpufreq-bindings.h>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <omp.h>
#include <ostream>
#include <papi.h>
#include <raplcap/raplcap.h>
#include <stddef.h>
#include <stdexcept>
#include <stdint.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#define POLYBENCH_CACHE_SIZE_KB 32770

void polybench_flush_cache() {
  int cs = POLYBENCH_CACHE_SIZE_KB * 1024 / sizeof(double);
  double *flush = (double *)calloc(cs, sizeof(double));
  int i;
  double tmp = 0.0;
#ifdef _OPENMP
#pragma omp parallel for reduction(+ : tmp) private(i)
#endif
  for (i = 0; i < cs; i++)
    tmp += flush[i];
  assert(tmp <= 10.0);
  free(flush);
}


namespace powercapping_utils {
raplcap rc;
raplcap_limit rl_short_initial;
raplcap_limit rl_short;
uint32_t NumPackages, NumDie;

bool isPackageZoneSupported() {
  switch (raplcap_pd_is_zone_supported(&rc, 0, 0, RAPLCAP_ZONE_PACKAGE)) {
  default:
    std::cerr << "Some error occured . Exiting the program." << std::endl;
    exit(1);
    break;
  case 0:
    std::cerr << "Package zone unsupported . Exiting the program" << std::endl;
    exit(1);
    break;
  case 1:
    return true;
  }
  return false;
}

void enablePackageZone() {
  if (raplcap_pd_set_zone_enabled(&rc, 0, 0, RAPLCAP_ZONE_PACKAGE, 1)) {
    std::cerr << "Some error occured while enabling zone " << std::endl;
  }
}

bool isPackageZoneEnabled() {
  switch (raplcap_pd_is_zone_enabled(&rc, 0, 0, RAPLCAP_ZONE_PACKAGE)) {
  default:
    std::cerr << "Some error occured . Exiting the program." << std::endl;
    exit(1);
    break;
  case 0:
    std::cerr << "Package zone disabled . Enabling Package zone" << std::endl;
    enablePackageZone();
    break;
  case 1:
    return true;
  }
  return false;
}

void getraplcapShortLimit(raplcap_limit &rl) {
  switch (raplcap_pd_is_constraint_supported(&rc, 0, 0, RAPLCAP_ZONE_PACKAGE,
                                             RAPLCAP_CONSTRAINT_SHORT_TERM)) {
  default:
    std::cerr << "Some error occured . Exiting the program." << std::endl;
    exit(1);
    break;
  case 0:
    std::cerr << "Rapl cap constraint for RAPLCAP_CONSTRAINT_SHORT_TERM "
                 "unsupported . Exiting the program"
              << std::endl;
    exit(1);
    break;
  case 1:
    if (raplcap_pd_get_limit(&rc, 0, 0, RAPLCAP_ZONE_PACKAGE,
                             RAPLCAP_CONSTRAINT_SHORT_TERM, &rl) != 0) {
      std::cerr << " Error encountered while getting the short term power cap "
                   "for zone 0 die 0 . Exiting the program"
                << std::endl;
      exit(1);
    }
    std::cerr << " The Short Limit is currently  : " << rl.watts << " W ."
              << std::endl;
  }
}

int initialize_raplcap() {
  if (raplcap_init(&rc)) {
    std::cerr << "Unable to initilize raplcap. Exiting the program"
              << std::endl;
    exit(1);
  }
  NumPackages = raplcap_get_num_packages(&rc);

  if (NumPackages == 0) {
    std::cerr << "There are no packages on the system. Exiting the program"
              << std::endl;
    exit(1);
  }

  NumDie = raplcap_get_num_die(&rc, 0);

  if (NumDie == 0) {
    std::cerr << "There are no die-s present. Exiting the program" << std::endl;
  }

  isPackageZoneEnabled();

  if (isPackageZoneSupported()) {
    getraplcapShortLimit(
        rl_short_initial); // we will reset to the initial value at the end
  }

  return 1;
}

void setraplcapShortLimit(double limit) {
  isPackageZoneEnabled();
  raplcap_limit currlimit;
  getraplcapShortLimit(currlimit);
  currlimit.watts = limit;
  if (raplcap_pd_set_limit(&rc, 0, 0, RAPLCAP_ZONE_PACKAGE,
                           RAPLCAP_CONSTRAINT_SHORT_TERM, &currlimit)) {
    std::cerr << " Some error occured while setting the limit. Exiting.."
              << std::endl;
    exit(1);
  }
  std::cerr << " Power cap set to : " << limit << std::endl;
}

void resetPowerCaps() {
  std::cerr << " Reseting the short limit to : " << rl_short_initial.watts
            << " W" << std::endl;
  setraplcapShortLimit(rl_short_initial.watts);
  std::cerr << " Resetting complete. " << std::endl;
}

void destroy_replcap() {
  resetPowerCaps();
  if (raplcap_destroy(&rc)) {
    std::cerr << " Error encounterd while destroying the replcap ."
              << std::endl;
  }
}

} // namespace powercapping_utils

namespace papi_inst {

unsigned long get_thread_id() {
  return static_cast<unsigned long>(omp_get_thread_num());
}

std::string papi_event_name;
int main_eventset = PAPI_NULL;
long long temp_main_count = 0;
long long final_count = 0;
int main_retval = PAPI_OK;
std::vector<int> eventsets(omp_get_max_threads(), PAPI_NULL);
std::vector<int> retvals(omp_get_max_threads(), PAPI_OK);
std::vector<long long> temp_counts(omp_get_max_threads(), 0);
std::vector<long long> total_counts(omp_get_max_threads(), 0);

inline void papi_inst_init_global() {
  const char *env_var_name = "PAPI_EVENT_NAME";
  papi_event_name = getenv(env_var_name);
  int retval_init = PAPI_library_init(PAPI_VER_CURRENT);
  if (retval_init != PAPI_VER_CURRENT) {
    printf("PAPI library init error !\n");
    exit(1);
  }
  if (PAPI_thread_init(get_thread_id) != PAPI_OK) {
    printf("PAPI library init error !\n");
    exit(1);
  }
}

inline void papi_inst_main_init() {
  main_retval = PAPI_create_eventset(&main_eventset);
  if (main_retval != PAPI_OK) {
    std::cerr << "Error Creating Event set" << PAPI_strerror(main_retval)
              << std::endl;
  }
  main_retval = PAPI_add_named_event(main_eventset, papi_event_name.c_str());

  if (main_retval != PAPI_OK) {
    std::cerr << "Error adding :" << papi_event_name << " "
              << PAPI_strerror(main_retval) << std::endl;
  }
  main_retval = PAPI_reset(main_eventset);
  if (main_retval != PAPI_OK) {
    std::cerr << "Error reseting PAPI " << PAPI_strerror(main_retval)
              << std::endl;
  }
}

inline void papi_inst_main_start() {
  // main_retval = PAPI_reset(main_eventset);
  // if (main_retval != PAPI_OK) {
  //   std::cerr << "Error reseting PAPI " << PAPI_strerror(main_retval)
  //             << std::endl;
  // }
  main_retval = PAPI_start(main_eventset);
  if (main_retval != PAPI_OK) {
    std::cerr << "Error PAPI " << PAPI_strerror(main_retval);
    fprintf(stderr, "Error PAPI: %s\n", PAPI_strerror(main_retval));
  }
}

inline void papi_inst_main_end() {
  main_retval = PAPI_stop(main_eventset, &temp_main_count);
  final_count += temp_main_count;
  std::cout << temp_main_count << "**" << std::endl;
  temp_main_count = 0;
  // main_retval = PAPI_reset(main_eventset);
  if (main_retval != PAPI_OK) {
    std::cerr << "Error reseting PAPI " << PAPI_strerror(main_retval)
              << std::endl;
  }
}

inline void papi_inst_thread_init() {
  int id = PAPI_thread_id();
  retvals[id] = PAPI_create_eventset(&eventsets[id]);
  if (retvals[id] != PAPI_OK) {
    std::cerr << "Error Creating Event set" << PAPI_strerror(retvals[id])
              << std::endl;
  }
  retvals[id] = PAPI_add_named_event(eventsets[id], papi_event_name.c_str());

  if (retvals[id] != PAPI_OK) {
    std::cerr << "Error adding :" << papi_event_name << " "
              << PAPI_strerror(retvals[id]) << std::endl;
  }
}

inline void papi_inst_thread_start() {
  int id = PAPI_thread_id();
  // retvals[id] = PAPI_reset(eventsets[id]);
  // if (retvals[id] != PAPI_OK) {
  //   std::cerr << "Error reseting PAPI " << PAPI_strerror(retvals[id])
  //             << std::endl;
  // }
  retvals[id] = PAPI_start(eventsets[id]);
  if (retvals[id] != PAPI_OK) {
    std::cerr << "Error PAPI " << PAPI_strerror(retvals[id]);
    fprintf(stderr, "Error PAPI: %s\n", PAPI_strerror(retvals[id]));
  }
}

inline void papi_inst_thread_end() {
  int id = PAPI_thread_id();
  retvals[id] = PAPI_stop(eventsets[id], &temp_counts[id]);
  total_counts[id] += temp_counts[id];
  temp_counts[id] = 0;
  retvals[id] = PAPI_reset(eventsets[id]);
  if (retvals[id] != PAPI_OK) {
    std::cerr << "Error reseting PAPI " << PAPI_strerror(retvals[id])
              << std::endl;
  }
}

inline void papi_count_all() {
  for (auto count : total_counts) {
    final_count += count;
    std::cout << count << " ";
  }
  std::cout << std::endl;
}

inline void papi_print_count() {
  std::cout << papi_event_name << "=" << final_count << std::endl;
}

} // namespace papi_inst

namespace energy_time {

std::vector<int> cpufreq_bindings_files_fd;
unsigned number_of_frequency_files_core;
std::vector<std::string> frequency_files_uncore = {
    "/sys/devices/system/cpu/intel_uncore_frequency/package_00_die_00/"
    "max_freq_khz"};
double time_reading_ns = 0;
double time_reading = 0;
long long unsigned energy_reading = 0;
long long unsigned energy_reading_core = 0;
std::chrono::time_point<std::chrono::high_resolution_clock> start_time_counter;
std::chrono::time_point<std::chrono::high_resolution_clock> end_time_counter;
long long unsigned start_energy_counter = 0;
long long unsigned start_energy_counter_core = 0;
long long unsigned end_energy_counter = 0;
long long unsigned end_energy_counter_core = 0;
// Global max energy value
long long unsigned energy_max = (1ULL << 32) - 1;
long long unsigned energy_max_core = (1ULL << 32) - 1;
std::atomic<bool> stop_event(false);
std::thread counter_overflows;
std::vector<int> energy_overflows = {-1,-1};
double energy_j = 0.0;

void cache_all_files_fd() {
  for (int i = 0; i < number_of_frequency_files_core; i++) {
    int fd = cpufreq_bindings_file_open(
        i, CPUFREQ_BINDINGS_FILE_SCALING_MAX_FREQ, O_WRONLY);
    cpufreq_bindings_files_fd.push_back(fd);
  }
}

void getScalingMaxFreqFiles() {
  // std::vector<std::string> files;
  unsigned number_of_frequency_files = 0;
  std::string basePath = "/sys/devices/system/cpu/";

  try {
    // Iterate over the entries in the base directory
    for (const auto &entry : std::filesystem::directory_iterator(basePath)) {
      if (entry.is_directory() &&
          entry.path().filename().string().rfind("cpu", 0) == 0) {
        // Check for cpufreq/scaling_max_freq inside cpuX directory
        std::filesystem::path freqFile =
            entry.path() / "cpufreq/scaling_max_freq";
        if (std::filesystem::exists(freqFile)) {
          number_of_frequency_files++;
        }
      }
    }
  } catch (const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
  }
  number_of_frequency_files_core = number_of_frequency_files;
  cache_all_files_fd();
}

} // namespace energy_time

extern "C" unsigned long long _mlir_ciface_get_energy();

// Function for the counter thread
void counter_thread(std::vector<int> &count) {

  // std::cout << "Energy overflow thread running" << std::endl;
  uint64_t energy_start = energy_time::energy_max;
  uint64_t energy_start_core = energy_time::energy_max_core;

  while (!energy_time::stop_event) {
    try {
      unsigned long long energy = _mlir_ciface_get_energy();
      unsigned long long energy_core = _mlir_ciface_get_energy_core();
      if (energy < energy_start) {
        count[0]++;
      }
      if (energy_core < energy_start_core) {
        count[1]++;
      }
      energy_start = energy;
      energy_start_core = energy_core;
    } catch (const std::exception &e) {
      std::cerr << "Error in reading the energy: " << e.what() << std::endl;
      std::exit(1);
    }
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }
}

unsigned long long get_max_energy() {
  unsigned long long energy_uj;
  std::ifstream energy_file(
      "/sys/class/powercap/intel-rapl:0/max_energy_range_uj");

  // Check if the file was opened successfully
  if (!energy_file.is_open()) {
    std::cerr << "Failed to open energy_uj file" << std::endl;
    throw std::runtime_error("File opening failed");
  }

  // Read the energy value
  energy_file >> energy_uj;
  if (energy_file.fail()) {
    std::cerr << "Failed to read energy" << std::endl;
    energy_file.close();
    throw std::runtime_error("File reading failed");
  }
#ifdef HUMAN_READABLE
  std::cout << "Max core energy : " << energy_uj << " uJ" << std::endl;
#endif
  // Close the file
  energy_file.close();
  return energy_uj;
}

unsigned long long get_max_energy_core() {
  unsigned long long energy_uj;
  std::ifstream energy_file(
      "/sys/class/powercap/intel-rapl:0:0/max_energy_range_uj");

  // Check if the file was opened successfully
  if (!energy_file.is_open()) {
    std::cerr << "Failed to open energy_uj file" << std::endl;
    throw std::runtime_error("File opening failed");
  }

  // Read the energy value
  energy_file >> energy_uj;
  if (energy_file.fail()) {
    std::cerr << "Failed to read energy" << std::endl;
    energy_file.close();
    throw std::runtime_error("File reading failed");
  }
#ifdef HUMAN_READABLE
  std::cout << "Max core energy : " << energy_uj << " uJ" << std::endl;
#endif
  // Close the file
  energy_file.close();
  return energy_uj;
}

void set_frequency_cap_core(long int frequency) {
  // for (int fd : energy_time::cpufreq_bindings_files_fd) {
  for (int i = 0; i < energy_time::cpufreq_bindings_files_fd.size(); i++) {
    ssize_t ret = cpufreq_bindings_set_scaling_max_freq(
        energy_time::cpufreq_bindings_files_fd[i], i, frequency);
    if (ret == -1) {
      std::cerr << "Failed to set max frequency: " << std::strerror(errno)
                << std::endl;
    }
  }
}

// Function to set the uncore frequency cap
void set_frequency_cap_uncore(long int frequency) {
  int total_files = energy_time::frequency_files_uncore.size();
  // #pragma omp parallel for
  for (int i = 0; i < total_files; i++) {
    std::fstream file;
    file.open(energy_time::frequency_files_uncore[i], std::ios::out);
    std::stringstream freq;
    if (file.is_open()) {
      file << frequency;
    }
    file.close();
  }
}

extern "C" {

void getScalingMaxFreqFiles_() { energy_time::getScalingMaxFreqFiles(); }

unsigned long long _mlir_ciface_get_energy() {
  unsigned long long energy_uj;
  std::ifstream energy_file("/sys/class/powercap/intel-rapl:0/energy_uj");

  // Check if the file was opened successfully
  if (!energy_file.is_open()) {
    std::cerr << "Failed to open energy_uj file" << std::endl;
    throw std::runtime_error("File opening failed");
  }

  // Read the energy value
  energy_file >> energy_uj;
  if (energy_file.fail()) {
    std::cerr << "Failed to read energy" << std::endl;
    energy_file.close();
    throw std::runtime_error("File reading failed");
  }
#ifdef HUMAN_READABLE
  std::cout << "Energy core: " << energy_uj << " uJ" << std::endl;
#endif
  // Close the file
  energy_file.close();
  return energy_uj;
}

unsigned long long _mlir_ciface_get_energy_core() {
  unsigned long long energy_uj;
  std::ifstream energy_file("/sys/class/powercap/intel-rapl:0/energy_uj");

  // Check if the file was opened successfully
  if (!energy_file.is_open()) {
    std::cerr << "Failed to open energy_uj file" << std::endl;
    throw std::runtime_error("File opening failed");
  }

  // Read the energy value
  energy_file >> energy_uj;
  if (energy_file.fail()) {
    std::cerr << "Failed to read energy" << std::endl;
    energy_file.close();
    throw std::runtime_error("File reading failed");
  }
#ifdef HUMAN_READABLE
  std::cout << "Energy core: " << energy_uj << " uJ" << std::endl;
#endif
  // Close the file
  energy_file.close();
  return energy_uj;
}

void _mlir_ciface_start_energy_time() {
  energy_time::getScalingMaxFreqFiles();
  energy_time::start_time_counter = std::chrono::high_resolution_clock::now();
  energy_time::energy_max = get_max_energy();
  energy_time::energy_max_core = get_max_energy_core();
  energy_time::start_energy_counter = _mlir_ciface_get_energy();
  energy_time::start_energy_counter_core = _mlir_ciface_get_energy_core();
  if (energy_time::stop_event) {
    // std::cout << "Energy overflow check thread stop event set to true might be "
    //              "an error";
    energy_time::stop_event = false;
  }
  energy_time::counter_overflows =
      std::thread(counter_thread, std::ref(energy_time::energy_overflows));
}

void _mlir_ciface_stop_energy_time() {
  energy_time::end_energy_counter = _mlir_ciface_get_energy();
  energy_time::end_energy_counter_core = _mlir_ciface_get_energy_core();
  energy_time::end_time_counter = std::chrono::high_resolution_clock::now();
  energy_time::stop_event = true;
  energy_time::counter_overflows.join();
}

void _mlir_ciface_print_power() {
  std::chrono::duration<double> timing =
      energy_time::end_time_counter - energy_time::start_time_counter;
  if (energy_time::energy_overflows[0] > 0) {
    // std::cout << "Number of times Energy Overflowed : "
    //           << energy_time::energy_overflows[0] << std::endl;
    energy_time::energy_reading =
        (energy_time::end_energy_counter - energy_time::start_energy_counter) +
        (energy_time::energy_overflows[0] * energy_time::energy_max);
  } else {
    energy_time::energy_reading =
        energy_time::end_energy_counter - energy_time::start_energy_counter;
  }
  if (energy_time::energy_overflows[1] > 0) {
    // std::cout << "Number of times Energy Overflowed : "
    //           << energy_time::energy_overflows[1] << std::endl;
    energy_time::energy_reading_core =
        (energy_time::end_energy_counter_core -
         energy_time::start_energy_counter_core) +
        (energy_time::energy_overflows[1] * energy_time::energy_max_core);
  } else {
    energy_time::energy_reading_core =
        energy_time::end_energy_counter_core -
        energy_time::start_energy_counter_core;
  }
  energy_time::time_reading_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(timing).count();
  energy_time::time_reading = double(energy_time::time_reading_ns) / 1000000000;
  energy_time::energy_j = double(energy_time::energy_reading - energy_time::energy_reading_core) / 1000000;
#ifdef HUMAN_READABLE
  std::cerr << "Measured Energy (J) : " << energy_time::energy_j << std::endl;
  std::cerr << "Measured Time (s) : " << energy_time::time_reading << std::endl;
  std::cerr << "Measured Energy (J) : " << energy_time::energy_reading
            << std::endl;
  std::cerr << "Start Energy (uJ) " << energy_time::start_energy_counter
            << std::endl;
  std::cerr << "End Energy (uJ) " << energy_time::end_energy_counter
            << std::endl;
#endif
  std::cout << energy_time::energy_j / energy_time::time_reading << std::endl;
}

void _mlir_ciface_printOpen() { std::cout << "( "; }
void _mlir_ciface_printClose() { std::cout << " )"; }
void _mlir_ciface_printComma() { std::cout << " , "; }
void _mlir_ciface_printNewline() { std::cout << std::endl; }

void _mlir_ciface_set_frequency_caps(long int core, long uncore) {
  // set_frequency_cap_core(core);
  set_frequency_cap_uncore(uncore);
  return;
}

// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------
void print_energy_time() {
  std::chrono::duration<double> timing =
      energy_time::end_time_counter - energy_time::start_time_counter;
  if (energy_time::energy_overflows[0] > 0) {
    // std::cout << "Number of times Energy Overflowed : "
    //           << energy_time::energy_overflows[0] << std::endl;
    energy_time::energy_reading =
        (energy_time::end_energy_counter - energy_time::start_energy_counter) +
        (energy_time::energy_overflows[0] * energy_time::energy_max);
  } else {
    energy_time::energy_reading =
        energy_time::end_energy_counter - energy_time::start_energy_counter;
  }
  energy_time::time_reading_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(timing).count();
  energy_time::time_reading = double(energy_time::time_reading_ns) / 1000000000;
  energy_time::energy_j = double(energy_time::energy_reading) / 1000000;
#ifdef HUMAN_READABLE
  std::cerr << "Measured Energy (J) : " << energy_time::energy_j << std::endl;
  std::cerr << "Measured Time (s) : " << energy_time::time_reading << std::endl;
  std::cerr << "Measured Energy (J) : " << energy_time::energy_reading
            << std::endl;
  std::cerr << "Start Energy (uJ) " << energy_time::start_energy_counter
            << std::endl;
  std::cerr << "End Energy (uJ) " << energy_time::end_energy_counter
            << std::endl;
#endif
  std::cout << energy_time::energy_j / energy_time::time_reading<< std::endl;
  // std::cout << energy_time::time_reading << std::endl;
  // std::cout << energy_time::energy_reading << std::endl;
  // std::cout << energy_time::start_energy_counter << std::endl;
  // std::cout << energy_time::end_energy_counter << std::endl;
}

void start_energy_time() {
  energy_time::getScalingMaxFreqFiles();
  energy_time::start_time_counter = std::chrono::high_resolution_clock::now();
  energy_time::energy_max = get_max_energy();
  energy_time::start_energy_counter = _mlir_ciface_get_energy();
  if (energy_time::stop_event) {
    // std::cout << "Energy overflow check thread stop event set to true might be "
    //              "an error";
    energy_time::stop_event = false;
  }
  energy_time::counter_overflows =
      std::thread(counter_thread, std::ref(energy_time::energy_overflows));
}

void stop_energy_time() {
  energy_time::end_energy_counter = _mlir_ciface_get_energy();
  energy_time::end_time_counter = std::chrono::high_resolution_clock::now();
  energy_time::stop_event = true;
  energy_time::counter_overflows.join();
}

void printOpen() { std::cout << "( "; }
void printClose() { std::cout << " )"; }
void printComma() { std::cout << " , "; }
void printNewline() { std::cout << std::endl; }

void set_frequency_caps(long int core, long uncore) {
  set_frequency_cap_core(core);
  set_frequency_cap_uncore(uncore);
  return;
}
// -------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------

// --------------------- PAPI
void papi_inst_init_global() { papi_inst::papi_inst_init_global(); }
void papi_inst_thread_init() { papi_inst::papi_inst_thread_init(); }
void papi_inst_thread_start() { papi_inst::papi_inst_thread_start(); }
void papi_inst_thread_end() { papi_inst::papi_inst_thread_end(); }
void papi_inst_main_init() { papi_inst::papi_inst_main_init(); }
void papi_inst_main_start() { papi_inst::papi_inst_main_start(); }
void papi_inst_main_end() { papi_inst::papi_inst_main_end(); }
void papi_count_all() { papi_inst::papi_count_all(); }
void papi_print_count() { papi_inst::papi_print_count(); }

void _mlir_ciface_papi_inst_init_global() {
  papi_inst::papi_inst_init_global();
}
void _mlir_ciface_papi_inst_thread_init() {
  papi_inst::papi_inst_thread_init();
}
void _mlir_ciface_papi_inst_thread_start() {
  papi_inst::papi_inst_thread_start();
}
void _mlir_ciface_papi_inst_thread_end() { papi_inst::papi_inst_thread_end(); }
void _mlir_ciface_papi_inst_main_init() { papi_inst::papi_inst_main_init(); }
void _mlir_ciface_papi_inst_main_start() { papi_inst::papi_inst_main_start(); }
void _mlir_ciface_papi_inst_main_end() { papi_inst::papi_inst_main_end(); }
void _mlir_ciface_papi_count_all() { papi_inst::papi_count_all(); }
void _mlir_ciface_papi_print_count() { papi_inst::papi_print_count(); }
}