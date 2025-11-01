builtin_interfaces/Time stamp

# --- Power & Throttling ---
bool undervoltage_detected
bool undervoltage_recent
bool undervoltage_history

bool overvoltage_detected
bool overvoltage_recent
bool overvoltage_history

bool throttled
bool throttled_recent
bool throttled_history

bool frequency_capped
bool frequency_capped_recent
bool frequency_capped_history

# --- Thermal ---
float32 cpu_temp_c
float32 gpu_temp_c
bool overtemp
bool overtemp_recent
bool soft_temp_limit

# --- CPU & Performance ---
float32 cpu_load_total_percent
float32[] cpu_load_per_core
float32 cpu_freq_mhz
float32 uptime_sec

# --- Memory ---
float32 mem_used_mb
float32 mem_total_mb
float32 swap_used_mb
float32 swap_total_mb

# --- Storage ---
float32 disk_used_percent
float32 disk_free_gb
float32 disk_total_gb

# --- Network ---
string primary_interface
float32 tx_rate_kbps
float32 rx_rate_kbps
uint64 tx_errors
uint64 rx_errors
bool link_up

# --- System & Boot Info ---
string os_release
string kernel_version
string firmware_version
string last_boot_reason
string startup_status

# --- Aggregated Session Stats ---
float32 max_cpu_temp_c
uint32 undervoltage_events
uint32 overvoltage_events
uint32 throttled_events
uint32 overtemp_events

# --- Health Summary ---
bool healthy
string status_summary
