# header.stamp is the host command Unix time (same instant as t_cmd_unix_ns).
# It approximates the trigger; the firmware provides NO hardware timestamp.
std_msgs/Header header
uint64 capture_id
# A: emit waveform and capture; P: passive capture only.
string mode
uint8 events
uint32 byte_count

# Host CLOCK_REALTIME and CLOCK_MONOTONIC, preserving all recorder clock fields.
# Monotonic values are meaningful only on the same host/boot, not as ROS stamps.
uint64 t_cmd_unix_ns
uint64 t_done_unix_ns
uint64 t_cmd_mono_ns
uint64 t_done_mono_ns
string t_cmd_utc

float64 sample_rate_hz
uint32 samples_per_event
uint8 channel_count
# Word bit index -> physical microphone number (1..16), in board order.
uint8[16] bit_to_microphone
# Each word preserves a little-endian uint16 from the original .bin:
# bits 0..7 = GPIOB[7:0] on the rising edge, bits 8..15 on the falling edge.
# One PDM bit per microphone per word; these are NOT PCM amplitudes or ranges.
uint16[] pdm_words
# Actual 1024-sample DAC table, silence-padded with 2048; empty in passive mode.
float64 waveform_sample_rate_hz
uint16[] transmit_waveform

# Optional host webcam image, published separately on ~/image/compressed.
# Image stamp is host receive time before the sonar command, not hardware exposure time.
bool has_image
builtin_interfaces/Time image_stamp

# Measurement label only; does not change acquisition or create files/directories.
string tag
