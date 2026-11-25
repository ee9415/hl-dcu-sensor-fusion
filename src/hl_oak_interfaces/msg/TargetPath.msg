# Vehicle front axle center: +X forward, +Y left, meters. Capture time, not publish time.
std_msgs/Header header
string source_id
uint64 frame_sequence
float64[4] coefficients
float64 valid_length_m
float64 observed_start_m
float64 fit_rmse_m
uint8 confidence
bool valid
bool calibrated
bool timing_verified
bool bench_only
uint32 capture_uncertainty_ms
string reason
