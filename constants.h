// Pin numbering on Teensy
#define ssPin       10
#define resetPin    24
#define updatePin   5
#define ps0Pin     25
#define ps1Pin     26
#define oskPin     27
#define pwrContrPin   28
//#define interruptPin  9 // NOTE!!! This is NOT on DDS3 and DDS4
#define interruptPin   2 //  This is on DDS3 and DDS4!!!

const unsigned long SPIclockspeed = 1000000;

// upper and lower freq and power limits DDS
const float lower_freq_lim = 1000000; // in Hz
const float upper_freq_lim = 160000000;
const float lower_power_lim = 0;
const float upper_power_lim = 100;

// Properties of hardware (HW) and software (SW) ramp
const float lower_HW_ramp_timelimit = 1; // shortest HW ramp time, given in microseconds
const float higher_HW_ramp_timelimit = 1e7; // longest HW ramp time, given in microseconds
const float software_ramp_time_limit = 15000; // 15000 ms = 15 s for now
const float software_ramp_min_timestep = 0.1; // 0.1 ms = 100 micros , given in ms
const float ramp_time_fraction_from_requested = 0.9; // this is to make sure that the ramp ends before the requested time so
// that the next trigger does not appear before the ramp is done and the timer is stopped
// VERY IMPORTANT! Pay attention to this ramp_time_fraction_from_requested, otherwise timing bugs will result!
const int waitAfterHWRamp = 10; // given in microseconds

// this is how many letters there are in the instruction sequence, like "fplw" or like "fvpv"
const byte numBlocksSeqStep = 4;

// the max possible steps for any sequence, this will be the max memory chunk for data
// This cannot be changed in any particular sequence but is rather a firmware limit
// for any possible sequence
const int max_possible_sequence_steps = 20;

// This is now the number of steps in a given sequence.
// obviously num_steps_total_sequence >= max_possible_sequence_steps
int num_steps_total_sequence;
