#ifndef ETHERCAT_MASTER_HPP
#define ETHERCAT_MASTER_HPP

#include "ros/ros.h"
#include <atomic>
#include <math.h>
#include <mutex>
#define EC_TIMEOUTMON 500
#define STATUS_WORD_MASK(x)                                                    \
  (x &= ~((1 << 4) | (1 << 5) | (1 << 8) | (1 << 9) | (1 << 14) | (1 << 15)))

// Control parameters
#define MAX_ACC 200000UL
#define MAX_VEL 10000000
#define MARGIN 75000
#define VEL_MARGIN 0.02 * MAX_VEL

// Conversion parameters
#define FULL_ROT 42000000.0
#define RAD_TO_POS (FULL_ROT / M_PI / 2.0)
#define TIME_CONV_VEL 4UL
#define TIME_CONV_ACC 25UL
// = 100 / (cycle_rate_in_ms ^ 2). Example: cycle rate = 2ms -->
// TIME_CONV_ACC = 100 / (2 * 2) = 25

// Max span in units (MAX approx 140deg = 2.443460952792061rad = 16333333.33...
// units)
#define MAX_SPAN 15166666

#ifdef __cplusplus
extern "C" {
#endif

#include "ethercat.h"
#include <assert.h>
#include <inttypes.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>

typedef enum { INIT = 1, PREOP = 2, SAFEOP = 4, OP = 8 } ethercat_states_t;

typedef enum {
  Not_ready_to_switch_on = 0x0,
  Switch_on_disabled = 1 << 6,
  Ready_to_switch_on = 1,
  Switch_on = 3,
  Operation_enabled = 0x7 | (1 << 12),
  Quick_stop_active = 0x7,
  Fault_reaction_active = 0x0f,
  Fault = 1 << 3
} statusword_state_t;

typedef enum {
  control_switch_on = 0,
  control_enable_voltage = 1,
  control_quick_stop = 2,
  control_enable_operation = 3,
  control_fault_reset = 7,
  control_4 = 4,
  control_5 = 5,
  control_6 = 6,
  control_8 = 8
} control_bit_t;

typedef enum { CSP = 0, CSV = 1, CST = 2, PP = 3 } operational_mode_t;

typedef struct {
  int expectedWKC;
  operational_mode_t mode;
  ethercat_states_t ethercat_state;
  statusword_state_t statusword_state;
} control_state_t;

extern control_state_t servo_state;

extern char IOMap[8096];
extern OSAL_THREAD_HANDLE check_thread;

extern pthread_t main_thread;
extern pthread_attr_t attr;
extern cpu_set_t cpuset;
extern struct sched_param param;
extern volatile int wkc;

// base_pos not atomic as normally only set once (then read)
extern uint32_t base_pos;
extern std::atomic_int32_t target;
extern std::atomic_bool enable_servo;
extern std::atomic_bool loop_flag;
extern std::atomic_bool check_flag;

/**
 * @brief Structure to hold the inputs from the EtherCAT Slave
 * Is the same for all control methods
 */
typedef struct {
  uint16 controlword;
  uint32 target;
} __attribute__((packed)) master_outputs;

/**
 * @brief Structure to hold the outputs from the EtherCAT Slave
 */
typedef struct {
  uint32 position;
  uint16 statusword;
  uint32 velocity;
  uint16 torque;
  uint32 erroract;
} __attribute__((packed)) CSP_inputs;
typedef CSP_inputs CSV_inputs;
typedef CSP_inputs PP_inputs;

extern std::atomic_int ethercat_state_ext;
extern std::mutex inputs_mutex;
extern CSP_inputs csp_inputs_ext;
extern CSV_inputs csv_inputs_ext;

// cppcheck-suppress [unusedFunction, unmatchedSuppression]
inline void set_output(uint16_t slave_nb, uint16_t controlword,
                       uint32_t value) {
  master_outputs *data_ptr;

  data_ptr = reinterpret_cast<master_outputs *>(ec_slave[slave_nb].outputs);
  data_ptr->controlword = controlword;
  data_ptr->target = value;
}

// cppcheck-suppress [unusedFunction, unmatchedSuppression]
inline CSP_inputs get_CSP_input(uint16_t slave_nb) {
  CSP_inputs inputs =
      *reinterpret_cast<CSP_inputs *>(ec_slave[slave_nb].inputs);

  return inputs;
}

// cppcheck-suppress [unusedFunction, unmatchedSuppression]
inline CSV_inputs get_CSV_input(uint16_t slave_nb) {
  CSV_inputs inputs =
      *reinterpret_cast<CSV_inputs *>(ec_slave[slave_nb].inputs);

  return inputs;
}

// cppcheck-suppress [unusedFunction, unmatchedSuppression]
inline PP_inputs get_PP_input(uint16_t slave_nb) {
  PP_inputs inputs = *reinterpret_cast<PP_inputs *>(ec_slave[slave_nb].inputs);

  return inputs;
}

int initialize_ethercat(const char *ifname, operational_mode_t mode);

/**
 * @brief Configures the servo when going from PREOP -> SAFEOP
 */
int configure_servo(uint16 slave);

/**
 * @brief Function that checks if the slave is still ok and
 * tries to recover the slave when required.
 */
OSAL_THREAD_FUNC ecatcheck(void *ptr);

void *loop();

/**
 * @brief Function that starts the main ethercat loop (in OP)
 */
int start_loop(operational_mode_t mode);

/**
 * @brief stops the main ethercat OP loop.
 * Slave will automatically go back to SAFEOP
 */
void stop_loop(void);

/**
 * @brief Function that resets the slave to INIT
 */
void reset_state(void);

uint32_t calc_csv_target(CSV_inputs csv_inputs, uint32_t cur_target);
uint32_t get_target_limited();

#ifdef __cplusplus
}
#endif

#endif