#ifndef ETHERCAT_MASTER_HPP
#define ETHERCAT_MASTER_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "ethercat.h"
#include <inttypes.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>

typedef enum { INIT = 1, PREOP = 2, SAFEOP = 4, OP = 8 } ethercat_states_t;

typedef enum {
  Not_ready_to_switch_on = 0x0,
  Switch_on_disabled = 1 << 6,
  Ready_to_switch_on = (1 << 5) | 1,
  Switch_on = (1 << 5) | 3,
  Operation_enabled = (1 << 5) | 7,
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

typedef enum { CSP = 0, CSV = 1, CST = 2 } operational_mode_t;

typedef struct {
  int expectedWKC;
  operational_mode_t mode;
  ethercat_states_t ethercat_state;
  statusword_state_t statusword_state;
} control_state_t;

control_state_t state{.mode = CSP,
                      .ethercat_state = INIT,
                      .statusword_state = Not_ready_to_switch_on};

char[8096] IOMap;
OSAL_THREAD_HANDLE check_thread;

pthread_t main_thread;
pthread_attr_t attr;
cpu_set_t cpuset;
struct sched_param param;
volatile int wkc;

/**
 * @brief Structure to hold the inputs from the EtherCAT Slave
 * Is the same for all control methods
 */
typedef struct {
  uint16 controlword;
  uint32 target;
} __attribute__((packed)) our_outputs;

int initialize_ethercat(const char *ifname);

/**
 * @brief Configures the servo when going from PREOP -> SAFEOP
 * mode: 0 = CSP, 1 = CSV, 2 = CST
 */
int configure_servo(int mode);

/**
 * @brief Function that checks if the slave is still ok and
 * tries to recover the slave when required.
 */
OSAL_THREAD_FUNC ecatcheck(void *ptr);

void *loop();

/**
 * @brief Function that starts the main ethercat loop (in OP)
 */
int start_loop(void);

/**
 * @brief stops the main ethercat OP loop.
 * Slave will automatically go back to SAFEOP
 */
void stop_loop(void);

#ifdef __cplusplus
}
#endif

#endif