#include "ethercat_master.hpp"

// BEGIN variables with possible race conditions
int expectedWKC = 0;
bool needlf = false;
// END variables with possible race conditions

std::atomic_uint32_t target = 0;

void *loop(void *mode_ptr) {
  operational_mode_t mode = *(operational_mode_t *)mode_ptr;

  CSP_inputs csp_inputs;
  CSV_inputs csv_inputs;

  // Start precise timer for 2000us
  // use clock_gettime(CLOCK_MONOTONIC, &ts) for better precision
  struct timespec tcur = {0, 0}, tprev = {0, 0};
  clock_gettime(CLOCK_MONOTONIC, &tcur);
  tprev = tcur;

  // Make the inputs (of master) valid!
  ec_send_processdata();
  wkc = ec_receive_processdata(EC_TIMEOUTRET);

  // Get inputs and set initial target depending on mode
  if (mode == operational_mode_t::CSP) {
    csp_inputs = get_CSP_input(1);
    target = csp_inputs.position;
  } else if (mode == operational_mode_t::CSV) {
    csv_inputs = get_CSV_input(1);
    target = csv_inputs.position;
  } else {
    // TODO CST not supported yet
    assert(0 && "Mode not supported");
  }

  /**Statusword
    * Bit 0 : Ready to switch on
    * Bit 1 : Switched on
    * Bit 2 : Operation enabled
    * Bit 3 : Fault
    * Bit 4 : Reserved / Voltage enabled
    * Bit 5 : Reserved / Quick stop
    * Bit 6 : Switch on disabled
    * Bit 7 : Warning
    * Bit 8 + 9 : Reserved
    * Bit 10 : TxPDOToggle
    * Bit 11 : Internal limit active
    * Bit 12 : Drive follows the command value
    * Bit 13 : Input cycle counter
    * Bit 14 + 15 : Reserved
    */
  uint16_t statusword;

  /**Controlword
   * Bit 0 : Switch on
   * Bit 1 : Enable voltage
   * Bit 2 : Reserved / Quick stop
   * Bit 3 : Enable operation
   * Bit 4 - 6 : Reserved
   * Bit 7 : Fault reset
   * Bit 8 - 15 : Reserved
   */
  uint16_t controlword = 0;

  uint32_t relative_offset = 0;
  int direction = 1;

  // Start control loop
  while (1) {
    // Start precise timer for 2000us
    do {
      clock_gettime(CLOCK_MONOTONIC, &tcur);
    } while ((tcur.tv_sec - tprev.tv_sec) * 1000000000 +
                  (tcur.tv_nsec - tprev.tv_nsec) <
              2000000);
    tprev = tcur;

    // Do logic
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    
    // Get input depending on mode
    if (mode == operational_mode_t::CSP) {
      csp_inputs = get_CSP_input(1);
    } else if (mode == operational_mode_t::CSV) {
      csv_inputs = get_CSV_input(1);
    } else {
      // TODO CST not supported yet
      assert(0 && "Mode not supported");
    }

    if (wkc >= expectedWKC) {

      needlf = true;

      // Get statusword depending on mode
      if (mode == operational_mode_t::CSP) {
        statusword = csp_inputs.statusword;
      } else if (mode == operational_mode_t::CSV) {
        statusword = csv_inputs.statusword;
      } else {
        // TODO CST not supported yet
        assert(0 && "Mode not supported");
      }

      // Mask/Ignore reserved bits (4,5,8,9,14,15) of status word
      statusword = statusword &
      ~((1 << 4) | (1 << 5) | (1 << 8) | (1 << 9) | (1 << 14) | (1 << 15));

      // TODO: Redo logic
      if (operation_enabled == 0) {
        switch (statusword) {
        case (Not_ready_to_switch_on): {
          /* Now the FSM should automatically go to Switch_on_disabled*/
          break;
        }
        case (Switch_on_disabled):
        case (Switch_on_disabled | (0b1 << 5)): {
          /* Automatic transition (2)*/
          controlword = 0;
          controlword |=
              (1 << control_enable_voltage) | (1 << control_quick_stop);
          if (i % 10 == 0) {
            controlword |= 1 << control_fault_reset;
          }
          break;
        }
        case (Ready_to_switch_on):
          // case (Ready_to_switch_on | (0b1 << 5)):
          {
            /* Switch on command for transition (3) */
            controlword |= 1 << control_switch_on;
            controlword = 0x07;
            break;
          }
        case (Switch_on): {
          /* Enable operation command for transition (4) */
          controlword |= 1 << control_enable_operation;
          break;
        }
        case (Operation_enabled): {
          /* Setting modes of operation
                    * Value Description
                    -128...-2 Reserved
                    -1 No mode
                    0 Reserved
                    1 Profile position mode
                    2 Velocity (not supported)
                    3 Profiled velocity mode
                    4 Torque profiled mode
                    5 Reserved
                    6 Homing mode
                    7 Interpolated position mode
                    8 Cyclic Synchronous position
                    ...127 Reserved*/

          uint16_t mode = 1; /* Setting Cyclic Synchronous position */
          int mode_size = sizeof(mode);
          // int SDO_result = ec_SDOwrite(1, MODES_OF_OPERATION_INDEX, 0,
          //                              0, mode_size, &mode,
          // EC_TIMEOUTRXM); controlword |= 0x1f;

          operation_enabled = 1;
          break;
        }
        case (Quick_stop_active): {
          break;
        }
        case (Fault_reaction_active): {
          break;
        }
        case (Fault):
        case (0x28): {
          /* Returning to Switch on Disabled */
          controlword = (1 << control_fault_reset);
          controlword |=
              (1 << control_enable_voltage) | (1 << control_quick_stop);
          break;
        }
        default: {
          printf("Unrecognized status\n");
          break;
        }
        }
      }

      relative_offset += 500 * direction;

      if (direction == 1 &&
          (int32_t)target - (int32_t)inputs.position > 250000) {
        relative_offset = 250000; // target - inputs.position;
      } else if (direction == -1 &&
                  (int32_t)inputs.position - (int32_t)target > 250000) {
        relative_offset = -250000; // target - inputs.position;
      }

      target = 125000000 * direction;

      printf("Target: %d ", target);
      printf("Value: %d ", inputs.position);
      printf("Relative offset: %d ", relative_offset);
      printf("Velocity %d ", inputs.velocity);
      printf("Torque %d ", inputs.torque);
      printf("Controlword: %#x ", controlword);
      printf("Statusword: %#x (%d) ", statusword, statusword);
      printf("Error: %d\r", inputs.erroract);

      set_output(1, controlword, target);
    }

    // Increment chk
    chk--;

    if (chk % 200 == 0) {
      direction = -direction;
    }
  }

  pthread_exit(NULL);

  return NULL;
}

int start_loop(operational_mode_t mode) {

  // Start the check thread
  osal_thread_create(&check_thread, 128000, &ecatcheck, NULL);

  // Set the state to operational
  state.expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
  ec_slave[0].state = EC_STATE_OPERATIONAL;

  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);
  ec_writestate(0);
  int chk = 200;
  do {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
  } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

  ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 4);

  if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
    return 0;
  }

  state.ethercat_state = OP;

  // Now that the slave is in operation, start the main loop thread
  pthread_attr_init(&attr);
  CPU_ZERO(&cpuset);
  CPU_SET(1, &cpuset);
  pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

  param.sched_priority = 99;

  pthread_attr_setschedparam(&attr, &param);

  // Create thread
  pthread_create(&main_thread, &attr, loop, (void *)&mode);
}

void stop_loop() {
  // TODO: Verify graceful shutdown, signals might also be send to self
  pthread_kill(main_thread, SIGKILL);
  pthread_join(main_thread, NULL);
}

int initialize_ethercat(const char *ifname, operational_mode_t mode) {
  state.mode = mode;

  if (ec_init(ifname)) {
    printf("ec_init on %s succeeded.\n", ifname);

    if (ec_config_init(FALSE) > 0) {

      printf("%d slaves found and configured.\n", ec_slavecount);

      ec_slave[1].PO2SOconfig = configure_servo;
      ec_config_map(&IOmap);
      ec_configdc();

      printf("Slaves mapped, state to SAFE_OP.\n");

      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

      //!! VERY IMPORTANT. THE SERVO NEEDS SOME TIME, OTHERWISE YOU'LL GET STUCK
      //!!!
      osal_usleep(100000);
    } else {
      printf("No slaves found!\n");
      return 1;
    }
  } else {
    printf("ec_init failed :(\n");
    return 1;
  }
}

OSAL_THREAD_FUNC ecatcheck(void *ptr) {
  int slave;
  (void)ptr; /* Not used */

  while (1) {
    if (state.ethercat_state == OP &&
        ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
      if (needlf) {
        needlf = FALSE;
        printf("\n");
      }
      /* one ore more slaves are not responding */
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++) {
        if ((ec_slave[slave].group == currentgroup) &&
            (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n",
                   slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
            printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n",
                   slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          } else if (ec_slave[slave].state > EC_STATE_NONE) {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              printf("MESSAGE : slave %d reconfigured\n", slave);
            }
          } else if (!ec_slave[slave].islost) {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE) {
              ec_slave[slave].islost = TRUE;
              printf("ERROR : slave %d lost\n", slave);
            }
          }
        }
        if (ec_slave[slave].islost) {
          if (ec_slave[slave].state == EC_STATE_NONE) {
            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              printf("MESSAGE : slave %d recovered\n", slave);
            }
          } else {
            ec_slave[slave].islost = FALSE;
            printf("MESSAGE : slave %d found\n", slave);
          }
        }
      }
      if (!ec_group[currentgroup].docheckstate)
        printf("OK : all slaves resumed OPERATIONAL.\n");
    }
    osal_usleep(10000);
  }
}

int configure_servo() {

  uint8 u8val;
  uint16 u16val;
  uint32 u32val;
  int retval = 0;

  // Set mode of operation
  if (state.mode == operational_mode_t::CSP) {
    // CSP
    u8val = 8;
    retval = ec_SDOwrite(slave, 0x7010, 0x0003, FALSE, sizeof(u8val), &u8val,
                         EC_TIMEOUTSAFE);
    if (retval == 0)
      return 0;

    uint16_t map_position_1c12[] = {0x0002, 0x1600, 0x1606};
    uint16_t map_position_1c13[] = {0x0005, 0x1A00, 0x1A01,
                                    0x1A02, 0x1A03, 0x1A06};

    retval = ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(map_position_1c12),
                         map_position_1c12, EC_TIMEOUTSAFE);
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(map_position_1c13),
                          map_position_1c13, EC_TIMEOUTSAFE);
    if (retval != 2)
      return 0;
  } else if (state.mode == operational_mode_t::CSV) {

    // CSV
    u8val = 8;
    retval = ec_SDOwrite(slave, 0x7010, 0x0003, FALSE, sizeof(u8val), &u8val,
                         EC_TIMEOUTSAFE);
    if (retval == 0)
      return 0;

    uint16_t map_velocity_1c12[] = {0x0002, 0x1600, 0x1606};
    uint16_t map_velocity_1c12[] = {0x0005, 0x1A00, 0x1A01,
                                    0x1A02, 0x1A03, 0x1A06};

    retval = ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(map_velocity_1c12),
                         map_velocity_1c12, EC_TIMEOUTSAFE);
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(map_velocity_1c13),
                          map_velocity_1c13, EC_TIMEOUTSAFE);
    if (retval != 2)
      return 0;
  }
  else {
    // TODO CST not supported yet
    assert(0 && "Mode not supported");
  }

  // Motor Settings

  // Set max current
  u32val = 5000;
  retval = ec_SDOwrite(slave, 0x8011, 0x11, FALSE, sizeof(u32val), &u32val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set rated current
  u32val = 1000;
  retval = ec_SDOwrite(slave, 0x8011, 0x12, FALSE, sizeof(u32val), &u32val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set motor pole pairs
  u8val = 3;
  retval = ec_SDOwrite(slave, 0x8011, 0x13, FALSE, sizeof(u8val), &u8val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set torque constant
  u32val = 100;
  retval = ec_SDOwrite(slave, 0x8011, 0x16, FALSE, sizeof(u32val), &u32val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set rotor moment of intertia
  u32val = 253;
  retval = ec_SDOwrite(slave, 0x8011, 0x18, FALSE, sizeof(u32val), &u32val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set winding inductance
  u16val = 70;
  retval = ec_SDOwrite(slave, 0x8011, 0x19, FALSE, sizeof(u16val), &u16val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set motor speed limitation
  u32val = 14000;
  retval = ec_SDOwrite(slave, 0x8011, 0x1b, FALSE, sizeof(u32val), &u32val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set motor temperature warn level
  u16val = 1200;
  retval = ec_SDOwrite(slave, 0x8011, 0x2b, FALSE, sizeof(u16val), &u16val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set motor temperature error level
  u16val = 1400;
  retval = ec_SDOwrite(slave, 0x8011, 0x2c, FALSE, sizeof(u16val), &u16val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set motor thermal time constant
  u16val = 1050;
  retval = ec_SDOwrite(slave, 0x8011, 0x2d, FALSE, sizeof(u16val), &u16val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set winding resistance
  u32val = 360;
  retval = ec_SDOwrite(slave, 0x8011, 0x30, FALSE, sizeof(u32val), &u32val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set voltage constant
  u32val = 5500;
  retval = ec_SDOwrite(slave, 0x8011, 0x31, FALSE, sizeof(u32val), &u32val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set commutation offset
  u16val = 270;
  retval = ec_SDOwrite(slave, 0x8011, 0x15, FALSE, sizeof(u16val), &u16val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Outputs

  // Set torque limitation
  u16val = 2250;
  retval = ec_SDOwrite(slave, 0x7010, 0x0b, FALSE, sizeof(u16val), &u16val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Amplifier Settings

  // Set current loop P gain
  u16val = 39;
  retval = ec_SDOwrite(slave, 0x8010, 0x13, FALSE, sizeof(u16val), &u16val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set current loop I gain
  u16val = 19;
  retval = ec_SDOwrite(slave, 0x8010, 0x12, FALSE, sizeof(u16val), &u16val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set velocity loop P gain
  u32val = 224;
  retval = ec_SDOwrite(slave, 0x8010, 0x15, FALSE, sizeof(u32val), &u32val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // 0x8010:17
  u32val = 1;
  retval = ec_SDOwrite(slave, 0x8010, 0x17, FALSE, sizeof(u32val), &u32val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Set velocity loop I gain
  u32val = 150;
  retval = ec_SDOwrite(slave, 0x8010, 0x14, FALSE, sizeof(u32val), &u32val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // FB Settings

  // Set observer feed-forward
  u8val = 100;
  retval = ec_SDOwrite(slave, 0x8000, 0x15, FALSE, sizeof(u8val), &u8val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  // Amplifier Settings

  // Set nominal DC link voltage
  u32val = 24000;
  retval = ec_SDOwrite(slave, 0x8010, 0x19, FALSE, sizeof(u32val), &u32val,
                       EC_TIMEOUTSAFE);
  if (retval == 0)
    return 0;

  return 1;
}