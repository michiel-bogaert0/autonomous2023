#pragma once

#include <dlfcn.h>
#include <stdio.h> // printf, fgets, sscanf
#include <stdlib.h>
#include <sys/io.h> // iopl
#include <unistd.h> // usleep

#define Sleep(_t_) usleep((_t_)*1000)

#if defined(__GNUC__)
#include <inttypes.h>

#if !defined(BOOL)
#define BOOL int
#define TRUE 1
#define FALSE 0
#endif
#if !defined(BYTE)
#define BYTE uint8_t
#endif
#if !defined(WORD)
#define WORD uint16_t
#endif
#if !defined(DWORD)
#define DWORD uint32_t
#endif
#if !defined(LONG)
#define LONG int32_t
#endif

#if !defined(UINT8)
#define UINT8 uint8_t
#endif
#if !defined(UINT16)
#define UINT16 uint16_t
#endif
#if !defined(UINT32)
#define UINT32 uint32_t
#endif
#if !defined(UINT64)
#define UINT64 uint64_t
#endif

#elif defined(__TURBOC__)
#if !defined(BOOL)
#define BOOL int
#define TRUE 1
#define FALSE 0
#endif
#if !defined(BYTE)
#define BYTE unsigned char
#endif
#if !defined(WORD)
#define WORD unsigned short
#endif
#if !defined(DWORD)
#define DWORD unsigned long
#endif
#if !defined(LONG)
#define LONG long
#endif
#endif

extern "C" {
BOOL get_BIOS(BYTE *REBIOS);
BOOL get_revision(BYTE *PCB_REVISION);
BOOL initial_SIO(BYTE Isolate_Type, BYTE DIO_NPN);
BOOL initial_SIO_path(BYTE Isolate_Type, BYTE DIO_NPN, const char *filename);
BOOL set_IO1_configuration(BYTE Iso, BYTE DI_mode, BYTE DO_mode, WORD Mask);
BOOL set_IO2_configuration(BYTE Iso, BYTE DI_mode, BYTE DO_mode, WORD Mask);
BOOL get_IO1_configuration(BYTE *Iso, BYTE *DI_mode, BYTE *DO_mode, WORD *Mask);
BOOL get_IO2_configuration(BYTE *Iso, BYTE *DI_mode, BYTE *DO_mode, WORD *Mask);
BOOL set_DIO1(BYTE DO_data);
BOOL set_DIO2(BYTE DO_data);
BOOL set_IO1_configuration_other(BYTE Iso, BYTE DI_mode, BYTE DO_mode,
                                 DWORD Mask);
BOOL set_IO2_configuration_other(BYTE Iso, BYTE DI_mode, BYTE DO_mode,
                                 DWORD Mask);
BOOL get_IO1_configuration_other(BYTE *Iso, BYTE *DI_mode, BYTE *DO_mode,
                                 DWORD *Mask);
BOOL get_IO2_configuration_other(BYTE *Iso, BYTE *DI_mode, BYTE *DO_mode,
                                 DWORD *Mask);
BOOL set_DIO1_other(DWORD DO_data);
BOOL set_DIO2_other(DWORD DO_data);
BOOL set_GPIO1(WORD GPIO_data);
BOOL set_GPIO2(WORD GPIO_data);

BOOL get_DIO1(BYTE *DO_data, BYTE *DI_data);
BOOL get_DIO2(BYTE *DO_data, BYTE *DI_data);
BOOL get_DIO1_other(DWORD *DO_data, DWORD *DI_data);
BOOL get_DIO2_other(DWORD *DO_data, DWORD *DI_data);
BOOL get_GPIO1(WORD *GPIO_data);
BOOL get_GPIO2(WORD *GPIO_data);
BOOL set_WDT(DWORD WDT);
BOOL get_WDT(DWORD *WDT);
BOOL cancel_WDT();
BOOL config_COMPORT(BYTE *PORT_NUM);
BOOL set_COMPORT_mode(BYTE port, BYTE mode, BYTE term);
BOOL get_COMPORT_mode(BYTE port, BYTE *mode, BYTE *term);
BOOL get_EC_base_address();
BOOL get_FAN_tachometer(WORD *FanTachometer);
BOOL get_FAN(BYTE *PWM);
BOOL set_FAN(BYTE PWM);
BOOL smart_FAN(BYTE FAN_support, BYTE FAN_mode, BYTE start_PWM,
               BYTE temp_FAN_start, BYTE temp_FAN_full_speed);
BOOL write_SMBus_byte(BYTE i2c_addr, BYTE cmd, BYTE data);
BOOL read_SMBus_byte(BYTE i2c_addr, BYTE cmd, BYTE *data);
BOOL initial_SMBUS();
BOOL initial_POE(BYTE Scan, BYTE ID);
BOOL get_POE_configuration(BYTE ID, BYTE *Auto, BYTE *Mask);
BOOL set_POE_configuration(BYTE ID, BYTE Auto, BYTE Mask);
BOOL get_POE_configuration_ip804(BYTE ID, BYTE *POE_Mode);
BOOL get_POE(BYTE ID, BYTE *POE);
BOOL get_POE_ip804(BYTE ID, BYTE *POE);
BOOL set_POE(BYTE ID, BYTE POE);
BOOL initial_POEW(BYTE Scan, BYTE ID, BYTE *POE_PORT);

BOOL set_POEW_configuration(BYTE ID, BYTE *Auto, BYTE *Mask);
BOOL set_POE_configuration_ip804(BYTE ID, BYTE *POE_Mode, BYTE *POE_Mask);
BOOL set_POEW(BYTE ID, BYTE *POEW);
BOOL get_POE_power(BYTE ID, BYTE *PORT_POWER);
BOOL initial_USB();
BOOL get_USB(BYTE ID, BYTE *USB);
BOOL set_USB(BYTE ID, BYTE USB);
BOOL get_CPU_voltage(BYTE *VIN0);
BOOL initial_SPEAKER(int hz);
BOOL beep(unsigned int duration);
BOOL return_dll_version(BYTE *MAJOR, BYTE *MINOR, WORD *BUILD, WORD *REVISION);

// CPU Teperature
BOOL get_CPU_temperature(BYTE *temperature);

// Voltages: VCORE, VCC_DDR, +12V, +5V, +3.3V
BOOL get_voltages(float *vin);
} /* extern "C" */
