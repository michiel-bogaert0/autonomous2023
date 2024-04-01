import ctypes
from ctypes import POINTER, c_byte, c_ubyte, c_ushort

import portio

portio.iopl(3)
mydll = ctypes.cdll.LoadLibrary("./libvecow.so")


mydll.initial_SIO.restype = ctypes.c_bool
mydll.initial_SIO.argtype = [ctypes.c_byte, ctypes.c_byte]

mydll.get_IO1_configuration.restype = ctypes.c_bool
mydll.get_IO1_configuration.argtype = [
    POINTER(ctypes.c_byte),
    POINTER(ctypes.c_byte),
    POINTER(ctypes.c_byte),
    POINTER(ctypes.c_ushort),
]
mydll.get_IO2_configuration.restype = ctypes.c_bool
mydll.get_IO2_configuration.argtype = [
    POINTER(ctypes.c_byte),
    POINTER(ctypes.c_byte),
    POINTER(ctypes.c_byte),
    POINTER(ctypes.c_ushort),
]

mydll.set_IO1_configuration.restype = ctypes.c_bool
mydll.set_IO1_configuration.argtype = [
    ctypes.c_byte,
    ctypes.c_byte,
    ctypes.c_byte,
    ctypes.c_ushort,
]
mydll.set_IO2_configuration.restype = ctypes.c_bool
mydll.set_IO2_configuration.argtype = [
    ctypes.c_byte,
    ctypes.c_byte,
    ctypes.c_byte,
    ctypes.c_ushort,
]

mydll.get_DIO1.restype = ctypes.c_bool
mydll.get_DIO1.argtype = [POINTER(ctypes.c_byte), POINTER(ctypes.c_byte)]
mydll.get_DIO2.restype = ctypes.c_bool
mydll.get_DIO2.argtype = [POINTER(ctypes.c_byte), POINTER(ctypes.c_byte)]

mydll.set_DIO1.restype = ctypes.c_bool
mydll.set_DIO1.argtype = [ctypes.c_byte, ctypes.c_byte]
mydll.set_DIO2.restype = ctypes.c_bool
mydll.set_DIO2.argtype = [ctypes.c_byte, ctypes.c_byte]


re = mydll.initial_SIO(1, 0)

if re is False:
    print("Initial SIO fail!\n")
else:
    bytep = POINTER(c_ubyte)

    iso = bytep(c_ubyte(0))
    di_mode1 = bytep(c_ubyte(0))
    do_mode1 = bytep(c_ubyte(0))
    ushortp = POINTER(c_ushort)
    mask1 = ushortp(c_ushort(0))
    print("Initial SIO succeed!\n")
    cin = input("Choose IO : (1/2)")
    if cin == "1":
        re = mydll.get_IO1_configuration(iso, di_mode1, do_mode1, mask1)
        if re:
            ISO_Non_in = input("Select Non-Isolated/Isolated mode : (0/1) ")
            print("get IO1 config succeed!")
            di_sinksource_in = input("Select DI Sink/Source mode : (0~FF) ")
            do_sinksource_in = input("Select D0 Sink/Source mode : (0/1) ")
            mydll.set_IO1_configuration(
                c_byte(int(ISO_Non_in)),
                c_byte(int("0x" + di_sinksource_in, 0)),
                c_byte(int(do_sinksource_in)),
                mask1,
            )

            bytepin_in = bytep(c_ubyte(0))
            bytepin_out = bytep(c_ubyte(0))
            do_pin = input("Choose Do pin : (1~8), 9 = ALL :")
            mydll.get_DIO1(bytepin_out, bytepin_in)
            pin = bytepin_out[0]
            if do_sinksource_in == "1":
                hilo_pin = input("Set DO Low/High: (0/1) ")
            elif do_sinksource_in == "0":
                hilo_pin = input("Set DO Low/High: (1/0) ")
            if do_pin != "9":
                if hilo_pin == "1":
                    print(1 << (int(do_pin) - 1))
                    pin |= 1 << (int(do_pin) - 1)
                else:
                    pin &= ~(1 << (int(do_pin) - 1))
            else:
                pin = 0xFF if hilo_pin == "1" else 0
            mydll.set_DIO1(pin)
    elif cin == "2":
        re = mydll.get_IO2_configuration(iso, di_mode1, do_mode1, mask1)
        if re:
            ISO_Non_in = input("Select Non-Isolated/Isolated mode : (0/1) ")
            print("get IO2 config succeed!")
            di_sinksource_in = input("Select DI Sink/Source mode : (0~FF) ")
            do_sinksource_in = input("Select D0 Sink/Source mode : (0/1) ")
            mydll.set_IO2_configuration(
                c_byte(int(ISO_Non_in)),
                c_byte(int("0x" + di_sinksource_in, 0)),
                c_byte(int(do_sinksource_in)),
                mask1,
            )

            bytepin_in = bytep(c_ubyte(0))
            bytepin_out = bytep(c_ubyte(0))
            do_pin = input("Choose Do pin : (1~8), 9 = ALL :")
            mydll.get_DIO2(bytepin_out, bytepin_in)
            pin = bytepin_out[0]
            if do_sinksource_in == "1":
                hilo_pin = input("Set DO Low/High: (0/1) ")
            elif do_sinksource_in == "0":
                hilo_pin = input("Set DO Low/High: (1/0) ")

            if do_pin != "9":
                if hilo_pin == "1":
                    print(1 << (int(do_pin) - 1))
                    pin |= 1 << (int(do_pin) - 1)
                else:
                    pin &= ~(1 << (int(do_pin) - 1))
            else:
                pin = 0xFF if hilo_pin == "1" else 0
            mydll.set_DIO2(pin)
