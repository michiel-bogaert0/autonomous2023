import ctypes
import os
from ctypes import POINTER, c_byte, c_ubyte, c_ushort
from functools import partial

import node_fixture.managed_node as mn
import portio
import rospy
from std_msgs.msg import Bool

bytep = POINTER(c_ubyte)
ushortp = POINTER(c_ushort)


class DIODriver(mn.ManagedNode):
    def __init__(self, name: str):
        """Driver for DIO (and iPC by extension)

        See documentation of the iPC (ECX2200), appendix B
        for more information about the API calls

        Args:
            name (str): Name of the node
        """
        super().__init__(name, mn.NodeManagingStatesEnum.ACTIVE)

        # Random stuff and variables
        portio.iopl(3)
        self.subscribers = []
        self.publishers = []
        self.enabled_DO = []
        self.enabled_DI = []

        # Load Vecow DLL
        dirname, _ = os.path.split(os.path.abspath(__file__))
        os.chdir(dirname)

        self.dll = ctypes.cdll.LoadLibrary("./libvecow.so")
        self.dll.initial_SIO.restype = ctypes.c_bool
        self.dll.initial_SIO.argtype = [ctypes.c_byte, ctypes.c_byte]

        self.dll.get_IO1_configuration.restype = ctypes.c_bool
        self.dll.get_IO1_configuration.argtype = [
            POINTER(ctypes.c_byte),
            POINTER(ctypes.c_byte),
            POINTER(ctypes.c_byte),
            POINTER(ctypes.c_ushort),
        ]
        self.dll.get_IO2_configuration.restype = ctypes.c_bool
        self.dll.get_IO2_configuration.argtype = [
            POINTER(ctypes.c_byte),
            POINTER(ctypes.c_byte),
            POINTER(ctypes.c_byte),
            POINTER(ctypes.c_ushort),
        ]

        self.dll.set_IO1_configuration.restype = ctypes.c_bool
        self.dll.set_IO1_configuration.argtype = [
            ctypes.c_byte,
            ctypes.c_byte,
            ctypes.c_byte,
            ctypes.c_ushort,
        ]
        self.dll.set_IO2_configuration.restype = ctypes.c_bool
        self.dll.set_IO2_configuration.argtype = [
            ctypes.c_byte,
            ctypes.c_byte,
            ctypes.c_byte,
            ctypes.c_ushort,
        ]

        self.dll.get_DIO1.restype = ctypes.c_bool
        self.dll.get_DIO1.argtype = [POINTER(ctypes.c_byte), POINTER(ctypes.c_byte)]
        self.dll.get_DIO2.restype = ctypes.c_bool
        self.dll.get_DIO2.argtype = [POINTER(ctypes.c_byte), POINTER(ctypes.c_byte)]

        self.dll.set_DIO1.restype = ctypes.c_bool
        self.dll.set_DIO1.argtype = [ctypes.c_byte, ctypes.c_byte]
        self.dll.set_DIO2.restype = ctypes.c_bool
        self.dll.set_DIO2.argtype = [ctypes.c_byte, ctypes.c_byte]

        # Initialize IO "as isolated sources" (not sure if needed)
        if not self.dll.initial_SIO(1, 1):
            rospy.logerr("Failed to initialize DIO!")
            exit(-1)

        self.spin()

    def doCleanup(self):
        """
        Cleans up when required
        """

        # Reset
        for sub in self.subscribers:
            sub.unregister()
        for pub in self.publishers:
            pub.unregister()

    def onConfigure(self):
        """
        Runs when the node is being configured
        """

        self.doCleanup()

        # First make sure the dio bank is configured
        try:
            self.dio_bank = rospy.get_param("bank")
        except KeyError:
            # No bank set. Exit
            rospy.logerr("No bank set for DIO driver. Exiting.")
            exit(-1)

        self.set_DIO = self.dll.set_DIO1 if self.dio_bank == 1 else self.dll.set_DIO2
        self.get_DIO = self.dll.get_DIO1 if self.dio_bank == 1 else self.dll.get_DIO2
        self.get_IO_configuration = (
            self.dll.get_IO1_configuration
            if self.dio_bank == 1
            else self.dll.get_IO2_configuration
        )
        self.set_IO_configuration = (
            self.dll.set_IO1_configuration
            if self.dio_bank == 1
            else self.dll.set_IO2_configuration
        )

        # Get config
        for i in range(8):
            if rospy.get_param(f"enable_DO{i}", False):
                self.enabled_DO.append(i)
            if rospy.get_param(f"enable_DI{i}", False):
                self.enabled_DI.append(i)

        # Make subscribers for the DO
        for i in self.enabled_DO:
            self.subscribers.append(
                rospy.Subscriber(
                    f"/output{i}",
                    Bool,
                    partial(self.set_digital_out, pin_nr=i),
                    queue_size=1,
                )
            )

        # Make publishers for the DI
        for i in self.enabled_DI:
            self.publishers.append(
                rospy.Publisher(
                    f"/input{i}",
                    Bool,
                    queue_size=1,
                )
            )

        # Setup stuff (isolated sources)
        self.set_IO_configuration(
            c_byte(int(1)),
            c_byte(int("0xFF", 0)),
            c_byte(int(1)),
            ushortp(c_ushort(0)),
        )

    def active(self):
        """
        Active loop. Basically reads out DI and publishes enabled ones
        """

        # Fetch and publish the DI
        bytepin_in = bytep(c_ubyte(0))
        bytepin_out = bytep(c_ubyte(0))

        self.get_DIO(bytepin_out, bytepin_in)

        pin = bytepin_in[0]

        for i, pin_nr in enumerate(self.enabled_DI):
            self.publishers[i].publish(Bool(data=(pin >> pin_nr) & 1))

    def set_digital_out(self, msg: Bool, pin_nr: int):
        """Sets the digital output

        Args:
            msg (Bool): boolean value to set
            pin_nr (int): pin number to use. Should be set automatically
        """

        # Get current config
        bytepin_in = bytep(c_ubyte(0))  # Status of inputs
        bytepin_out = bytep(c_ubyte(0))  # Status of outputs (doesn't matter)

        self.get_DIO(bytepin_out, bytepin_in)

        pin = bytepin_out[0]

        # Now set the pin by masking
        if msg.data:
            pin |= 1 << (pin_nr - 1)
        else:
            pin &= ~(1 << (pin_nr - 1))

        self.set_DIO(pin)


node = DIODriver("dio_driver")
