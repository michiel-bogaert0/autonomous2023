import ctypes
import os
from ctypes import POINTER

import node_fixture.managed_node as mn
import portio
import rospy


class DIODriver(mn.ManagedNode):
    def __init__(self, name: str):
        """Driver for DIO (and iPC by extension)

        See documentation of the iPC (ECX2200), appendix B
        for more information about the API calls

        Args:
            name (str): Name of the node
        """
        super().__init__(name)

        portio.iopl(3)

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

    def onConfigure(self):
        # Get bank ID

        pass


node = DIODriver("dio_driver")
