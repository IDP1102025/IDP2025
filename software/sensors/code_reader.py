import struct
from time import sleep,time
import machine


class CodeReader:
    TINY_CODE_READER_I2C_ADDRESS = 0x0C
    TINY_CODE_READER_DELAY = 0.05
    TINY_CODE_READER_LENGTH_FORMAT = "H"
    TINY_CODE_READER_MESSAGE_SIZE = 254
    TINY_CODE_READER_MESSAGE_FORMAT = "B" * TINY_CODE_READER_MESSAGE_SIZE
    TINY_CODE_READER_I2C_FORMAT = TINY_CODE_READER_LENGTH_FORMAT + TINY_CODE_READER_MESSAGE_FORMAT
    TINY_CODE_READER_I2C_BYTE_COUNT = struct.calcsize(TINY_CODE_READER_I2C_FORMAT)

    def __init__(self, scl_pin, sda_pin, freq=400000):
        """
        Initializes the CodeReader instance.
        
        :param scl_pin: Pin number for SCL
        :param sda_pin: Pin number for SDA
        :param freq: Frequency for I2C communication
        """
        self.scl_pin = scl_pin
        self.sda_pin = sda_pin

        # Set up the I2C interface
        self.i2c = machine.I2C(1,
                               scl=machine.Pin(self.scl_pin),
                               sda=machine.Pin(self.sda_pin))
        self.data = None

    def read_data(self):
        """
        Reads data from the code reader using I2C.
        :return: A tuple containing the length and message data, or None if there's an error
        """
        try:
            # Read the data from the device
            raw_data = self.i2c.readfrom(self.TINY_CODE_READER_I2C_ADDRESS, self.TINY_CODE_READER_I2C_BYTE_COUNT)
            
            # Unpack the data based on the format
            length = struct.unpack_from(self.TINY_CODE_READER_LENGTH_FORMAT, raw_data, 0)[0]
            message = struct.unpack_from(self.TINY_CODE_READER_MESSAGE_FORMAT, raw_data, struct.calcsize(self.TINY_CODE_READER_LENGTH_FORMAT))
            
            # Return the length and the message
            return length, message[:length]
        except Exception as e:
            print(f"Error reading data: {e}")
            return None

    def poll_for_code(self, timeout):
        """
        Polls the sensor for a code for a certain number of seconds.
        
        :param timeout: Maximum number of seconds to poll for.
        :return: Unpacked data if a code is found, or None if no code is found.
        """
        self.data = None # Reset data 
        start_time = time()
        while (time() - start_time) < timeout:
            data = self.read_data()
            if data:
                length, message = data
                if length > 0:  # If a valid code is found
                    return message
            sleep(self.TINY_CODE_READER_DELAY) # Delay before polling again
        return None
    
