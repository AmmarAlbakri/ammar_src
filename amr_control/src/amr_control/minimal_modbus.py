import minimalmodbus

class MotorDevice:

    def __init__(self, port,baudrate):
        debug_mode = False  # False
        self.motor_right = minimalmodbus.Instrument(port, slaveaddress=2,
                                                    debug=debug_mode)  # port name, slave address (in decimal)
        self.motor_left = minimalmodbus.Instrument(port, slaveaddress=1,
                                                   debug=debug_mode)  # port name, slave address (in decimal)

        self.motor_right.serial.baudrate = baudrate
        self.motor_left.serial.baudrate = baudrate

    def read_status_register(self, register_address, motor_name):
        if motor_name == "right":
            value = self.motor_right.read_register(register_address, 0)
        else:
            value = self.motor_left.read_register(register_address, 0)
        return value

    def write_function_register(self, register_address, motor_name, value):
        if motor_name == "right":
            self.motor_right.write_register(
                register_address, value, functioncode=6)
        else:
            self.motor_left.write_register(
                register_address, value, functioncode=6)

    def read_speed_status(self, motor_name):
        return self.read_status_register(0, motor_name)

    def read_frequency_status(self, motor_name):
        return self.read_status_register(1, motor_name)

    def read_torque_status(self, motor_name):
        return self.read_status_register(2, motor_name)

    def read_rotor_position_status(self, motor_name):
        return self.read_status_register(4, motor_name)

    def read_speed_command_status(self, motor_name):
        return self.read_status_register(2, motor_name)

    def read_encoder_absolute_status(self, motor_name):
        return self.read_status_register(29, motor_name) + self.read_status_register(30, motor_name) * 65536

    def write_speed_command(self, motor_name, value):
        if(value < 0):
            value = 65535 - abs(value)
        self.write_function_register(20000, motor_name, value)

    def stop(self):
        self.write_speed_command("right", 0)
        self.write_speed_command("left", 0)

class LiftDevice:
    def __init__(self, port,baudrate):
        debug_mode = False  # False
        self.lift = minimalmodbus.Instrument(port, slaveaddress=1,
                                              debug=debug_mode)  # port name, slave address (in decimal)

        self.lift.serial.baudrate = baudrate

    def read_status_register(self, register_address):
        value = self.lift.read_register(register_address, 0)
        return value

    def write_function_register(self, register_address,  value):
        self.lift.write_register(register_address, value, functioncode=6)

    def read_speed_status(self):
        return self.read_status_register(0)

    def read_frequency_status(self):
        return self.read_status_register(1)

    def read_torque_status(self):
        return self.read_status_register(2)

    def read_rotor_position_status(self):
        return self.read_status_register(4)

    def read_speed_command_status(self):
        return self.read_status_register(2)

    def read_encoder_absolute_status(self):
        return self.read_status_register(29) + self.read_status_register(30) * 65536

    def read_encoder_absolute_status_Deneme(self, motor_name):
        return self.read_status_register(29, motor_name)

    def write_speed_command(self,  value):
        if(value < 0):
            value = 65535 - abs(value)
        self.write_function_register(20000,  value)

    def stop(self):
        self.write_speed_command(0)
