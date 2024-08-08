from machine import UART, Pin
import time
import select
import sys

# poll_obj = select.poll()
# poll_obj.register(sys.stdin, select.POLLIN)
# 
# data=0
class RMCS:
    def __init__(self, uart_num, baudrate, tx_pin, rx_pin):
        self.uart = UART(uart_num, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin))
        self.max_byte_array_size = 8
        self.byte_array = bytearray(self.max_byte_array_size)
        self.sprt = None

    def serial_selection(self, x):
        self.sprt = x

    def begin(self, baudrate):
        self.uart.init(baudrate=baudrate)

    def enable_analog_mode(self, slave_id):
        return self.write_single_register(slave_id, 2, 1)

    def disable_analog_mode(self, slave_id):
        return self.write_single_register(slave_id, 2, 0)

    def enable_digital_mode(self, slave_id, direction):
        data = 257 if direction == 0 else 265
        return self.write_single_register(slave_id, 2, data)

    def disable_digital_mode(self, slave_id, direction):
        data = 256 if direction == 0 else 264
        return self.write_single_register(slave_id, 2, data)

    def absolute_position(self, slave_id, count):
        if self.absolute_move(slave_id, count):
            if self.encoder_position_enable(slave_id):
                return 1
        return 0

    def speed(self, slave_id, speed):
        return self.write_single_register(slave_id, 14, speed)

    def speed_feedback(self, slave_id):
        result = self.read_single_register(slave_id, 24, 1)
        q = self.value(result)
        return q - 65535 if q > 32765 else q

    def brake_motor(self, slave_id, direction):
        data = 260 if direction == 0 else 268
        return self.write_single_register(slave_id, 2, data)

    def encoder_position_lpr(self, slave_id, lpr):
        return self.write_single_register(slave_id, 10, lpr)

    def disable_position_mode(self, slave_id):
        return self.write_single_register(slave_id, 2, 512)

    def encoder_position_enable(self, slave_id):
        return self.write_single_register(slave_id, 2, 513)

    def absolute_move(self, slave_id, count):
        data = count if count >= 0 else 4294967295 - (-count)
        hidata = data >> 16
        lodata = data & 0xFFFF
        if self.write_single_register(slave_id, 16, lodata):
            if self.write_single_register(slave_id, 18, hidata):
                return 1
        return 0

    def acceleration(self, slave_id, acceleration):
        return self.write_single_register(slave_id, 12, acceleration)

    def proportional_gain(self, slave_id, pp_gain):
        return self.write_single_register(slave_id, 4, pp_gain)

    def integral_gain(self, slave_id, pi_gain):
        return self.write_single_register(slave_id, 6, pi_gain)

    def feedforward_gain(self, slave_id, vf_gain):
        return self.write_single_register(slave_id, 8, vf_gain)

    def position_feedback(self, slave_id):
        result = self.read_single_register(slave_id, 20, 2)
        x = result[7:15]
        x = self.swap_nibbles(x)
        q = int(x, 16)
        return q - 4294967295 if q > 2147483647 else q

    def write_single_register(self, slave_id, address, value):
        result = self.modbus_string(slave_id, 6, address, value)
        self.uart.write(result)
        time.sleep(0.1)
        response = self.uart.read()
        if not response:
            print("CHECK YOUR SLAVE_ID")
            return 0
        res = response[1:17].decode()
        if self.lrc(res, 6):
            return 1
        return 0

    def modbus_string(self, slave_id, fc, address, data):
        add_lo_byte = address & 0x00FF
        add_hi_byte = (address & 0xFF00) >> 8
        data_lo_byte = data & 0x00FF
        data_hi_byte = (data & 0xFF00) >> 8
        sum_ = -(slave_id + fc + add_lo_byte + add_hi_byte + data_lo_byte + data_hi_byte)
        result = f":{self.print_hex(slave_id)}{self.print_hex(fc)}{self.print_hex(add_hi_byte)}{self.print_hex(add_lo_byte)}{self.print_hex(data_hi_byte)}{self.print_hex(data_lo_byte)}{self.print_hex(sum_)}\r\n"
        return result

    def print_hex(self, number):
        value = hex(number & 0xFF)[2:].upper()
        return value if len(value) > 1 else '0' + value

    def read_single_register(self, slave_id, address, num_registers):
        result = self.modbus_string(slave_id, 3, address, num_registers)
        self.uart.write(result)
        time.sleep(0.1)
        response = self.read_string_until(self.uart, '\n')
        if response:
            res = response[1:18]
            if self.lrc(res, 7):
                return response
        print("CHECK YOUR SLAVE_ID")
        return "0"

    def value(self, input_):
        x = input_[7:-3]
        try:
            return abs(int(x, 16))
        except ValueError:
            print(f"Invalid hex string: {x}")
            return 0

    def lrc(self, s, length):
        byte_array = self.hex_to_bytes(s)
        sum_ = sum(byte_array[:length])
        lrc = (~sum_ + 1) & 0xFF
        return lrc == byte_array[length]

    def hex_to_bytes(self, hex_string):
        hex_string = ''.join([x for x in hex_string if x in '0123456789ABCDEFabcdef'])
        byte_array = bytearray(self.max_byte_array_size)
        for i in range(0, len(hex_string), 2):
            try:
                byte_array[i // 2] = int(hex_string[i:i + 2], 16)
            except ValueError:
                print(f"Invalid hex character in string: {hex_string[i:i + 2]}")
                return None
        return byte_array

    def swap_nibbles(self, input_str):
        return ''.join(input_str[i:i + 2][::-1] for i in range(0, len(input_str), 2))

    def read_pp_gain(self, slave_id):
        result = self.read_single_register(slave_id, 10, 0)
        q = self.value(result)
        print(q)
        return q

    def read_string_until(self, uart, terminator='\n'):
        response = ''
        while True:
            char = uart.read(1)
            if char is None:
                continue
            char = char.decode('utf-8')
            if char == terminator:
                break
            response += char
        return response

    def write_parameter(self, slave_id, inp_control_mode, pp_gain, pi_gain, vf_gain, lpr, acceleration, speed):
        results = [
            self.write_single_register(slave_id, 2, inp_control_mode),
            self.proportional_gain(slave_id, pp_gain),
            self.integral_gain(slave_id, pi_gain),
            self.feedforward_gain(slave_id, vf_gain),
            self.encoder_position_lpr(slave_id, lpr),
            self.acceleration(slave_id, acceleration),
            self.speed(slave_id, speed)
        ]
        if all(results):
            if self.save(slave_id):
                print("ALL PARAMETERS SET")
            else:
                print("NOT SAVED")
        else:
            print("ERROR IN WRITING PARAMETERS")

    def read_parameter(self, slave_id):
        params = [
            ("DEVICE_MODBUS_ADDRESS", self.read_device_modbus_address(slave_id)),
            ("INP_CONTROL_BYTE", self.read_inp_control_byte(slave_id)),
            ("INP_MODE_BYTE", self.read_inp_mode_byte(slave_id)),
            ("PP_GAIN_BYTE", self.read_pp_gain_byte(slave_id)),
            ("PI_GAIN_BYTE", self.read_pi_gain_byte(slave_id)),
            ("VF_GAIN_BYTE", self.read_vf_gain_byte(slave_id)),
            ("LINES_PER_ROT", self.read_lines_per_rot(slave_id)),
            ("TRP_ACL_WORD", self.read_trp_acl_word(slave_id)),
            ("TRP_SPD_WORD", self.read_trp_spd_word(slave_id))
        ]
        for name, value in params:
            if value >= 0:
                print(f"{name}: {value}")
            else:
                print("ERROR IN READING")
                return

    def read_device_modbus_address(self, slave_id):
        return self.read_single_register_value(slave_id, 1)

    def read_inp_control_byte(self, slave_id):
        return self.read_single_register_value(slave_id, 2)

    def read_inp_mode_byte(self, slave_id):
        return self.read_single_register_value(slave_id, 3)

    def read_pp_gain_byte(self, slave_id):
        return self.read_single_register_value(slave_id, 4)

    def read_pi_gain_byte(self, slave_id):
        return self.read_single_register_value(slave_id, 6)

    def read_vf_gain_byte(self, slave_id):
        return self.read_single_register_value(slave_id, 8)

    def read_lines_per_rot(self, slave_id):
        return self.read_single_register_value(slave_id, 10)

    def read_trp_acl_word(self, slave_id):
        return self.read_single_register_value(slave_id, 12)

    def read_trp_spd_word(self, slave_id):
        return self.read_single_register_value(slave_id, 14)

    def read_single_register_value(self, slave_id, address):
        result = self.read_single_register(slave_id, address, 1)
        if result:
            x = result[9:11]
            try:
                return int(x, 16)
            except ValueError:
                print(f"Invalid hex string: {x}")
                return -1
        return -1

    def save(self, slave_id):
        save_value = (slave_id << 8) | 0xFF
        return self.write_single_register(slave_id, 0, save_value)

    def reset(self, slave_id):
        if self.write_single_register(slave_id, 4, 0):
            return self.save(slave_id)
        return 0

    def estop(self, slave_id):
        return self.write_single_register(slave_id, 2, 1792)

    def stop(self, slave_id):
        return self.write_single_register(slave_id, 2, 1793)

    def set_home(self, slave_id):
        return self.write_single_register(slave_id, 2, 2048)

    def restart(self, slave_id):
        return self.write_single_register(slave_id, 2, 2304)

# Example usage
if __name__ == "__main__":
    rmcs1 = RMCS(1, 9600, 4, 5)
    slave_id1 = 7
    pp_gain1 = 32
    pi_gain1 = 16
    vf_gain1 = 32
    lpr1 = 2262
    acceleration1 = 45
    speed1 = 90
    
    rmcs1.write_parameter(7,1,pp_gain1,pi_gain1,vf_gain1,lpr1,acceleration1,speed1)

    rmcs2 = RMCS(0, 9600, 12, 13)
    slave_id2 = 7
    pp_gain2 = 32
    pi_gain2 = 16
    vf_gain2 = 32
    lpr2 = 2262
    acceleration2 = 45
    speed2 = 90
    
    rmcs2.write_parameter(7,1,pp_gain2,pi_gain2,vf_gain2,lpr2,acceleration2,speed2)

#def write_parameter(self, slave_id, inp_control_mode, pp_gain, pi_gain, vf_gain, lpr, acceleration, speed):

#     rmcs1.integral_gain(slave_id1, pi_gain1)
#     rmcs1.feedforward_gain(slave_id1, vf_gain1)
#     rmcs1.encoder_position_lpr(slave_id1, lpr1)
# 
#     rmcs2.integral_gain(slave_id2, pi_gain2)
#     rmcs2.feedforward_gain(slave_id2, vf_gain2)
#     rmcs2.encoder_position_lpr(slave_id2, lpr2)

    print("Parameters for motor1")
    rmcs1.read_parameter(slave_id1)
    print("Parameters for motor2")
    rmcs2.read_parameter(slave_id2)
    time.sleep(2)
    
    

    for i in range(100):
        rmcs1.enable_digital_mode(slave_id1, 0)
        rmcs2.enable_digital_mode(slave_id2, 1)
        rmcs1.acceleration(slave_id1, acceleration1)
        rmcs2.acceleration(slave_id2, acceleration2)

        print("Sending speed command - 90 RPM")
        rmcs1.speed(slave_id1, speed1)
        rmcs2.speed(slave_id2, speed2)

        current_speed1 = rmcs1.speed_feedback(slave_id1) 
        current_speed2 = rmcs2.speed_feedback(slave_id2) 
        print(f"Current Speed feedback: 1:{current_speed1} 2:{current_speed2}")
        time.sleep(2)

    print("Disable Motor")
    rmcs1.disable_digital_mode(slave_id1, 1)
    rmcs2.disable_digital_mode(slave_id2, 1)




