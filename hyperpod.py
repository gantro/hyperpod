#import Adafruit_BBIO.GPIO as GPIO
#import Adafruit_BBIO.ADC as ADC #make sure to setup adc!!!
import can
import random
import time
import socket


class TextColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def log(text):
    print(TextColors.OKGREEN + "[INFO]: " + TextColors.ENDC + text)


def log_warning(text):
    print(TextColors.WARNING + "[WARNING]: " + TextColors.ENDC + text)


def log_error(text):
    print(TextColors.FAIL + "[ERROR]: " + TextColors.ENDC + text)


def log_state(text):
    print(TextColors.OKBLUE + "[STATE]: " + TextColors.ENDC + text)


class ActiveDrain:

    pin = ""

    def __init__(self, pin):
        self.pin = pin
        # GPIO.setup(pin, GPIO.OUT)
        log("Sink port created on pin: " + pin)

    def on(self):
        # GPIO.output(self.pin, GPIO.HIGH)
        log("pin on")

    def off(self):
        # GPIO.output(self.pin, GPIO.LOW)
        log("pin off")


class Encoder:

    pin = ""
    step = 0

    def __init__(self, pin):
        self.pin = pin
        self.step = 0
        # GPIO.setup(pin, GPIO.IN)
        log("Encoder created on pin: " + pin)

    def start(self):
        # GPIO.add_event_detect(self.pin, GPIO.RISING, callback=self.inc())
        log("Encoder " + self.pin + " started")

    def stop(self):
        # GPIO.remove_event_detect(self.pin)
        log("Encoder " + self.pin + " stopped")

    def inc(self):  # Call with interrupt
        self.step = self.step + 1

    def reset(self):
        self.step = 0
        log("Encoder " + self.pin + " reset")


class EncoderSim:

    pin = ""
    step = 0

    def __init__(self, pin):
        self.pin = pin
        self.step = 0
        log("Encoder created on pin: " + pin)

    def start(self):
        log("Encoder " + self.pin + " started")

    def stop(self):
        log("Encoder " + self.pin + " stopped")

    def inc(self):  # Call with interrupt
        self.step = self.step + 1

    def reset(self):
        self.step = 0
        log("Encoder " + self.pin + " reset")


class Accelerometer:

    pin_x = ""
    pin_y = ""
    pin_z = ""

    x_raw = 0.0
    y_raw = 0.0
    z_raw = 0.0

    x = 0.0
    y = 0.0
    z = 0.0

    cal_x = 1.0
    cal_y = 1.0
    cal_z = 1.0

    def __init__(self, pin_x, pin_y, pin_z):
        self.pin_x = pin_x
        self.pin_y = pin_y
        self.pin_z = pin_z
        log("Accelerometer created on pins: (x = " + pin_x + ", y = " + pin_y + ", z = " + pin_z + ")")

    def calibrate(self, cal_x, cal_y, cal_z):
        self.cal_x = cal_x
        self.cal_y = cal_y
        self.cal_z = cal_z
        log("Calibration values set for Accelerometer " + self.pin_x)

    def read(self):
        # self.xRaw = ADC.read(self.pinX)
        # self.yRaw = ADC.read(self.pinY)
        # self.yRaw = ADC.read(self.pinZ)
        # self.xRaw = ADC.read(self.pinX)
        # self.yRaw = ADC.read(self.pinY)
        # self.yRaw = ADC.read(self.pinZ)

        self.x = self.cal_x * self.x_raw
        self.y = self.cal_y * self.y_raw
        self.z = self.cal_z * self.z_raw


class AccelerometerSim:

    pin_x = ""
    pin_y = ""
    pin_z = ""

    x_raw = 0.0
    y_raw = 0.0
    z_raw = 0.0

    x = 0.0
    y = 0.0
    z = 0.0

    cal_x = 1.0
    cal_y = 1.0
    cal_z = 1.0

    def __init__(self, pin_x, pin_y, pin_z):
        self.pin_x = pin_x
        self.pin_y = pin_y
        self.pin_z = pin_z
        log("Accelerometer created on pins: (x = " + pin_x + ", y = " + pin_y + ", z = " + pin_z + ")")

    def calibrate(self, cal_x, cal_y, cal_z):
        self.cal_x = cal_x
        self.cal_y = cal_y
        self.cal_z = cal_z
        log("Calibration values set for Accelerometer " + self.pin_x)

    def read(self):
        self.x_raw = random.uniform(0, 1.8)
        self.y_raw = random.uniform(0, 1.8)
        self.y_raw = random.uniform(0, 1.8)

        self.x = self.cal_x * self.x_raw
        self.y = self.cal_y * self.y_raw
        self.z = self.cal_z * self.z_raw


class I2CDevices:

    pressure_high = 0.0
    pressure_low = 0.0
    pressure_brakes = 0.0
    pressure_tank = 0.0
    therm_a = 0.0
    therm_b = 0.0
    therm_c = 0.0
    therm_d = 0.0
    therm_e = 0.0

    adc_a = None
    adc_b = None

    def __init__(self):
        print("todo")


class Motor:

    status_word = 0x00
    position_actual = 0x0000
    torque_actual = 0x00
    temp_controller = 0x0
    temp_motor = 0x0
    voltage_dc = 0x00
    voltage_logic = 0x00
    current_demand = 0x00
    current_actual = 0x00
    electrical_angle = 0x00
    current_phase_a = 0x00
    current_phase_b = 0x00

    torque_target = 0x00
    velocity_target = 0x0000
    position_target = 0x0000

    bus = None

    def __init__(self, bus):
        self.bus = bus

        log("Waiting for motor controller...")
        message = bus.recv(10.0)
        if message:
            log("Motor Controller Connected!")

            try:
                bus.send(can.Message(arbitration_id=0x60a, is_extended_id=False,
                                     data=[0x40, 0x18, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00]))
                bus.recv()
                log("Motor controller initialized")

            except can.CanError:
                log_error("Could not send CAN to motor controller")

        else:
            log_error("Motor Controller Init Timeout")

    def sync(self):
        try:
            self.bus.send(can.Message(arbitration_id=0x080, is_extended_id=False,
                          data=[0x00]))
            can.Message = self.bus.recv()
            if can.Message:
                self.status_word = int.from_bytes(can.Message.data[0:2], byteorder='big')
                self.position_actual = int.from_bytes(can.Message.data[2:6], byteorder='big')
                self.torque_actual = int.from_bytes(can.Message.data[6:8], byteorder='big')

            can.Message = self.bus.recv()
            if can.Message:
                self.temp_controller = int.from_bytes(can.Message.data[0], byteorder='big')
                self.temp_motor = int.from_bytes(can.Message.data[1], byteorder='big')
                self.voltage_dc = int.from_bytes(can.Message.data[2:4], byteorder='big')
                self.voltage_logic = int.from_bytes(can.Message.data[4:6], byteorder='big')
                self.current_demand = int.from_bytes(can.Message.data[6:8], byteorder='big')

            can.Message = self.bus.recv()
            if can.Message:
                self.current_actual = int.from_bytes(can.Message.data[0:2], byteorder='big')
                self.electrical_angle = int.from_bytes(can.Message.data[2:4], byteorder='big')
                self.current_phase_a = int.from_bytes(can.Message.data[4:6], byteorder='big')
                self.current_phase_b = int.from_bytes(can.Message.data[6:8], byteorder='big')

        except can.CanError:
            log_error("Could not sync to motor contoller")

    def set_speed(self, torque):
        log("Motor speed set to " + str(torque))
        self.torque_target = torque
        torque_byte = bytearray(torque)
        self.bus.send(can.Message(arbitration_id=0x20a, is_extended_id=False,
                                  data=[0x06, 0x00, 0x00, 0x00, 0x00, 0x00]
                                  .append(int(torque_byte[0])).append(int(torque_byte[1]))))

        self.bus.send(can.Message(arbitration_id=0x20a, is_extended_id=False,
                                  data=[0x07, 0x00, 0x00, 0x00, 0x00, 0x00]
                                  .append(int(torque_byte[0])).append(int(torque_byte[1]))))

        self.bus.send(can.Message(arbitration_id=0x20a, is_extended_id=False,
                                  data=[0x0F, 0x00, 0x00, 0x00, 0x00, 0x00]
                                  .append(int(torque_byte[0])).append(int(torque_byte[1]))))

    def start(self):
        self.bus.send(can.Message(arbitration_id=0x000, is_extended_id=False,
                                  data=[0x01, 0x00]))

    def stop(self):
        self.bus.send(can.Message(arbitration_id=0x000, is_extended_id=False,
                                  data=[0x80, 0x00]))


class Interface:

    encoder_a = None
    encoder_b = None
    accel_a = None
    accel_b = None
    i2c = None

    motor = None
    bms = None
    brakes = None
    precharge = None
    contact = None

    acceleration = 0.0
    velocity = 0.0
    distance = 0.0

    time = 0.0
    time_old = 0.0

    def __init__(self, sim):
        if sim:
            self.encoder_a = EncoderSim("Sim_1")
            self.encoder_b = EncoderSim("Sim_2")
            self.accel_a = AccelerometerSim("A1", "A2", "A3")
            self.accel_b = AccelerometerSim("B1", "B2", "B3")

        else:
            self.encoder_a = Encoder("h8_15")
            self.encoder_b = Encoder("h8_17")
            self.accel_a = Accelerometer("h9_36", "h9_35", "h9_33")
            self.accel_b = Accelerometer("h9_40", "h9_38", "h9_37")

        self.brakes = ActiveDrain("h8_9")
        self.precharge = ActiveDrain("h8_10")
        self.contact = ActiveDrain("h8_11")

        self.accel_a.calibrate(10.0, 10.0, 10.0)
        self.accel_b.calibrate(10.0, 10.0, 10.0)

    def setup(self):
        self.time = time.time()
        self.accel_a.read()
        self.accel_b.read()
        self.encoder_a.reset()
        self.encoder_b.reset()

    def read(self):
        self.time_old = self.time
        self.accel_a.read()
        self.accel_b.read()
        self.time = time.time()

        self.acceleration = (self.accel_a.x + self.accel_b.x) / 2.0
        self.velocity = self.velocity + (self.acceleration * (self.time - self.time_old))
        self.distance = self.distance + (self.velocity * (self.time - self.time_old))

        self.distance = max(self.distance, self.encoder_a.step)


class StateMachine:

    sim = False
    interface = None
    tcp = None

    def __init__(self, initial_state, interface, tcp, sim=False):
        self.sim = sim
        self.interface = interface
        self.tcp = tcp
        self.current_state = initial_state

    def run(self, word=0x00):
        self.current_state = self.current_state.next(word, self.tcp)
        word_out = self.current_state.run(self.interface, self.tcp, self.sim)
        return word_out


class State:

    def run(self, interface, tcp, sim):
        assert 0, "run not implemented"

    def next(self, word, tcp):
        assert 0, "next not implemented"


class Boot(State):

    def run(self, interface, tcp, sim):
        if sim:
            log_state("Code started in simulation mode")

        else:
            log_state("Running...")

        return 0xff

    def next(self, word, tcp):
        if word == 0xff:
            log_state("State changed to INITIALIZE")
            return Initialize()
        else:
            return self


class Initialize(State):

    def run(self, interface, tcp, sim):
        tcp_ip = '127.0.0.1'
        tcp_port = 8001

        interface.setup()

        try:
            tcp.connect((tcp_ip, tcp_port))
        except socket.error as e:
            log_error(str(e))
            return 0xff

        return 0x00

    def next(self, word, tcp):
        if word == 0x00:
            log_state("State changed to SAFE MODE")
            tcp.send(b'\x06\x00')
            return SafeMode()
        else:
            return self


class SafeMode(State):

    def run(self, interface, tcp, sim):
        interface.read()
        word_out = tcp.recv(4)[0]
        return word_out

    def next(self, word, tcp):
        if word == 0x01:
            log_state("State changed to FUNCTIONAL TESTS")
            tcp.send(b'\x06\x01')
            return FunctionalTests()
        else:
            return self


class FunctionalTests(State):

    def run(self, interface, tcp, sim):
        interface.read()

        if sim:
            word_out = tcp.recv(4)[0]
            return word_out

    def next(self, word, tcp):
        if word == 0x02:
            return Loading()
        if word == 0x00 | 0x0d:
            return SafeMode()
        else:
            return self


class Loading(State):

    def run(self, interface, tcp, sim):
        log("Ready to load")
        word_out = tcp.recv(4)[0]
        return word_out

    def next(self, word, tcp):
        if word == 0x02:
            log_state("State changed to PREFLIGHT")
            tcp.send(b'\x06\x02')
            return PreFlightCheck()
        if word == 0x00 | word == 0x0d:
            log_state("State changed to SAFE MODE")
            tcp.send(b'\x06\x00')
            return SafeMode()
        else:
            return self


class PreFlightCheck(State):

    def run(self, interface, tcp, sim):
        if sim:
            word_out = tcp.recv(4)[0]
            return word_out

    def next(self, word, tcp):
        if word == 0x03:
            log_state("State changed to READY")
            tcp.send(b'\x06\x03')
            return ReadyLaunch()
        if word == 0x00 | word == 0x0d:
            log_state("State changed to SAFE MODE")
            tcp.send(b'\x06\x00')
            return SafeMode()
        else:
            return self


class ReadyLaunch(State):

    def run(self, interface, tcp, sim):
        interface.contact.on()
        word_out = tcp.recv(4)[0]
        return word_out

    def next(self, word, tcp):
        if word == 0x04:
            log_state("State changed to ACCELERATE")
            tcp.send(b'\x06\x04')
            return Acceleration()
        if word == 0x00 | word == 0x0d:
            log_state("State changed to SAFE MODE")
            tcp.send(b'\x06\x00')
            return SafeMode()
        else:
            return self


class Acceleration(State):

    def run(self, interface, tcp, sim):
        if sim:
            time.sleep(4)

        else:
            interface.motor.set_speed(100)

            while interface.distance < 1000:
                pass

        return 0x0b

    def next(self, word, tcp):
        if word == 0x0b:
            log_state("State changed to COAST")
            tcp.send(b'\x06\x0b')
            return Coast()
        if word == 0x0d | word == 0x05:
            log_state("State changed to ABORT")
            tcp.send(b'\x06\x0d')
            return Abort()
        else:
            return self


class Coast(State):

    def run(self, interface, tcp, sim):
        # interface.motor.set_speed(0)
        log("hit")
        return 0x0c

    def next(self, word, tcp):
        if word == 0x0c:
            log_state("State changed to BRAKE")
            tcp.send(b'\x06\x0c')
            return Brake()
        if word == 0x0d | word == 0x05:
            log_state("State changed to ABORT")
            tcp.send(b'\x06\x0d')
            return Abort()
        else:
            return self


class Brake(State):

    def run(self, interface, tcp, sim):
        if sim:
            time.sleep(2)
        else:
            interface.motor.stop()
            log("Motor off")
            interface.brakes.off()
            log("Brakes engaged")

            while interface.velocity >= 0.1:
                pass

        return 0x00

    def next(self, word, tcp):
        if word == 0x00:
            log_state("State changed to SAFE MODE")
            tcp.send(b'\x06\x00')
            return SafeMode()
        if word == 0x0d | word == 0x05:
            log_state("State changed to ABORT")
            tcp.send(b'\x06\x0d')
            return Abort()
        else:
            return self


class Abort(State):

    def run(self, interface, tcp, sim):
        log_warning("RUN ABORTED")
        interface.motor.stop()
        log_warning("Motor off")
        interface.brakes.off()
        log_warning("Brakes engaged")
        interface.contact.off()
        interface.precharge.off()
        log_warning("Battery off")
        word_out = tcp.recv(4)[0]
        return word_out

    def next(self, word, tcp):
        if word == 0x00:
            log_state("State changed to SAFE MODE")
            tcp.send(b'\x06\x00')
            return SafeMode()
        else:
            return self


def main():

    tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    can.rc['interface'] = 'socketcan_ctypes'
    control_word = 0x00
    can_interface = 'can1'
    # bus = can.interface.Bus(can_interface)

    interface = Interface(True)
    sm = StateMachine(Boot(), interface, tcp, True)

    while True:
        try:
            control_word = sm.run(control_word)
        except KeyboardInterrupt:
            sm.run(0x0d)
            tcp.close()
            log("interupt")
            assert 0, "exiting..."


if __name__ == "__main__":
    main()
