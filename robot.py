# !/usr/bin/env python3
import time
import os
import struct
import io
import stat


class Device:
    @staticmethod
    def open_rw(path):
        mode = stat.S_IMODE(os.stat(path)[stat.ST_MODE])
        can_read = bool(mode & stat.S_IRGRP)
        can_write = bool(mode & stat.S_IWGRP)

        if can_read and can_write:
            return io.FileIO(path, 'r+')
        elif can_write:
            return io.FileIO(path, 'w')
        else:
            return io.FileIO(path, 'r')

    @staticmethod
    def get_motor_by_port(port):
        for motor in os.listdir('/sys/class/tacho-motor/'):
            with open("/sys/class/tacho-motor/" + motor + "/address") as f:
                if port.encode('ASCII') == f.read()[10:14].encode('ASCII'):
                    return motor

    @staticmethod
    def get_sensor_by_port(port):
        for sensor in os.listdir('/sys/class/lego-sensor/'):
            with open("/sys/class/lego-sensor/" + sensor + "/address") as f:
                if port.encode('ASCII') == f.read()[10:13].encode('ASCII'):
                    return sensor

    def __init__(self, path):
        super().__setattr__('path', path)
        super().__setattr__('file_names', {})

    def __getattribute__(self, name):
        if super().__getattribute__('file_names').get(name) is not None:
            return super().__getattribute__(name)(super().__getattribute__('file_names').get(name))
        return super().__getattribute__(name)

    def __getattr__(self, name):
        super().__setattr__(name, Device.funktion(self, name))
        return super().__getattribute__(name)(super().__getattribute__('file_names').get(name))

    def __setattr__(self, name, value):
        if super().__getattribute__('file_names').get(name) is None:
            super().__setattr__(name, Device.funktion(self, name))
        super().__getattribute__('file_names').get(name).write(str(value).encode('ASCII'))
        super().__getattribute__('file_names').get(name).flush()

    def __del__(self):
        for name in super().__getattribute__('file_names'):
            super().__getattribute__('file_names')[name].close()

    @staticmethod
    def funktion(self, name):
        # open file
        self.file_names[name] = Device.open_rw(self.path + name)

        def file_reader(file):
            # read from file
            file.seek(0)
            return file.readline()

        return file_reader


class Motor(Device):
    def __init__(self, port):
        super().__init__('/sys/class/tacho-motor/' + Device.get_motor_by_port(port) + "/")

    def running(self):
        return int(self.speed) == 0

    # doesn't be used often, so will be open and close at ones, delete it if you use it really oft
    def stop_action(self, stop_action):
        stop_action_file = Device.open_rw(self.path + 'stop_action')
        stop_action_file.write(stop_action.encode('ASCII'))
        stop_action_file.close()


class DistanceSensor(Device):
    def __init__(self, port):
        super().__init__("/sys/class/lego-sensor/" + Device.get_sensor_by_port(port) + "/")

    # doesn't be used often, so will be open and close at ones, delete it if you use it really oft
    def mode(self, mode):
        mode_file = Device.open_rw(self.path + 'mode')
        mode_file.write(mode.encode('ASCII'))
        mode_file.close()


# example
class Robot:
    def __init__(self, output_left=None, output_right=None, input_color=None, input_distance=None):
        self.left_motor = Motor(output_left)
        self.right_motor = Motor(output_right)

        self.left_motor.stop_action('brake')
        self.right_motor.stop_action('brake')

        self.distance_sensor = DistanceSensor(input_distance)

        self.distance_sensor.mode('IR-PROX')

    def stop(self):
        self.left_motor.command = 'stop'
        self.right_motor.command = 'stop'

    def run_left(self, speed=200):  # speed in grad per second
        self.left_motor.speed_sp = speed
        self.left_motor.command = "run-forever"

    def run_right(self, speed=200):  # speed in grad per second
        self.right_motor.speed_sp = speed
        self.right_motor.command = "run-forever"

    def run(self, speed_left=200, speed_right=200):
        self.run_left(speed_left)
        self.run_right(speed_right)

    def drive(self, degree_left, degree_right, speed=200, waiting=True):
        self.left_motor.position_sp = degree_left
        self.right_motor.position_sp = degree_right
        self.left_motor.speed_sp = speed
        self.right_motor.speed_sp = speed
        self.left_motor.command = "run-to-rel-pos"
        self.right_motor.command = "run-to-rel-pos"
        if waiting:
            while self.left_motor.running() or self.right_motor.running():
                pass

    def stay(self):
        offset = 42
        alfa = 0.001
        one_minus_alfa = 1 - alfa

        Kc = 10
        Pc = 0.50
        dT = 1 / 80

        Kp = Kc * 0.6
        Ki = 2 * Kp / Pc * dT
        Kd = Kp * Pc / (8 * dT)

        Kp = Kc
        Ki = 0
        Kd = 0

        integral = 0
        derivative = 0
        last_error = 0

        time_target = time.perf_counter() + dT
        while True:
            value = int(self.distance_sensor.value0)
            print(value, offset)
            error = (value - offset)
            integral += error
            derivative = error - last_error

            speed = int(Kp * error + Ki * integral + Kd * derivative)
            if speed > 1050:
                speed = 1050
            elif speed < -1050:
                speed = -1050

            print(speed)
            self.run(-speed, -speed)

            last_error = error

            offset = alfa * value + one_minus_alfa * offset

            while time.perf_counter() < time_target:
                pass
            time_target += dT

    def main1(self):
        while True:
            try:
                print('value0:', self.distance_sensor.value0)
            except:
                pass
            try:
                print('value1:', self.distance_sensor.value1)
            except:
                pass
            try:
                print('value2:', self.distance_sensor.value2)
            except:
                pass
            try:
                print('value3:', self.distance_sensor.value3)
            except:
                pass
            try:
                print('value4:', self.distance_sensor.value4)
            except:
                pass
            try:
                print('value5:', self.distance_sensor.value5)
            except:
                pass
            try:
                print('value6:', self.distance_sensor.value6)
            except:
                pass
            try:
                print('value7:', self.distance_sensor.value7)
            except:
                pass
            try:
                print('value8:', self.distance_sensor.value8)
            except:
                pass
            time.sleep(0.5)

    def main(self):
        self.stay()


if __name__ == "__main__":
    robot = Robot(output_left='outA', output_right='outB', input_distance='in1')
    robot.main()
