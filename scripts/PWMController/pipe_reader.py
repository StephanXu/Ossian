from PWMController import PWMController
from absl import app
from absl import flags
import ctypes
import os

FLAGS = flags.FLAGS
flags.DEFINE_string('fifo-name', None, 'Path to FIFO pipe file.')
flags.DEFINE_integer('pwm-freq', 50, 'PWM frequency.')


class PWMInfo(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('channel', ctypes.c_uint8),
                ('value', ctypes.c_double)]

    def receive(self, raw_data):
        fit = min(len(raw_data), ctypes.sizeof(self))
        ctypes.memmove(ctypes.addressof(self), raw_data, fit)

    def to_bytes(self):
        return bytes(self)

    def get_value(self):
        return {'channel': self.channel, 'value': self.value}

    @staticmethod
    def size():
        return ctypes.sizeof(PWMInfo)


def main(argv):
    del argv
    pwm = PWMController()
    pwm.pwm_freq(FLAGS.pwm_freq)
    while True:
        try:
            os.mkfifo(FLAGS.pipe_filename)
        except FileExistsError:
            pass
        print("Opening FIFO...")
        with open(FLAGS.pipe_filename, 'rb') as fifo:
            print("FIFO opened")
            while True:
                data = fifo.read(data.size())
                if len(data) == 0:
                    print("Writer closed")
                    break
                pwm_info = PWMInfo()
                pwm_info.receive(data)
                print('Read: "{0}"'.format(data))
                print(f'Serialize: {pwm_info.get_value()}')
                pwm.set_channel_pulse(pwm_info.channel, pwm_info.time)


if __name__ == '__main__':
    app.run(main)
