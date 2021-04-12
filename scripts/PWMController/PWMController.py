import argparse
import Adafruit_PCA9685


# Initialize the PCA9685 using the default address (0x40).

class PWMController:
    PULSE_OPEN = [1.0, 1.0, 1.0, 1.0]
    PULSE_CLOSE = [1.7, 1.7, 1.7, 1.7]

    def __init__(self, pwm_freq: int = 50, reset: bool = False):
        self.pwm_freq = pwm_freq
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(self.pwm_freq)
        if reset:
            self.reset_channels()

    def get_pulse_value(self, time):
        pulse_length = 1000000  # 1,000,000 us per second
        pulse_length //= self.pwm_freq  # 60 Hz
        # print('{0}us per period'.format(pulse_length))
        pulse_length //= 4096  # 12 bits of resolution
        # print('{0}us per bit'.format(pulse_length))
        time *= 1000
        pulse = int(time / pulse_length)
        # print('pulse val {0}'.format(pulse))
        return pulse

    def set_channel_pulse(self, channel, time):
        pulse = self.get_pulse_value(time)
        self.pwm.set_pwm(channel, 0, pulse)

    def reset_channels(self):
        for i in range(4):
            pulse = self.get_pulse_value(self.PULSE_CLOSE[i])
            self.pwm.set_pwm(i, 0, pulse)

    def set_channel_state(self, channel, is_open):
        print(f'Servo {channel} - Opened ? {is_open}')
        self.set_channel_pulse(channel, self.PULSE_OPEN[channel] if is_open else self.PULSE_CLOSE[channel])
        if channel >= 20:
            print('Resetting channels')
            self.reset_channels()


if __name__ == '__main__':
    def str2bool(v):
        if v.lower() in ('yes', 'true', 't', 'y', '1'):
            return True
        elif v.lower() in ('no', 'false', 'f', 'n', '0'):
            return False
        else:
            raise argparse.ArgumentTypeError('Unsupported value encountered.')


    def print_args(args):
        print(f'-c {args.channel} '
              f'-o {args.is_open}')


    parser = argparse.ArgumentParser(usage="-c channel -o is_open", description="help info.")
    parser.add_argument("-c", type=int, required=True,
                        help="The channel number, specify a number greater than 20 to reset.", dest="channel")
    parser.add_argument("-o", type=str2bool, help="Flag to specify whether the channel is open.", dest="is_open")
    args = parser.parse_args()
    print_args(args)
    controller = PWMController()
    controller.set_channel_pulse(args.channel, args.is_open)
