import logging

import pypot.dynamixel.error

logger = logging.getLogger(__name__)


class OverloadError(BaseException):
    pass


class MotionErrorHandler(pypot.dynamixel.error.BaseErrorHandler):
    """
    This class is a basic handler that just skip the communication errors.
    """

    def __init__(self):
        super(MotionErrorHandler, self).__init__()
        self.timeout_counter = {'total': 0}

    def handle_timeout(self, timeout_error):
        super(MotionErrorHandler, self).handle_timeout(timeout_error)
        if type(timeout_error.ids) == int:
            motor = timeout_error.ids
            if motor not in self.timeout_counter:
                self.timeout_counter[motor] = 0
            self.timeout_counter[motor] += 1
        else:
            for motor in timeout_error.ids:
                if motor not in self.timeout_counter:
                    self.timeout_counter[motor] = 0
                self.timeout_counter[motor] += 1
        self.timeout_counter['total'] += 1
        logger.debug("Number of timeouts: {}".format(
            " | ".join(["{}: {}".format(k, v) for k, v in
                        self.timeout_counter.items()])))

    def handle_checksum_error(self, instruction_packet):
        msg = 'Checksum error after sending {}'.format(
            instruction_packet)

        logger.error(msg)

    def handle_overload_error(self, instruction_packet):
        msg = ("Overload error after sending {} - please unplug NICO's " +
               "power cable and then replug it"
               ).format(
            instruction_packet)

        raise OverloadError(msg)
