import logging

import pypot.dynamixel.error

logger = logging.getLogger(__name__)


class OverloadError(BaseException):
    pass


class MotionErrorHandler(pypot.dynamixel.error.BaseErrorHandler):
    """
    This class is a basic handler that just skip the communication errors.
    """

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
