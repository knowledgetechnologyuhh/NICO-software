import logging
import rospy


class RosLoggingHandler(logging.Handler):
    """Custom logging handler that forwards log messages to rospy.log functions."""

    def emit(self, record):
        log_entry = f"{record.msg}"
        if record.levelno == logging.DEBUG:
            rospy.logdebug(log_entry)
        elif record.levelno == logging.INFO:
            rospy.loginfo(log_entry)
        elif record.levelno == logging.WARNING:
            rospy.logwarn(log_entry)
        elif record.levelno == logging.ERROR:
            rospy.logerr(log_entry)
        elif record.levelno == logging.CRITICAL:
            rospy.logfatal(log_entry)
