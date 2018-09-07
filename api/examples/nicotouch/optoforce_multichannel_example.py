import logging
import time

from nicotouch import OptoforceMultichannel

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

sensor = OptoforceMultichannel.OptoforceMultichannel(
    ser_number="ONR0A003",  # ser_number can be ommited for autodetection
    # cache_frequency=10  # set cache frequency to activate cashed mode
)

while True:
    raw = sensor.get_sensor_values_raw()
    newton = sensor.get_sensor_values()
    for key in raw:
        if key:
            logging.info(
                "Sensor {} raw: x: {} y: {} z: {}".format(key,
                                                          *raw[key]))
            logging.info(
                "Sensor {} newton: x: {} y: {} z: {}".format(key,
                                                             *newton[key]))
    time.sleep(.1)
