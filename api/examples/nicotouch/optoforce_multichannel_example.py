import argparse
import logging
import time

from nicotouch import OptoforceMultichannel

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

parser = argparse.ArgumentParser()
parser.add_argument("command",
                    help="raw(get raw sensor values) or newton(get sensor " +
                    "values in Newton)")
parser.add_argument('--serial', nargs='?', default=None,
                    help="serial number of the sensors device")
parser.add_argument('--cont', action="store_true", default=False,
                    help="do not stop after one reading")
parser.add_argument('--cache_frequency', nargs='?', type=int, default=None,
                    help="enables cached mode with given frequency if set")
args = parser.parse_args()
command = args.command

sensor = OptoforceMultichannel.OptoforceMultichannel(
    ser_number=args.serial,  # connects to any optoforce sensor on the usb bus
                             # if ommitted or None
    cache_frequency=args.cache_frequency  # new values are read from serial on
    # each call if ommitted or None
)

while True:
    if args.command == "raw":
        values = sensor.get_sensor_values_raw()
        format_string = "Sensor {:{len}} {}: x: {:8.1f} y: {:8.1f} z: {:8.1f}"
    elif args.command == "newton":
        values = sensor.get_sensor_values()
        format_string = "Sensor {:{len}} {}: x: {:16.12f} y: {:16.12f} " +\
            "z: {:16.12f}"
    else:
        logger.error(("Unknown command {} - known commands are 'raw' and " +
                      "'newton'").format(args.command))
        break

    logger.info("Metadata: Time: {} Count: {} Status: {}".format(
        *[values[k] for k in ("time", "count", "status")]))
    max_len = max(map(len, map(str, values["forces"].keys())))
    for key in values["forces"]:
        if key:
            logger.info(format_string.format(key, args.command,
                                             *values["forces"][key],
                                             len=max_len
                                             ))
    if not args.cont:
        break
    if args.cache_frequency:
        time.sleep(1.0 / args.cache_frequency)
