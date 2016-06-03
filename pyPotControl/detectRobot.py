LOGGING = {
    'version': 1,
    'disable_existing_loggers': True,
    'formatters': {
        'time': {
            'format': '%(asctime)s',
        },
    },
    'handlers': {
        'file': {
            'level': 'DEBUG',
            'class': 'logging.FileHandler',
            'filename': 'pypot.log',
            'formatter': 'time',
        },
    },
    'loggers': {
        'pypot.dynamixel.io': {
            'handlers': ['file', ],
            'level': 'DEBUG',
        },
    },
}

from pypot.dynamixel import autodetect_robot
import logging
import logging.config
import json

#logging.config.dictConfig(LOGGING)
logging.basicConfig(filename='example.log',level=logging.DEBUG)

my_robot = autodetect_robot()

config = my_robot.to_config()

with open('my_smaller_robot.json', 'wb') as f:
    json.dump(config, f)

