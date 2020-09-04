#!/usr/bin/env python

import time

import nicomsg.srv as srv
import rospy


def talker():
    rospy.init_node("text_to_speech_example", anonymous=True)
    # load audiofile
    say = rospy.ServiceProxy("nico/text_to_speech/say", srv.SayText)
    # default
    say(
        text="This is a test sentence.",
        language="en-GB",
        pitch=0,
        speed=1.0,
        blocking=True,
    )

    # change pitch
    say(
        text="This is a test sentence with a slightly lower pitch.",
        language="en-GB",
        pitch=-0.1,
        speed=1.0,
        blocking=True,
    )

    # change speed without changing pitch
    say(
        text="This is a fast test sentence.",
        language="en-GB",
        pitch=0,
        speed=1.3,
        blocking=True,
    )

    # change pitch and re-adjust speed
    say(
        text="This is a test sentence with increased pitch but readjusted speed.",
        language="en-GB",
        pitch=0.2,
        speed=2 ** -0.2,
        blocking=True,
    )

    # non-blocking
    response = say(
        text="This is also a test sentence, but the call doesn't block.",
        language="en-GB",
        pitch=0.0,
        speed=1.0,
        blocking=False,
    )
    print("sleep until non-blocking call is done.")
    time.sleep(response.duration)

    # different language, e.g german
    say(
        text="Dies ist ein deutscher Beispielsatz.",
        language="de",
        pitch=0.0,
        speed=1.0,
        blocking=True,
    )


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
