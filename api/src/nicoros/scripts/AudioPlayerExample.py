#!/usr/bin/env python

import time

import nicomsg.msg as msg
import nicomsg.srv as srv
import rospy


def talker(file):
    rospy.init_node("audio_player_example", anonymous=True)
    # load audiofile
    load = rospy.ServiceProxy("nico/audioplayer/load_file", srv.LoadAudio)
    response = load(
        file,  # filename
        0.0,  # startposition from which to load file
        0.0,  # duration of the file to load (full length if 0)
    )
    id = response.id  # id assigned by the audioplayer to the loaded segment
    # get position and duration of the file
    get_duration = rospy.ServiceProxy("nico/audioplayer/get_duration", srv.GetValue)
    get_position = rospy.ServiceProxy("nico/audioplayer/get_position", srv.GetValue)
    print("Duration of the loaded file: %f", get_duration(id))
    print("Current playback position: %f", get_position(id))
    # play audio for 10 seconds
    print("Play file for 10 seconds")
    play_publisher = rospy.Publisher(
        "nico/audioplayer/play", msg.sf, queue_size=1, latch=True
    )
    play_publisher.publish(id, 1.0)  # id returned from load, volume percentage
    time.sleep(10)
    # pause
    print("pausing")
    pause_publisher = rospy.Publisher(
        "nico/audioplayer/pause", msg.s, queue_size=1, latch=True
    )
    pause_publisher.publish(id)
    time.sleep(1)
    print("Current playback position: %f", get_position(id))
    # resume (to restart from the beginning call start instead)
    print("Resume playing the file")
    resume_publisher = rospy.Publisher(
        "nico/audioplayer/resume", msg.s, queue_size=1, latch=True
    )
    resume_publisher.publish(id)
    time.sleep(1)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--file", help=("Path to an audio file to play"),
    )
    args = parser.parse_args()

    try:
        talker(args.file)
    except rospy.ROSInterruptException:
        pass
