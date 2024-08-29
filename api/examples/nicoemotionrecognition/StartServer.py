#!/usr/bin/env python
import argparse
from nicoemotionrecognition.EmotionRecognitionServer import EmotionRecognitionServer


parser = argparse.ArgumentParser(
    description=("Starts backend for emotion recognition model with or without GUI")
)

parser.add_argument(
    "--disable-gui", dest="gui", action="store_false", help="Disables the GUI."
)

args = parser.parse_args()

server = EmotionRecognitionServer(args.gui, "development")
