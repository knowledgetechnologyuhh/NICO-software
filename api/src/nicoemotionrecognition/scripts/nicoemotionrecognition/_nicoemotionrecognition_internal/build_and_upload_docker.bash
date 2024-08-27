#!/bin/bash

# log into repository
docker login git.informatik.uni-hamburg.de:4567
# get active branch name for tag
CURRENT_BRANCH=$(git branch --show-current)
# generate tag to upload to repository
IMAGE_TAG=git.informatik.uni-hamburg.de:4567/wtm-robots-and-equipment/nico-software/emotionrecognition:$CURRENT_BRANCH
# build image
docker build -t $IMAGE_TAG .
# push to repository
docker push $IMAGE_TAG
