#!/usr/bin/env python

print("Starting programm")
import time

# import sys
# import random
# import datetime

# import array

import theano
import theano.tensor as T

print("Theano & Theano.tensor imported")
import lasagne

print("lasagne imported")
from subprocess import call
from FaceExpression import faceExpression

from nicomotion import Motion
from nicomotion import Mover
import numpy as np

from PIL import Image


def build_net(input_var=None):
    # As a third model, we'll create a CNN of two convolution + pooling stages
    # and a fully-connected hidden layer in front of the output layer.

    # Input layer, as usual:
    l_in = lasagne.layers.InputLayer(shape=(None, 6, 80, 60),
                                     input_var=input_var)
    # This time we do not apply input dropout, as it tends to work less well
    # for convolutional layers.


    # Convolutional layer with 32 kernels of size 5x5. Strided and padded
    # convolutions are supported as well; see the docstring.
    l_conv_1 = lasagne.layers.Conv2DLayer(
        l_in, num_filters=16, filter_size=(3, 3),
        nonlinearity=lasagne.nonlinearities.rectify,
        W=lasagne.init.GlorotUniform())
    # Expert note: Lasagne provides alternative convolutional layers that
    # override Theano's choice of which implementation to use; for details
    # please see http://lasagne.readthedocs.org/en/latest/user/tutorial.html.

    # Max-pooling layer of factor 2 in both dimensions:
    l_pool_1 = lasagne.layers.MaxPool2DLayer(l_conv_1, pool_size=(1, 1))

    # Another convolution with 32 5x5 kernels, and another 2x2 pooling:
    l_conv_2 = lasagne.layers.Conv2DLayer(
        l_pool_1, num_filters=16, filter_size=(4, 4),
        nonlinearity=lasagne.nonlinearities.rectify,
        W=lasagne.init.GlorotUniform())
    l_pool_2 = lasagne.layers.MaxPool2DLayer(l_conv_2, pool_size=(1, 1))

    # l_flatten = lasagne.layers.FlattenLayer(l_pool_2, nonlinearity=lasagne.nonlinearities.identity)
    # https://groups.google.com/forum/#!topic/lasagne-users/7UO8EQk6f6M
    # l_spatial_softmax = lasagne.

    # l_pool_2_drop = lasagne.layers.DropoutLayer(l_pool_2, p=0.001)

    l_hid1 = lasagne.layers.DenseLayer(
        l_pool_2, num_units=900,
        nonlinearity=lasagne.nonlinearities.sigmoid,
        W=lasagne.init.GlorotUniform())

    # We'll now add dropout of 50%:
    # l_hid1_drop = lasagne.layers.DropoutLayer(l_hid1, p=0.001)

    # Another 800-unit layer:
    l_hid2 = lasagne.layers.DenseLayer(
        l_hid1, num_units=900,
        nonlinearity=lasagne.nonlinearities.sigmoid,
        W=lasagne.init.GlorotUniform())

    l_out = lasagne.layers.DenseLayer(
        l_hid2, num_units=6,
        nonlinearity=lasagne.nonlinearities.sigmoid,
        W=lasagne.init.GlorotUniform())

    # l_merge = lasagne.layers.ConcatLayer([l_x_dense_2, l_y_dense_2])

    # l_shape = lasagne.layers.ReshapeLayer(l_merge, (1,2,30))

    return l_out


# START of CODE

# move the robot in start position, ask for object and take an image
picture_directory = "/tmp/"
fMS = 0.02
fMS_hand=1.0
robot = Motion.Motion("../../../../json/nico_humanoid_legged_with_hands_mod.json", vrep=False)

safety_relevant_joints=["_shoulder_z","_shoulder_y", "_arm_x", "_elbow_y"]

#Check if no joint is in strange position, like on the table, so that we can not break an arm
for joint in safety_relevant_joints:

    if abs(robot.getAngle("l"+joint))>15 or abs(robot.getAngle("r"+joint))>15:
        if joint == "_arm_x":
            if abs(robot.getAngle("l" + joint)+30) > 15 or abs(robot.getAngle("r" + joint)-30) > 15:
                print "!!!!!!!!!! Joint " + joint + " in wrong position "+ str(robot.getAngle("r" + joint)-30) +" . Please do not break my arm! !!!!!!!!!!!!"
                quit()
            else:
                print ""
        else:
            print "!!!!!!!!!! Joint " + joint + " in wrong position . Please do not break my arm! !!!!!!!!!!!!"
            quit()



# set the robot to be compliant
#robot.disableTorqueAll()

mover_path="../../../../moves_and_positions/"
mov = Mover.Mover(robot,stiff_off=False)

fe = faceExpression()
fe.sendFaceExpression("happiness")
#time.sleep(1)

# build the neural network
print("Building network")
network = build_net()

print("Loading network from files")
# And load parameters from file

# used for evaqluation : model_selflearning-CORE1-0.0701329990989.npz

with np.load('model_selflearning-MERGED-0.0455165514722.npz') as f:
    param_values = [f['arr_%d' % i] for i in range(len(f.files))]
lasagne.layers.set_all_param_values(network, param_values)
print("Network loaded, connecting to robot.")


# enable torque of left arm joints
robot.enableForceControl("head_z", 20)
robot.enableForceControl("head_y", 20)

robot.enableForceControl("r_shoulder_z", 20)
robot.enableForceControl("r_shoulder_y", 20)
robot.enableForceControl("r_arm_x", 20)
robot.enableForceControl("r_elbow_y", 20)
robot.enableForceControl("r_wrist_z", 20)
robot.enableForceControl("r_wrist_x", 20)

robot.enableForceControl("r_indexfingers_x", 20)
robot.enableForceControl("r_thumb_x", 20)

mov.move_file_position(mover_path + "pos_ready_arms_and_head.csv",
                           subsetfname=mover_path + "subset_right_arm.csv",
                           move_speed=0.05)

ttr=mov.move_file_position(mover_path + "pos_90_right_arm.csv",
                           subsetfname=mover_path + "subset_right_arm.csv",
                           move_speed=0.05)

# open hand and move to start position
robot.openHand('RHand', fractionMaxSpeed=fMS_hand)

print "Wait for " + str(ttr) + " seconds"

#!! Measured 1.8 seconds
time.sleep(1.8)


# begin dice loop
while (True):

    # open hand
    robot.openHand('RHand',
                   fractionMaxSpeed=fMS_hand)  # move to start position (example: ([-19.12,  11.65, -30.37,  79.52, -38.37,  -8.66]))


    # move head into recording position
    robot.setAngle("head_z", 0, fMS)
    robot.setAngle("head_y", 45, fMS)

    # move left and right hand to rest positions
    #mov.move_file_position(mover_path + "pos_ready_arms_and_head.csv",
    #                       subsetfname=mover_path + "subset_left_arm.csv",
    #                       move_speed=0.01)


    #raw_input()



    mov.move_file_position(mover_path + "pos_prep_right_arm.csv",
                           subsetfname=mover_path + "subset_right_arm.csv",
                           move_speed=0.05)
    start_time = time.time()
    #raw_input()


    fe = faceExpression()
    fe.sendFaceExpression("neutral")

    #time.sleep(4)
    print("Please put the Grasp-learning object onto the table. Thank you")
    #time.sleep(2)

    print("I am looking at the object")
    call(["fswebcam", "-r", "640x480", "-d", "/dev/video0", "--jpeg", "95", "-D", "1",
          picture_directory + "current0.jpg"])
    call(["fswebcam", "-r", "640x480", "-d", "/dev/video1", "--jpeg", "95", "-D", "1",
          picture_directory + "current1.jpg"])

    mov.move_file_position(mover_path + "pos_pre_grasp_right_arm.csv",
                           subsetfname=mover_path + "subset_right_arm.csv",
                           move_speed=0.04)



    #time.sleep(2)

    # Erik, move the head up
    # smile
    # and look neutral again
    robot.setAngle("head_y", 0, 0.05)
    time.sleep(0.3)
    fe.sendFaceExpression("happiness")
    time.sleep(1)
    fe.sendFaceExpression("neutral")



    # open hands AGAIN!
    robot.setAngle('r_indexfingers_x', -160, 0.05)
    robot.setAngle('r_thumb_x', -180, 0.05)

    # compute the grasping position from the network
    X_all = []

    file_v0 = picture_directory + "current0.jpg"
    file_v1 = picture_directory + "current1.jpg"

    # open file
    # im_original = cv2.imread("./cleaned/"+file, cv2.IMREAD_COLOR)
    im_original_v0 = Image.open(file_v0).convert('RGB')
    im_original_v1 = Image.open(file_v1).convert('RGB')

    # same post processing as in the learning

    # crop image
    # crop_img = im_original[200:400, 140:420]
    # x: 160 - 400
    # y: 240 - 360
    # Crop from x, y, w, h -> 100, 200, 300, 400
    # NOTE: its img[y: y + h, x: x + w] and *not* img[x: x + w, y: y + h]

    im_crop_v0 = im_original_v0.crop((140, 200, 420, 400))
    im_crop_v1 = im_original_v1.crop((190, 200, 470, 400))
    # NOTE: it box - The crop rectangle, as a (left, upper, right, lower)-tuple.

    # im_original = cv2.resize(crop_img, (80,60))
    im_resize_v0 = im_crop_v0.resize((80, 60))
    im_resize_v1 = im_crop_v1.resize((80, 60))

    im_resize_v0.save('cutout_v0.jpg')
    im_resize_v1.save('cutout_v1.jpg')

    # converting PIL to openCV for further processing
    im_cv_v0 = np.array(im_resize_v0)
    im_cv_v1 = np.array(im_resize_v1)
    # Convert RGB to BGR
    im_cv_v0 = im_cv_v0[:, :, ::-1].copy()
    im_cv_v1 = im_cv_v1[:, :, ::-1].copy()

    maximum = 255
    minimum = 0
    imR0 = []
    imG0 = []
    imB0 = []
    imR1 = []
    imG1 = []
    imB1 = []
    im = []
    for i in range(60):
        lineR0 = []
        lineG0 = []
        lineB0 = []
        lineR1 = []
        lineG1 = []
        lineB1 = []
        for j in range(80):
            # im_original[i][j] =  (im_original[i][j]-minimum) / (maximum-minimum))
            lineR0.append((float(im_cv_v0[i][j][0]) - float(minimum)) / (float(maximum) - float(minimum)))
            lineG0.append((float(im_cv_v0[i][j][1]) - float(minimum)) / (float(maximum) - float(minimum)))
            lineB0.append((float(im_cv_v0[i][j][2]) - float(minimum)) / (float(maximum) - float(minimum)))
            lineR1.append((float(im_cv_v1[i][j][0]) - float(minimum)) / (float(maximum) - float(minimum)))
            lineG1.append((float(im_cv_v1[i][j][1]) - float(minimum)) / (float(maximum) - float(minimum)))
            lineB1.append((float(im_cv_v1[i][j][2]) - float(minimum)) / (float(maximum) - float(minimum)))
        imR0.append(lineR0)
        imG0.append(lineG0)
        imB0.append(lineB0)
        imR1.append(lineR1)
        imG1.append(lineG1)
        imB1.append(lineB1)
    im.append(imR0)
    im.append(imG0)
    im.append(imB0)
    im.append(imR1)
    im.append(imG1)
    im.append(imB1)
    X_all.append(im)

    X_all = np.array(X_all)
    X_all = X_all.astype(theano.config.floatX)
    X_all = np.reshape(X_all, (len(X_all), 6, 80, 60))

    # X_input = np.array(X_input)
    # X_input = X_input.astype(theano.config.floatX)
    # added one dimension to the input... but still not helping...
    # X_input = np.reshape(X_input, (len(X_input),1,70,70))

    # get the trained networks poutput for the (whole) input image

    # !!!!!! This might not be sparta, but this is how Theano function are compiled for existing networks !!!!
    # good tutorial at: http://deeplearning.net/software/theano/tutorial/debug_faq.html
    # plus: https://media.readthedocs.org/pdf/lasagne/latest/lasagne.pdf
    x = T.tensor4('x')
    y = lasagne.layers.get_output(network, x)
    fun = theano.function([x], y)
    response = fun(X_all)

    # print("Network response for joint values")
    # print(response)

    # transform back response, knowing the min and max values of all 6 angles for training:
    # manually copied from the learning step

    #a = np.load('model_selflearning-MERGED-0.0455165514722.npz')
    a = np.load('jointvalues.npz')

    print a

    a_min = a['A'][0]
    a_max = a['A'][1]
    b_min = a['A'][2]
    b_max = a['A'][3]
    c_min = a['A'][4]
    c_max = a['A'][5]
    d_min = a['A'][6]
    d_max = a['A'][7]
    e_min = a['A'][8]
    e_max = a['A'][9]
    f_min = a['A'][10]
    f_max = a['A'][11]

    # a_min = -75.38
    # a_max = -9.63
    # b_min = -40.13
    # b_max = 20.35
    # c_min = -36.79
    # c_max = 3.91
    # d_min = 35.03
    # d_max = 102.9
    # e_min = -73.19
    # e_max = 44.62
    # f_min = -180.0
    # f_max = 116.35

    # !Check if this formula is correct...
    aNet = ((response[0][0] * (a_max - a_min)) + a_min)  # substract 5 degree to einsure better grasp
    bNet = ((response[0][1] * (b_max - b_min)) + b_min)
    cNet = ((response[0][2] * (c_max - c_min)) + c_min)
    dNet = ((response[0][3] * (d_max - d_min)) + d_min)
    eNet = 180  # ((response[0][4] * (e_max-e_min)) + e_min)
    fNet = ((response[0][5] * (f_max - f_min)) + f_min)

    print("Joint values from network")
    print(aNet)
    print(bNet)
    print(cNet)
    print(dNet)
    print(eNet)
    print(fNet)

    elapsed_time=time.time() - start_time
    wt=0.5
    if (wt-elapsed_time) > 0:
        time.sleep(wt-elapsed_time)

    # move hand into position
    robot.setAngle("r_shoulder_z", aNet, fMS)
    robot.setAngle("r_shoulder_y", bNet, fMS)
    robot.setAngle("r_arm_x", cNet, fMS)

    robot.setAngle("r_wrist_z", eNet, fMS_hand)
    robot.setAngle("r_wrist_x", fNet, fMS_hand)
    robot.setAngle("r_elbow_y", dNet-20, fMS)
    #raw_input()
    time.sleep(0.5)
    robot.setAngle("r_elbow_y", dNet, fMS*1.5)

    while (abs(robot.getAngle("r_elbow_y")-dNet)>3):
        time.sleep(0.1)

    #raw_input()

    # close hand
    robot.closeHand('RHand', fractionMaxSpeed=fMS_hand)
    time.sleep(2)

    thumbAngle = robot.getAngle("r_thumb_x")

    #robot.stop_sync()
    print("thumbAngle")
    print(thumbAngle)
    #raw_input()

    if thumbAngle > -75 or thumbAngle < -120:

        print("Error: I think I have not correctly grasped the object.")

        ems = ["sadness", "anger"]
        import random

        fe = faceExpression()
        fe.sendFaceExpression(random.choice(ems))
        time.sleep(2)
    else:
        fe = faceExpression()
        fe.sendFaceExpression("happiness")

        # move arm to ready position
        #ttr=mov.move_file_position(mover_path + "pos_ready_arms_and_head.csv",
        #                       subsetfname=mover_path + "subset_left_arm.csv",
        #                       move_speed=0.01)
        #time.sleep(ttr)

        robot.setAngle("r_shoulder_z", -10, fMS)
        robot.setAngle("r_shoulder_y", 22, fMS)
        robot.setAngle("r_arm_x", 26, fMS)
        robot.setAngle("r_elbow_y", -80, fMS)
        robot.setAngle("r_wrist_z", -180, fMS_hand)
        robot.setAngle("r_wrist_x", -85, fMS_hand)

        time.sleep(2)
        # move arm to block
        #l_block_position(robot)

        #time.sleep(2)

        robot.openHand('RHand', fractionMaxSpeed=fMS_hand)
        time.sleep(5)
        # robot.setAngle("r_shoulder_z", 13, fMS)
        # robot.setAngle("r_shoulder_y", 5, fMS)
        # robot.setAngle("r_arm_x", 26, fMS)
        # robot.setAngle("r_elbow_y", -95, fMS)
        # robot.setAngle("r_wrist_z", 3, fMS)
        # robot.setAngle("r_wrist_x", 174, fMS)
        robot.setAngle("r_shoulder_z", 25, fMS)
        robot.setAngle("r_shoulder_y", 21, fMS)
        robot.setAngle("r_arm_x", 29, fMS)
        # robot.setAngle("r_elbow_y", -85, fMS)
        robot.setAngle("r_wrist_z", 25, fMS_hand)
        robot.setAngle("r_wrist_x", -86, fMS_hand)

        #time.sleep(2)
        #l_shove_position(robot)
        #time.sleep(2)
        #l_block_position(robot)
        #time.sleep(2)
        #l_ready_position(robot)
        #time.sleep(5)
        #l_rest_position(robot)





    ##lift object to central middle
    # robot.setAngle("l_shoulder_z", -27, fMS)
    # robot.setAngle("l_shoulder_y", -90, fMS)
    # robot.setAngle("l_arm_x", -18, fMS)
    # robot.setAngle("l_elbow_y", 34, fMS)
    # robot.setAngle("l_wrist_z", -26, fMS)
    # robot.setAngle("l_wrist_x", -168, fMS)
    #
    # robot.setAngle("head_y", 0, fMS)
    # time.sleep(10)
