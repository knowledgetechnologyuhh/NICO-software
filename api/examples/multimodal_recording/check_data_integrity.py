import sqlite3
import os
import time

# Minimal of exspected camera pictures in one directory
MIN_CAM_PICS = 100

# Maximum diffrence in numbers of pictures between the two cameras
MAX_NUM_DIF_CAMS = 3

# We need the frames per second here
FPS = 30

# Maximum difference in seconds between sound recording and camera recording (picture numbers * FPS)
MAX_DIFF_DUR_SOUND_VISION = 4

# Maximum difference in seconds between sound recording and camera recording (picture numbers * FPS)
MAX_DIFF_TIME_VISION = 2

# Maximum amount of touch sensor update errors
MAX_UPD_TOUCH = 5

# Maximum amount of touch sensor update errors  in succession
MAX_UPD_TOUCH_SUC = 2

# Minimal current value
MIN_CUR = -10.0

# Minimal current value
MAX_CUR = 1000.0

# Minimal pos value
MIN_POS = -180.0

# Minimal pos value
MAX_POS = 180.0

# Maximum Difference between two position readings
MAX_POS_DIFFERENCE = 5.0


# Wait until all pictures are written on harddisk
def get_number_of_filenames(dir_name):
    #print("dirname " + str(dir_name) + " " + str(os.listdir(dir_name)))
    # return(len([name for name in os.listdir(dir_name) if os.path.isfile(name)]))
    return(len(os.listdir(dir_name)))


def wait_for_camera_writing(dir_name):
    cur_num_cam_files = get_number_of_filenames(dir_name)
    time.sleep(1)
    while (cur_num_cam_files < get_number_of_filenames(dir_name)):
        cur_num_cam_files = get_number_of_filenames(dir_name)
        time.sleep(1)


def get_wav_file_length(sf_name):
    import wave
    import contextlib
    with contextlib.closing(wave.open(sf_name, 'r')) as f:
        frames = f.getnframes()
        rate = f.getframerate()
        length = frames / float(rate)
        return (length)


def data_check_clean(dir_name, df_l, df_r,running_time=None):

    # Open the dataframes
    #index_l, row_l in df_l.iterrows()
    #index_r, row_r in df_r.iterrows()
    errs=[]

    #get maximum difference between two position lines
    for side in [df_l, df_r]:
        filter_col = [col for col in side if col.endswith('pos')]
        #print filter_col
        #raw_input()
        filter_col=['head_y_pos', 'head_z_pos', 'r_arm_x_pos', 'r_elbow_y_pos', 'r_shoulder_y_pos', 'r_shoulder_z_pos']
        max_col=side[filter_col].diff().max(axis=1)
        #print(max_col)
        max_val=max_col.max()
        if (max_val>MAX_POS_DIFFERENCE):
            print "Max val: " + str(max_val)
            errs.append ("Pos difference")

    #return ""

    # if number of datalines of camera1 has not the same number as pictures, return false
    cam1_num = get_number_of_filenames(dir_name+"/camera1/")
    if cam1_num != df_l.shape[0]:
        #print "cam1: " + str(cam1_num) + " dfl_l_num: " + str(df_l.shape[0])
        errs.append ("cam1_num")

    # same for the second camera
    cam2_num = get_number_of_filenames(dir_name+"/camera2/")
    if cam2_num != df_r.shape[0]:
        errs.append ("cam2_num")

    # suspicious small amount of camera pictures
    if cam2_num < MIN_CAM_PICS or cam1_num < MIN_CAM_PICS:
        errs.append "low amount of pics and data entries")

    if (abs(cam2_num-cam1_num) > MAX_NUM_DIF_CAMS):
        print "cam1: " + str(cam1_num) + " cam2: " + str(cam2_num)
        errs.append ("difference of camera pictures between right and left too high")

    #Check the frames if accurate running time was given
    if (running_time!=None):
        exspected_frames=int(round(running_time*FPS))
        if (abs(exspected_frames-cam1_num)>MAX_DIFF_TIME_VISION) or \ 
            (abs(exspected_frames-cam2_num)>MAX_DIFF_TIME_VISION):
            errs.append ("difference between exspected camera picture and recorded ones too high")

    # if somethig is wrong with the soundfile, false
    import glob
    sf_name = glob.glob(dir_name+"/*.wav")[0]
    if sf_name == "" or sf_name == None:
        errs.append ("no wave file existent")

    #print "Wave-length: " + str(get_wav_file_length(sf_name))

    wav_duration = get_wav_file_length(sf_name)
    cam_duration = cam1_num*FPS/1000.0

    if wav_duration-cam_duration > MAX_DIFF_DUR_SOUND_VISION:
        #print "cam time :" + str(cam1_num*FPS/1000.0) + \
        #    "wav time :" + str(wav_duration)
        errs.append ("wave vision time difference too high")

    # As sound recording starts before vision recording, this should always be longer
    if cam_duration-wav_duration > 0:
        print "cam time :" + str(cam1_num*FPS/1000.0) + \
            "wav time :" + str(wav_duration)
        errs.append ("vision duration higher than wav duration")

    # if something is wrong with the touch data
    # on the touch data, the time stamp has to change on every datapoint
    for side in [df_l, df_r]:
        last_timecode = ""
        iso_errors = 0
        acu_iso_errors = 0
        for index, row in side.iterrows():
            #print str(row["touch_isotime"])
            if last_timecode == row["touch_isotime"]:
                #print "index: " + str(index) + "isotime: " + row["touch_isotime"] + " picture isotime " + row["isotime"]
                iso_errors += 1
                acu_iso_errors += 1
            else:
                last_timecode = row["touch_isotime"]
                acu_iso_errors = 0
        if iso_errors > MAX_UPD_TOUCH:
            errs.append ("equal iso-time in touch-sensor-data")
        if acu_iso_errors > MAX_UPD_TOUCH_SUC:
            errs.append ("successive equal iso-time in touch-sensor-data")

    # if current data are too high or too low
    for side in [df_l, df_r]:
        for index, row in side.iterrows():
            #print "index, row: " + str(index) + " " +str(row)
            #cur_values = [value for key, value in row.items() if key.endswith("_cur")]
            cur_values = [v for k,v in row.iteritems() if k.endswith("_cur")]
            cur_values_sorted = sorted(cur_values)
            #print "cur_values_sorted: " + str(cur_values_sorted) 
            if (cur_values_sorted[0] < MIN_CUR):
                # raw_input()
                errs.append ("current to low")
            cur_values_sorted_rev = sorted(cur_values, reverse=True)
            if (cur_values_sorted_rev[0] > MAX_CUR):
                #print cur_values_sorted_rev
                # raw_input()
                errs.append ("current to high: " + str(cur_values_sorted_rev[0]))

    # if position data are too high or too low
    for side in [df_l, df_r]:
        for index, row in side.iterrows():
            #pos_values = [value for key, value in row.items() if key.endswith("_pos")]
            pos_values = [v for k,v in row.iteritems() if k.endswith("_pos")]
            pos_values_sorted = sorted(pos_values)
            if (pos_values_sorted[0] < MIN_POS):
                #print str(pos_values_sorted) + " " + str(pos_values_sorted[0])
                #raw_input()
                errs.append ("pos to low")
            pos_values_sorted_rev = sorted(pos_values, reverse=True)
            if (pos_values_sorted_rev[0] > MAX_POS):
                #print pos_values_sorted_rev
                # raw_input()
                errs.append ("pos to high")
    if errs ==[]:
        return ""
    else
        return errs

