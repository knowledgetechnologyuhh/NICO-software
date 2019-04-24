class db_definitions:

    #def __init__(self):
        format_str_subsample = """
            INSERT INTO subsample (subsample_number,
            sample_number , subsample_iteration, thumb_position, finger_position,
            thumb_current,finger_current, timecode_local, status, status_touch,
            counter_touch, thumb_touch_x, thumb_touch_y, thumb_touch_z, index_touch_x,
            index_touch_y, index_touch_z, ring_touch_x, ring_touch_y, ring_touch_z,
            timecode_touch, status_touch_newton, counter_touch_newton,
            thumb_touch_x_newton, thumb_touch_y_newton, thumb_touch_z_newton,
            index_touch_x_newton, index_touch_y_newton, index_touch_z_newton,
            ring_touch_x_newton, ring_touch_y_newton, ring_touch_z_newton,
            timecode_touch_newton )
            VALUES (NULL, "{sample_number}", "{subsample_iteration}", "{thumb_position}",
                "{finger_position}",
                "{thumb_current}", "{finger_current}", "{timecode_local}", "{status}",
                "{status_touch}", "{counter_touch}", "{thumb_touch_x}", "{thumb_touch_y}",
                "{thumb_touch_z}", "{index_touch_x}", "{index_touch_y}", "{index_touch_z}",
                "{ring_touch_x}", "{ring_touch_y}", "{ring_touch_z}", "{timecode_touch}",
                "{status_touch_newton}", "{counter_touch_newton}",
                "{thumb_touch_x_newton}", "{thumb_touch_y_newton}",
                "{thumb_touch_z_newton}", "{index_touch_x_newton}",
                "{index_touch_y_newton}", "{index_touch_z_newton}",
                "{ring_touch_x_newton}", "{ring_touch_y_newton}", "{ring_touch_z_newton}",
                "{timecode_touch_newton}");
            """

        format_str_subsample_arm = """
            INSERT INTO subsample_arm (subsample_number,
            sample_number , subsample_iteration, l_elbow_y_position, l_arm_x_position,
            l_shoulder_z_position, l_shoulder_y_position, l_elbow_y_current,
            l_arm_x_current, l_shoulder_z_current, l_shoulder_y_current, timecode_local,
            status_touch, counter_touch, thumb_touch_x, thumb_touch_y, thumb_touch_z,
            index_touch_x, index_touch_y, index_touch_z, ring_touch_x, ring_touch_y,
            ring_touch_z, timecode_touch, status_touch_newton, counter_touch_newton,
            thumb_touch_x_newton, thumb_touch_y_newton, thumb_touch_z_newton,
            index_touch_x_newton, index_touch_y_newton, index_touch_z_newton,
            ring_touch_x_newton, ring_touch_y_newton, ring_touch_z_newton,
            timecode_touch_newton )
            VALUES (NULL, "{sample_number}",  "{subsample_iteration}",
            "{l_elbow_y_position}", "{l_arm_x_position}", "{l_shoulder_z_position}",
            "{l_shoulder_y_position}", "{l_elbow_y_current}", "{l_arm_x_current}",
            "{l_shoulder_z_current}","{l_shoulder_y_current}", "{timecode_local}",
            "{status_touch}", "{counter_touch}", "{thumb_touch_x}",
            "{thumb_touch_y}", "{thumb_touch_z}", "{index_touch_x}", "{index_touch_y}",
            "{index_touch_z}", "{ring_touch_x}", "{ring_touch_y}", "{ring_touch_z}",
            "{timecode_touch}",
            "{status_touch_newton}", "{counter_touch_newton}",
            "{thumb_touch_x_newton}", "{thumb_touch_y_newton}",
            "{thumb_touch_z_newton}", "{index_touch_x_newton}",
            "{index_touch_y_newton}", "{index_touch_z_newton}",
            "{ring_touch_x_newton}", "{ring_touch_y_newton}", "{ring_touch_z_newton}",
            "{timecode_touch_newton}");"""

        format_str_sample = """
            INSERT INTO sample (sample_number,object_name , timecode)
            VALUES (NULL, "{object_name}", "{timecode}");"""

        create_table_sample = """
            CREATE TABLE sample (
            sample_number INTEGER PRIMARY KEY,
            object_name VARCHAR(20),
            timecode VARCHAR(30));"""

        create_table_subsample = """
            CREATE TABLE subsample (
            subsample_number INTEGER PRIMARY KEY,
            sample_number INTEGER,
            subsample_iteration INTEGER,
            thumb_position FLOAT,
            finger_position FLOAT,
            thumb_current FLOAT,
            finger_current FLOAT,
            timecode_local VARCHAR(30),
            status INTEGER,
            status_touch INTEGER,
            counter_touch INTEGER,
            thumb_touch_x INTEGER,
            thumb_touch_y INTEGER,
            thumb_touch_z INTEGER,
            index_touch_x INTEGER,
            index_touch_y INTEGER,
            index_touch_z INTEGER,
            ring_touch_x INTEGER,
            ring_touch_y INTEGER,
            ring_touch_z INTEGER,
            timecode_touch VARCHAR(30),
            status_touch_newton INTEGER,
            counter_touch_newton INTEGER,
            thumb_touch_x_newton FLOAT,
            thumb_touch_y_newton FLOAT,
            thumb_touch_z_newton FLOAT,
            index_touch_x_newton FLOAT,
            index_touch_y_newton FLOAT,
            index_touch_z_newton FLOAT,
            ring_touch_x_newton FLOAT,
            ring_touch_y_newton FLOAT,
            ring_touch_z_newton FLOAT,
            timecode_touch_newton VARCHAR(30));"""

        create_table_subsample_arm = """
            CREATE TABLE subsample_arm (
            subsample_number INTEGER PRIMARY KEY,
            sample_number INTEGER,
            subsample_iteration INTEGER,
            l_elbow_y_position FLOAT,
            l_arm_x_position FLOAT,
            l_shoulder_z_position FLOAT,
            l_shoulder_y_position FLOAT,
            l_elbow_y_current FLOAT,
            l_arm_x_current FLOAT,
            l_shoulder_z_current FLOAT,
            l_shoulder_y_current FLOAT,
            timecode_local VARCHAR(30),
            status_touch INTEGER,
            counter_touch INTEGER,
            thumb_touch_x INTEGER,
            thumb_touch_y INTEGER,
            thumb_touch_z INTEGER,
            index_touch_x INTEGER,
            index_touch_y INTEGER,
            index_touch_z INTEGER,
            ring_touch_x INTEGER,
            ring_touch_y INTEGER,
            ring_touch_z INTEGER,
            timecode_touch VARCHAR(30),
            status_touch_newton INTEGER,
            counter_touch_newton INTEGER,
            thumb_touch_x_newton FLOAT,
            thumb_touch_y_newton FLOAT,
            thumb_touch_z_newton FLOAT,
            index_touch_x_newton FLOAT,
            index_touch_y_newton FLOAT,
            index_touch_z_newton FLOAT,
            ring_touch_x_newton FLOAT,
            ring_touch_y_newton FLOAT,
            ring_touch_z_newton FLOAT,
            timecode_touch_newton VARCHAR(30));"""

