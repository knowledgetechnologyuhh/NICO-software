class db_definitions:

    def __init__(self):
        self.format_str_subsample = """
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

        self.format_str_subsample_arm = """
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

        self.format_str_sample = """
            INSERT INTO sample (sample_number,object_name , timecode)
            VALUES (NULL, "{object_name}", "{timecode}");"""
