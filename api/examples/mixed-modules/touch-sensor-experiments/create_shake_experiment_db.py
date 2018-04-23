import sqlite3
connection = sqlite3.connect("./data_shake/experiment.db")

cursor = connection.cursor()

# delete
#cursor.execute("""DROP TABLE sample;""")
#cursor.execute("""DROP TABLE sub_sample;""")

sql_command = """
CREATE TABLE sample ( 
sample_number INTEGER PRIMARY KEY, 
object_name VARCHAR(20), 
object_weight VARCHAR(20), 
object_comb VARCHAR(40), 
timecode VARCHAR(30));"""

cursor.execute(sql_command)

sql_command = """
CREATE TABLE subsample ( 
subsample_number INTEGER PRIMARY KEY,
sample_number INTEGER,
subsample_iteration INTEGER,
timecode_local VARCHAR(30));"""

cursor.execute(sql_command)

sql_command = """
CREATE TABLE move ( 
move_number INTEGER PRIMARY KEY,
subsample_number INTEGER,
move_iteration INTEGER,
l_wrist_x_pos FLOAT,
l_wrist_z_pos FLOAT,
l_elbow_y_pos FLOAT,
l_arm_x_pos,
l_shoulder_z_pos,
l_shoulder_y_pos,
l_elbow_y_cur FLOAT,
timecode_local VARCHAR(30),
status INTEGER,
status_touch INTEGER,
counter_touch INTEGER,
x_touch INTEGER,
y_touch INTEGER,
z_touch INTEGER,
timecode_touch VARCHAR(30));"""

cursor.execute(sql_command)

# never forget this, if you want the changes to be saved:
connection.commit()

connection.close()

