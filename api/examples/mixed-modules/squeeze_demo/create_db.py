import sqlite3

def create_db(filename):
    connection = sqlite3.connect(filename)

    cursor = connection.cursor()

    # delete
    #cursor.execute("""DROP TABLE sample;""")
    #cursor.execute("""DROP TABLE sub_sample;""")

    sql_command = """
    CREATE TABLE sample ( 
    sample_number INTEGER PRIMARY KEY, 
    object_name VARCHAR(20), 
    timecode VARCHAR(30));"""

    cursor.execute(sql_command)

    sql_command = """
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
    x_touch INTEGER,
    y_touch INTEGER,
    z_touch INTEGER,
    timecode_touch VARCHAR(30));"""

    cursor.execute(sql_command)

    # never forget this, if you want the changes to be saved:
    connection.commit()

    connection.close()

