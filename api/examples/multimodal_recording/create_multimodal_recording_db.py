import sqlite3
connection = sqlite3.connect("./multimodal_experiment.db")

cursor = connection.cursor()

# delete
#cursor.execute("""DROP TABLE sample;""")
#cursor.execute("""DROP TABLE sub_sample;""")

sql_command = """
CREATE TABLE sample ( 
sample_number INTEGER PRIMARY KEY, 
object_name VARCHAR(20),
action VARCHAR(20), 
timecode VARCHAR(30));"""

cursor.execute(sql_command)

# never forget this, if you want the changes to be saved:
connection.commit()

connection.close()

