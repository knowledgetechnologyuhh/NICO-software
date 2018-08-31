#Generating videos for all data
import sqlite3

import subprocess

connection = sqlite3.connect("./multimodal_experiment.db")
cursor = connection.cursor()



# gets all the samples
cursor.execute("SELECT action,sample_number FROM sample")
print("fetchall:")
result = cursor.fetchall()
for r in result:
    comm="convertVision.sh"
    commpar=str(r[0])+"/"+str(r[1])
    print(comm + " " + commpar)
    subprocess.call(["bash",comm,commpar])
