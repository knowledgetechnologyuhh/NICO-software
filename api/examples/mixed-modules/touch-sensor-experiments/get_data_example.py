#Example for getting data from Experiment data
import sqlite3
connection = sqlite3.connect("./data/experiment.db")
cursor = connection.cursor()


# gets all the samples
cursor.execute("SELECT * FROM sample,subsample where sample.sample_number=subsample.sample_number")
print("fetchall:")
result = cursor.fetchall()
for r in result:
    print(r)

print "That have been all data"
print "please give a <RETURN> for showing only yellow banana data"
raw_input()

# gets all the samples with the "yellow_banana"
cursor.execute("SELECT * FROM sample,subsample where sample.sample_number=subsample.sample_number and sample.object_name='yellow_banana'")
print("fetchall:")
result = cursor.fetchall()
for r in result:
    print(r)