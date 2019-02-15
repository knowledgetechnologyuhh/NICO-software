#Example for getting data from Experiment data
import sqlite3
connection = sqlite3.connect("./data_shake/experiment.db")
cursor = connection.cursor()


# gets all the samples
cursor.execute("SELECT * FROM sample INNER JOIN subsample ON sample.sample_number=subsample.sample_number INNER JOIN move ON move.subsample_number=subsample.subsample_number")
print("fetchall:")
result = cursor.fetchall()
for r in result:
    print(r)
    

print "That have been all data"
print "please give a <RETURN> for showing only ST_1 data"
raw_input()

# gets all the samples with the "ST_1" (Stone, weight 1)
cursor.execute("SELECT * FROM sample INNER JOIN subsample ON sample.sample_number=subsample.sample_number INNER JOIN move ON move.subsample_number=subsample.subsample_number where object_comb='ST_1'")
print("fetchall:")
result = cursor.fetchall()
for r in result:
    print(r)


print "please give a <RETURN> to show all filenames with ST_1 recordings (the long versions)"
raw_input()

cursor.execute("SELECT sample.sample_number FROM sample where object_comb='ST_1'")

result = cursor.fetchall()
for r in result:
    print("./s_"+str(r[0])+".wav")
    
print "please give a <RETURN> to show all filenames with ST_1 recordings (the short versions)"
raw_input()

cursor.execute("SELECT subsample.subsample_number FROM sample INNER JOIN subsample ON sample.sample_number=subsample.sample_number where object_comb='ST_1'")

result = cursor.fetchall()
for r in result:
    print("./sub_"+str(r[0])+".wav")

