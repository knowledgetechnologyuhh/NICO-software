import check_data_integrity
import sqlite3
import pandas as pd

data_dir="/informatik3/wtm/datasets/KT Internal Datasets/20180615_NICO_multimodal_recording/"
database_file="/informatik3/wtm/datasets/KT Internal Datasets/20180615_NICO_multimodal_recording/multimodal_experiment.db"

#Factory for reading rows of mysql to get dict instead of a list
def dict_factory(cursor, row):
    d = {}
    for idx, col in enumerate(cursor.description):
        d[col[0]] = row[idx]
    return d

#interate through the existing directories

connection = sqlite3.connect(database_file)
connection.row_factory = dict_factory
cursor = connection.cursor()

cursor.execute(
        "SELECT * FROM sample ")
result = cursor.fetchall()
error_num=0

for r in result:
    #print r
    #raw_input()
    sample_dir=data_dir+r['action']+"/"+str(r['sample_number'])+"/"
    try:
        dfr = pd.read_csv(sample_dir+"right_cam_synced_data.csv")
        dfl = pd.read_csv(sample_dir+"left_cam_synced_data.csv")
        errors_in_data=check_data_integrity.data_check_clean(sample_dir,dfl,dfr)
        if errors_in_data!="":
            print ("\nError in directory " + sample_dir + " : " +errors_in_data)
            error_num+=1
    except:
        print "Error !!!!!!"
    #else:
    #    print ("\nNo Error in directory " + sample_dir)
print ("\nErrors:" + str(error_num))