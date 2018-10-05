experiment_objects=["red_tomato","green_sausage","red_ball","yellow_banana","red_banana", 
        "yellow_dice","green_pepper","blue_ball","red_dice","puple_grapes","red_sponge","orange_carrot",     
        "black_hat","purple_duck","orange_fish","green_figure"]


def numberToString(number):
             
    return experiment_objects[number]

def get_experiment_object_id(obj_str):
    return experiment_objects.index(obj_str)
      

#Example for getting data from Experiment data
import sqlite3
connection = sqlite3.connect("./experiment.db")
cursor = connection.cursor()

import pandas as pd

# gets all the samples
#cursor.execute("Select * from sample,subsample where sample.sample_number=subsample.sample_number")
#print("fetchall:")
#result = cursor.fetchall()
#for r in result:
#    print(r)

#print "That have been all data"
#print "please give a <RETURN> for showing only yellow banana data"
#raw_input()

# gets all the IDs with the "red_ball"
#cursor.execute("SELECT * FROM sample,subsample where \
#sample.sample_number=subsample.sample_number and sample.object_name='red_ball'")

network_values= ["thumb_position","finger_position","thumb_current","finger_current","x_touch", "y_touch", "z_touch","object_number"]

#df= pd.DataFrame(columns={"thumb_position":float(),"finger_position":float(),
#                            "thumb_current":float(),"finger_current":float(),"x_touch":float(), "y_touch":float(), "z_touch":float()
#                            ,"object_number":int()})
df= pd.DataFrame(columns=["thumb_position","finger_position",
                            "thumb_current","finger_current","x_touch", "y_touch", "z_touch"
                            ,"object_number"])



for object_name in experiment_objects:
    cursor.execute("SELECT sample_number FROM sample where sample.object_name='"+object_name+"'")

    #print("fetchall:")
    result = cursor.fetchall()
    stop_positions=[]
    for r in result:
        #print(r)
        cursor.execute("Select thumb_position,finger_position,thumb_current,finger_current,x_touch, y_touch, z_touch from subsample where sample_number="+str(r[0])+" order \
        by subsample_number")
        result2 = cursor.fetchall()
        former_position=-185
        found=False
        for q in result2:
            if former_position+3 > q[0] and not found:
                stop_positions.append(q)
                #df.append()
                to_add=list(q)
                to_add.append(get_experiment_object_id(object_name))
                print to_add
                df.loc[len(df)+1]=to_add
                found=True
            else:
                former_position=q[0]
    
    print ("stop_positions: " + str(stop_positions)) 
    print "\n\n " + object_name
    import numpy as np
    print 'variance: ', np.var(stop_positions)
    print 'mean: ', np.mean(stop_positions)
    print 'median: ', np.median(stop_positions)
    #raw_input()
print df
connection = sqlite3.connect("stop_values.db")
df.to_sql("subsample", connection, if_exists="replace",index=False)
connection.commit()
connection.close()