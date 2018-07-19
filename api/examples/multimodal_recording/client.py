# -*- coding: utf-8 -*-
"""
Created on Tue Jul  3 16:49:11 2018

@author: twiefel
"""
import requests
import json
class Client:
    def __init__(self, handler_function,port,server="localhost"):
        self.server = server
        self.port = port
        self.handler_function = handler_function
    def call(self,args):
        #args = ["bli","bla","blub"]
        payload = {'args':args}
        r = requests.post("http://"+self.server+":"+str(self.port)+"/server", data=payload)
        result = r.json()["return_values"]
        return result

class ClientWrapper:
    def __init__(self,wrapped_class, port, server="localhost"):
        self.wrapped_class = wrapped_class
        #attrs = vars(self.wrapped_class)
        #print dir(self.wrapped_class)
        #print attrs
        #print self.wrapped_class.__dict__.items()
        from types import FunctionType
        
        def methods(cls):
            return [x for x, y in cls.__dict__.items() if type(y) == FunctionType]
        self.methods = methods(self.wrapped_class)
        self.server = server
        self.port = port

    def __getattr__local(self,attr):
        #print "getting attr",[attr]
        #orig_attr = self.wrapped_class.__getattribute__(attr)
        orig_attr = getattr(self.wrapped_class,attr)
        
        if callable(orig_attr):
            def hooked(*args, **kwargs):
                self.pre()
                result = orig_attr(*args, **kwargs)
                # prevent wrapped_class from becoming unwrapped
                if result == self.wrapped_class:
                    return self
                self.post()
                return result
            return hooked
        else:
            return orig_attr

    def __getattr__(self,attr):
        #print "getting attr",[attr]
        #orig_attr = self.wrapped_class.__getattribute__(attr)
        if attr in self.methods:
            #print "attr",attr,"is callable"
            def hooked(*args, **kwargs):

                #print "args",args
                #print "kwargs",kwargs

                payload = {'args':args,"kwargs":kwargs}
                

                data = json.dumps(payload)
                #print data
                #new_data = json.loads(data)
                #print new_data
                
                
                r = requests.post("http://"+self.server+":"+str(self.port)+"/"+attr, json=data)
                #print r.json()
                result = r.json()["return_values"]

                # prevent wrapped_class from becoming unwrapped
                if result == self.wrapped_class:
                    return self
                    
                return result
            return hooked
        else:
            #print "attr",attr,"is not callable"
            r = requests.post("http://"+self.server+":"+str(self.port)+"/"+attr)
            result = r.json()["return_values"]
            return result
