# -*- coding: utf-8 -*-
"""
Created on Tue Jul  3 16:49:05 2018

@author: twiefel
"""



from flask import Flask, request, jsonify
class Server:
    def __init__(self, handler_function,port):
        app = Flask(__name__)
        def internal_handler_function():
            args = request.form.getlist('args')
            #return_values = "hallo"
            return_values = handler_function(args)
            res = {}
            res['status'] = "OK"
            res['return_values'] = return_values
            return jsonify(res)
        app.add_url_rule('/server',"server", internal_handler_function,methods=['POST',])
        app.run(host='0.0.0.0',port=port, debug=False, use_reloader=False,)

class ServerWrapper:
    def __init__(self,wrapped_class, port):
        self.wrapped_class = wrapped_class
        #attrs = vars(self.wrapped_class)
        attrs = dir(self.wrapped_class)
        #print attrs
        app = Flask(__name__)
        def internal_handler_function():

            #print "args",args
            attr = str(request.url_rule)[1:]
            #print "getting attr",[attr]

            #orig_attr = self.wrapped_class.__getattribute__(attr)
            orig_attr = getattr(self.wrapped_class,attr)
            #print "orig_attr",orig_attr
            if callable(orig_attr):
                #print "request.json",request.json
                #print "request.json type",type(request.json)
                import json
                new_data = json.loads(request.json)
                #print "new_data request.json",new_data
                #print "new_data request.json type",type(new_data)
                def hooked(*args, **kwargs):
                    #print "processing hooked"
                    #print args
                    #print kwargs
                    result = orig_attr(*args, **kwargs)
                    #print "result",result
                    # prevent wrapped_class from becoming unwrapped
                    if result == self.wrapped_class:
                        return self
                    return result
                return_values = hooked(*new_data["args"],**new_data["kwargs"])
            else:
                return_values = orig_attr
            #return_values = "hallo"
            #print "return_values",return_values
            res = {}
            res['status'] = "OK"
            res['return_values'] = return_values
            return jsonify(res)
        for attr in attrs:
            #setattr(self,attr)
            #print "adding",attr
            app.add_url_rule('/'+attr,"server", internal_handler_function,methods=['POST',])
        app.run(host='0.0.0.0',port=port, debug=False, use_reloader=False,)

    def __getattr__(self,attr):
        orig_attr = self.wrapped_class.__getattribute__(attr)
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

    def pre(self):
        print ">> pre"

    def post(self):
        print "<< post"
