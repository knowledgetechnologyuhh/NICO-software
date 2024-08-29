
# -*- coding: utf-8 -*-
"""
Rough scp copy. Will only work on linux.

@author: strahl
"""

def sc_copy(source,target):
    from subprocess import call

    #cmd = "scp user1@host1:files user2@host2:files"
    cmd ="scp " + source + " " + target
    call(cmd.split(" "))

if __name__== '__main__':
    sc_copy("wtmpc211:/tmp/test.txt","/tmp/test.txt")
