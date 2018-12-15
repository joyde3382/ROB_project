#!/usr/bin/python
import cPickle
import os
import re
import json

#communicate with another process through named pipe
#one for receive command, the other for send command
rfPath = "./Computer_vision/p1"
#wfPath = ".//p2"

rp = open(rfPath, 'r')
response = rp.read()
#print "P2 hear %s" % response
rp.close()


#y = json.dumps(response)
y = json.loads(response)

# the result is a Python dictionary:
print(y)
print(y["Yellow"])

yellow = y["Yellow"]
Blue = y["Blue"]

print(Blue["center"])

center = Blue["center"]

print(len(center))
 
