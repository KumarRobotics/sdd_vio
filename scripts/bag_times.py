#!/usr/bin/env python

import rosbag
import sys


# update_progress() : Displays or updates a console progress bar
## Accepts a float between 0 and 1. Any int will be converted to a float.
## A value under 0 represents a 'halt'.
## A value at 1 or bigger represents 100%
def update_progress(progress):
    barLength = 30 # Modify this to change the length of the progress bar
    block = int(round(barLength*progress))
    text = "\rPercent: [{0}] {1:.2f}%".format( "#"*block + "-"*(barLength-block), progress*100)
    sys.stdout.write(text)
    sys.stdout.flush()


with rosbag.Bag('output.bag', 'w') as outbag:
    inbag = rosbag.Bag(sys.argv[1])
    num_msgs = inbag.get_message_count()
    msg_count = 0
    for topic, msg, t in inbag.read_messages():
        if msg_count % 1000 == 0:
            update_progress(float(msg_count)/num_msgs)
        msg_count += 1

        if topic == "/rosout":
            continue
        if hasattr(msg, 'header'):
            outbag.write(topic, msg, msg.header.stamp)
        else:
            outbag.write(topic, msg, t)
update_progress(1)
sys.stdout.write('\n')
sys.stdout.flush()
