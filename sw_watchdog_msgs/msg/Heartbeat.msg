#
# Copyright (c) 2024 by Laza Medical.
# All Rights Reserved.
#
# Heartbeat message that supports additional checkpoint information for
# control flow monitoring on the receiver side.

# Two-integer timestamp in header is expressed as seconds and nanoseconds.
std_msgs/Header header

# The unique identifier of the active checkpoint.
uint16 checkpoint_id 0
