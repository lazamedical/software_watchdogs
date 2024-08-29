#
# Copyright (c) 2024 by Laza Medical.
# All Rights Reserved.
#
# Status message for publishing lease expiry information of the watched entity.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# The unique identifier of the active sender.
string sender

# The number of missed heartbeats.
uint16 missed_number 0
