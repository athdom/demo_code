#!/usr/bin/env python

from math import pi

#AX-12
AX_MAX_TORQUE = 1.5
AX_POS_STEPS = 1024
AX_POS_RANGE = (300*pi)/180
AX_POS_UNIT = AX_POS_RANGE/(AX_POS_STEPS + 1)
AX_POS_CENTER = 512
AX_VEL_CENTER = 1024
AX_VEL_UNIT = 0.111
AX_TORQUE_STEPS = 1023.0
AX_TORQUE_UNIT = AX_TORQUE_STEPS/AX_MAX_TORQUE
AX_MAX_TORQUE_LIMIT = int(AX_TORQUE_STEPS/5)
AX_MAX_CURRENT = 1.5
AX_CURRENT_UNIT = AX_TORQUE_STEPS/AX_MAX_CURRENT

#MX-* steps
MX_POS_RANGE = 2*pi
MX_POS_STEPS = 4095
MX_POS_CENTER = 2048
MX_POS_UNIT = MX_POS_RANGE/(MX_POS_STEPS + 1)
MX_VEL_CENTER = 1024
MX_VEL_UNIT = 0.114433
MX_TORQUE_STEPS = 1023.0

#MX-28 max torque
MX28_MAX_CURRENT = 1.4
MX28_CURRENT_UNIT = MX_TORQUE_STEPS/MX28_MAX_CURRENT
MX28_MAX_TORQUE_LIMIT = int(MX_TORQUE_STEPS/5)
#MX-64 max torque
MX64_MAX_CURRENT = 4.1
MX64_CURRENT_UNIT = MX_TORQUE_STEPS/MX64_MAX_CURRENT
MX64_MAX_TORQUE_LIMIT = int(MX_TORQUE_STEPS/5)
MX64_K = 0.67
MX64_P = 0.2191

#MX-106 max torque
MX106_MAX_CURRENT = 5.2
MX106_CURRENT_UNIT = MX_TORQUE_STEPS/MX106_MAX_CURRENT
MX106_MAX_TORQUE_LIMIT = int(MX_TORQUE_STEPS/5)
MX106_K = 0.67
MX106_P = 0.2191

