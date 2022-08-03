#!/usr/bin/env python

MX64_PRESENT_SPEED_H = 38
MX64_PRESENT_SPEED_L = 39
MX64_PRESENT_VOLTAGE = 42
MX64_PRESENT_CURRENT_H = 69
MX64_PRESENT_CURRENT_L = 68
MX64_TORQUE_CONTROL_EN = 70 # (0X46)	Torque Control Mode Enable	Torque control mode on/off	RW	0 (0X00)
MX64_TORQUE_CONTROL_L = 71 #(0X47)	Goal Torque(L)	Lowest byte of goal torque value	RW	0 (0X00)
MX64_TORQUE_CONTROL_H = 72 #(0X48)	Goal Torque(H)	Highest byte of goal torque value	RW	0 (0X00)
MX_GOAL_ACCELARATION = 73
MX_P = 28
MX_I = 27
MX_D = 26
