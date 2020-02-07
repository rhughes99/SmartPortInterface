#!/bin/bash
export PRUN=1
export TARGET=SmartPortPru
echo PRUN=$PRUN
echo TARGET=$TARGET

# Configure PRU pins
# Inputs:
# WDATA		r31.t0	P8_45
# P0/REQ	r31.t1	P8_46
# P1		r31.t2	P8_43
# P2		r31.t3	P8_44
# P3		r31.t4	P8_41
config-pin P8_45 pruin
config-pin P8_46 pruin
config-pin P8_43 pruin
config-pin P8_44 pruin
config-pin P8_41 pruin

# Outputs:
# OUTEN		r30.t5	P8_42	// 0=LS367 enabled, RDAT active
# RDAT		r30.t6	P8_39
# ACK		r30.t7	P8_40	// 1=ready to send/receive
# LED		r30.t8	P8_27
# TEST		r30.t9	P8_29
config-pin P8_42 pruout
config-pin P8_39 pruout
config-pin P8_40 pruout
config-pin P8_27 pruout
config-pin P8_29 pruout
