#!/bin/env python3

import serial
import pwn
import subprocess
import sys
import threading
import time

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

with subprocess.Popen(
	[sys.argv[1]], # Point argv[1] to the client executable
	bufsize = 0,
	stdin = subprocess.PIPE,
	stdout = subprocess.PIPE
) as debugger_proc:
	def process_debugger_data():
		while True:
			reply_size = pwn.u32(debugger_proc.stdout.read(0x4))
			reply = debugger_proc.stdout.read(reply_size)
			if reply:
				print(f'->DEV\n{pwn.hexdump(reply)}')
				chunk_len = 3
				for i in range(0, len(reply), chunk_len):
					s.write(reply[i:][0:chunk_len])
					# Reader on the other end is a bit slow
					# this assures no bytes are dropped on the other end
					time.sleep(0.003)

	def process_serial_data():
		while True:
			data = s.read(0x1000)
			if data:
				print(f'<-DEV\n{pwn.hexdump(data)}')
				debugger_proc.stdin.write(data)

	t = threading.Thread(target = process_debugger_data)
	t.daemon = False
	t.start()
	process_serial_data()
