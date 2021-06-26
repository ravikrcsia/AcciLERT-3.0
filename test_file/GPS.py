#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO

import serial
import time

ser = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=2)
ser.flushInput()

power_key = 4
rec_buff = ''
rec_buff2 = ''
time_count = 0

raw=''
msg= ''

def main_sms(msg1):
	ser.write(b"AT\r")
	rcv = ser.read(10)
	print(rcv)
	time.sleep(1)

	ser.write(b"AT+CMGF=1\r")
	time.sleep(1)
	ser.write(b'AT+CMGS="9769538452"\r')
	text = msg1
	print("sending message")
	time.sleep(1)
	ser.reset_output_buffer()
	time.sleep(1)
	ser.write(str.encode(text+chr(26)))
	time.sleep(3)
	print("message sent")

def send_at(command,back,timeout):
	
	ser.write((command+'\r\n').encode())
	time.sleep(timeout)
	
	rec_buff = ''
	
	if ser.inWaiting():
		rec_buff = ser.read(ser.inWaiting())
# 		print(rec_buff.decode())

	if rec_buff != '':
		if back not in rec_buff.decode():
			print(command + ' ERROR')
			print(command + ' back:\t' + rec_buff.decode())
			return 0
		else:
			print(rec_buff.decode())
			#To get the lat and log
			raw=str(rec_buff.decode())
			raw=raw[21:].split(",")
			
			if raw !=['']:
				msg='https://www.google.co.in/maps/place/'+raw[3]+","+raw[4]
				print(msg)
				
				main_sms(msg)
				
			return 1
	else:
		
		print('GPS is not ready')
		print("HERE 1")
		return 0

def get_gps_position():
	rec_null = True
	answer = 0
	print('Start GPS session...')
	rec_buff = ''
	gps_power("on")
	
	while rec_null:
		print(">>>>")
		answer = send_at('AT+CGNSINF','+CGNSINF: ',1)
		print("<<<<")
		if 1 == answer:
			answer = 0
			if ',,,,,,' in rec_buff:
				print('GPS is not ready')
				print("HERE 2")
				rec_null = False
				time.sleep(1)
		else:
			print('error %d'%answer)
			rec_buff = ''
			send_at('AT+CGPS=0','OK',1)
			return False
		time.sleep(2)


def power_on(power_key):
	print('SIM7600X is starting:')
	GPIO.setmode(GPIO.BOARD)
	GPIO.setwarnings(False)
# 	GPIO.setup(power_key,GPIO.OUT)
# 	time.sleep(0.1)
# 	GPIO.output(power_key,GPIO.HIGH)
# 	time.sleep(2)
# 	GPIO.output(power_key,GPIO.LOW)
# 	time.sleep(2)
	ser.flushInput()
	print('SIM7600X is ready')

def power_down(power_key):
	print('SIM7600X is loging off:')
# 	gps_power("off")
# 	GPIO.output(power_key,GPIO.HIGH)
# 	time.sleep(3)
# 	GPIO.output(power_key,GPIO.LOW)
# 	time.sleep(2)
	print('Good bye')
	
def gps_power(state):
	if state=="on":
		send_at('AT+CGNSPWR=1','OK',1)
	elif state=="off":
		send_at('AT+CGNSPWR=0','OK',1)
	else:
		print("Invalid state !!!")
	
	time.sleep(2)

try:
	get_gps_position()
	power_down(power_key)
except:
	if ser != None:
		ser.close()
	power_down(power_key)
	GPIO.cleanup()

	
if ser != None:
	ser.close()
	GPIO.cleanup()	

