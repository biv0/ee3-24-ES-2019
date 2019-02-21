import paho.mqtt.client as mqtt
import json
import queue
import smbus
import RPi.GPIO as gpio
import time
import socket

#########################
######## SETUP ##########
#########################

serial_num = "0000008"
BRKR_ADDR = "broker.hivemq.com"
#BRKR_ADDR = "test.mosquitto.org"
PORT_NUM = 1883
#PORT_NUM = 8884
pubTopic = "IC.embedded/InternetOfTeam/piToApp/" + serial_num
subTopic = "IC.embedded/InternetOfTeam/appToPi/" + serial_num

bus = smbus.SMBus(1)
DEV_ADDR = 0x48
configBytes = [0x8f, 0x83] #sets read to single shot and the ADC gain to maximum

#BCM pins for GPIO
vib_pin = 17
buz_pin = 18
led_pin = 27
but_pin = 4

#GPIO config
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
gpio.setup(vib_pin, gpio.OUT)
gpio.setup(buz_pin, gpio.OUT)
gpio.setup(led_pin, gpio.OUT)
buzzer = gpio.PWM(buz_pin, 3000) #sets PWM freq for piezo
gpio.setup(but_pin, gpio.IN, pull_up_down=gpio.PUD_UP)

global json_q
json_q = queue.Queue()

global conn_q
conn_q = queue.Queue()

#########################
####### FUNCTIONS #######
#########################

def mqtt_msg_cb(client, data, message):
	strMsg = str(message.payload.decode("utf-8"))
	receJson = json.loads(strMsg)
	json_q.put(receJson)

	print("message received: " ,strMsg)
	print("message topic: ",message.topic)

def mqtt_disconn_cb(client, userdata, rc):
	print("DISCONNECT")
	time.sleep(0.5)
	#automatically reconnect on disconnect
	conn_func(client)

def mqtt_conn_cb(client, userdata, flags, rc):
	print("CONNECTED", userdata, flags, rc)
	client.subscribe(subTopic)
	conn_q.put(str(rc))

def conn_func(client):
	while True:
		client.connect(BRKR_ADDR, port=PORT_NUM)
		client.loop_start()
		print("connecting...")
		time.sleep(5)

		if not conn_q.empty():
			if conn_q.get() == "0": #connection good
				with conn_q.mutex:
					conn_q.queue.clear() #thread safe queue clearing
				break

def convert_raw_ADC(raw_adc, offset):
#	print("raw_adc: ",raw_adc)
	x = raw_adc - offset
#	print("x: ",x)
	y = (0.0683)*abs(x) + 6.746
#	print("y: ",y)
	return y

def read_ADC():
	bus.write_i2c_block_data(DEV_ADDR, 0x01, configBytes)
	data  = bus.read_i2c_block_data(DEV_ADDR, 0x00, 2)
	raw_adc = data[0]*256 + data[1]
	if raw_adc > 32767: 	#convert to correct range
		raw_adc -= 65535
	return raw_adc
 
def beep(delay):
	buzzer.start(50)
	time.sleep(delay)
	buzzer.stop()

def get_ip_address():
	ip_address = '';
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.connect(("8.8.8.8",80))
	ip_address = s.getsockname()[0]
	s.close()
	return ip_address

#calibrate the torque wrench
def zero_wrench():
	while True:
		if gpio.input(but_pin) == gpio.LOW:
			break

	#triple beep on start up
	beep(0.5)
	gpio.output(led_pin,gpio.LOW)
	time.sleep(0.25)
	beep(0.25)
	time.sleep(0.25)
	beep(0.25)
	time.sleep(0.25)

	avgNum = 50 # 50*0.1s = 5secs
	i=0
	ADC_sum = 0
	for i in range(avgNum):
		ADC_sum += read_ADC()
		time.sleep(0.1)

	torqueOffset =  ADC_sum/avgNum

	#beep twice when done
	beep(0.5)
	time.sleep(0.25)
	beep(0.5)
	time.sleep(0.25)

	#return average value from sensor and subtract from offset
	return torqueOffset

#main polling loop
def main():

	#initial connect to broker
	client = mqtt.Client()
	client.on_message = mqtt_msg_cb
	client.on_disconnect = mqtt_disconn_cb
	client.on_connect = mqtt_conn_cb
	#client.tls_set(ca_certs="mosquitto.org.crt",certfile="client.crt",keyfile="client.key")

	#wait for cb to get connection status
	conn_func(client)

	time.sleep(0.2)

	#start calibrating
	tqOff = 5
	torqueLimit=-1 
	current_adc = 0
	msg_flag = False
	gpio.output(led_pin,gpio.HIGH)
	print("start calibration")
	tqOff = zero_wrench()

	#FOR DEBUGGING PURPOSES ONLY
	try:
		ip_addr = get_ip_address()
		client.publish("IC.embedded/InternetOfTeam/ip_addr", str(ip_addr))
		print("IP ADDRESS: ", ip_addr)
	except:
		print("IP ERROR")

	print("main loop")
	jsonMsg = {}
	dictMsg = {"ID":0, "CAR":"", "START":"", "FINISH":"", "TORQUE":0, "FTORQUE":0}
	while True: 
		if not json_q.empty():
			jsonMsg = json_q.get()
			try:
				torqueLimit = int(jsonMsg["TORQUE"])
				print("torque:",torqueLimit)
				if torqueLimit < 200 or torqueLimit > 0:
					dictMsg["TORQUE"] = torqueLimit
					dictMsg["START"] = str(time.ctime(time.time()))
					dictMsg["ID"] = jsonMsg["ID"]
					dictMsg["CAR"] = jsonMsg["CAR"]
					#visual prompt
					gpio.output(led_pin,gpio.HIGH)
					time.sleep(0.1)
					gpio.output(led_pin,gpio.LOW)
				else:
					print("Invalid Torque -> 1")
					client.publish(pubTopic, "TORQUE ERROR1")

			except:
				print("Invalid Torque -> 2")
				client.publish(pubTopic, "TORQUE ERROR2")

		#get info from ADC
		raw_adc = read_ADC()
		currentTorque = convert_raw_ADC(raw_adc, tqOff)

		if currentTorque >= torqueLimit and torqueLimit != -1:
			if not msg_flag:
				msg_flag = True
				dictMsg["FINISH"] = str(time.ctime(time.time())) 
				dictMsg["FTORQUE"] = currentTorque
				done_MSG = json.dumps(dictMsg)
				print("Publishing: ",currentTorque) 
				client.publish(pubTopic, done_MSG, qos=2)

				gpio.output(led_pin, gpio.HIGH)
				gpio.output(vib_pin, gpio.HIGH)
				buzzer.start(50)

		else:
			msg_flag = False
			gpio.output(led_pin, gpio.LOW)
			gpio.output(vib_pin, gpio.LOW)
			buzzer.stop()

		if gpio.input(but_pin) == gpio.LOW:
			tqOff = zero_wrench()

		time.sleep(0.1)


	#shutdown
	client.loop_stop()
	client.disconnect()
	gpio.cleanup()


if __name__ == '__main__':
	print("start")
	main()
