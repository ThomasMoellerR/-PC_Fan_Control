import time
import numpy as np
import RPi.GPIO as GPIO
from w1thermsensor import W1ThermSensor
import paho.mqtt.client as mqtt
import threading

var_case_rear = 0
var_cpu_rear = 0
var_cpu_front = 0
var_case_front = 0



def set_case_rear(pwm):
    pwm_case_rear.ChangeDutyCycle(pwm)
    global var_case_rear
    var_case_rear = pwm

def set_cpu_rear(pwm):
    if pwm < 7: pwm = 7
    pwm_cpu_rear.ChangeDutyCycle(pwm)
    global var_cpu_rear
    var_cpu_rear = pwm

def set_cpu_front(pwm):
    if pwm < 7: pwm = 7
    pwm_cpu_front.ChangeDutyCycle(pwm)
    global var_cpu_front
    var_cpu_front = pwm

def set_case_front(pwm):
    pwm_case_front.ChangeDutyCycle(pwm)
    global var_case_front
    var_case_front = pwm



def thread1():
    console_counter = 0
    temperature = 0
    sm = "idle"

    while True:

        try:
            sensor = W1ThermSensor()
            temperature = sensor.get_temperature() # beim Abziehen des Sensors kommt hier eine  0 (int) zurück

        except: # ein Abziehen des Sensors wird erst einige Zeit später bemerkt

            print("exception occoured")
            temperature = 0 # temperatur manuell auf 0 setzen

            now = time.localtime(time.time())
            date_string = time.strftime("%Y-%m-%d", now)
            time_string = time.strftime("%H:%M:%S", now)


            f = open("/home/pi/a/log.txt", "a")
            f.write(date_string + " " + time_string + " exception occoured\n")
            f.close()


        if temperature == 0: # im Fehlerfall
            temperature = 100 # maximale Tempeartur => da springen sofort die Lüfter an


        temperature = round(temperature, 0) # nur runden

        console_counter += 1

        print(str(console_counter) + "\t" + sm + "\t" + str(temperature) + "\t" + str(var_case_rear) + "\t" + str(var_cpu_rear) + "\t" + str(var_cpu_front) + "\t" + str(var_case_front))

        if sm == "idle":
            if temperature >= 45:
                set_case_rear(40)
                set_cpu_rear(40)
                set_cpu_front(40)
                set_case_front(40)

                sm = "gaming"


        if sm == "gaming":
            if temperature <= 32:
                set_case_rear(0)
                set_cpu_rear(0)
                set_cpu_front(0)
                set_case_front(0)

                sm = "idle"

        time.sleep(1)






def thread2():
    while True:
        client.on_connect = on_connect
        client.on_message = on_message

        client.connect("192.168.2.52", 1883, 60)

        # Blocking call that processes network traffic, dispatches callbacks and
        # handles reconnecting.
        # Other loop*() functions are available that give a threaded interface and a
        # manual interface.
        client.loop_forever()






# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("pc/fan")



# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic + " "+ msg.payload.decode("utf-8"))

    if msg.topic == "pc/fan":
        set_case_rear(float(msg.payload.decode("utf-8")))
        set_cpu_rear(float(msg.payload.decode("utf-8")))
        set_cpu_front(float(msg.payload.decode("utf-8")))
        set_case_front(float(msg.payload.decode("utf-8")))


f = open("/home/pi/a/log.txt", "w") # clear
f.close()

time.sleep(10) # WLAN Connection
client = mqtt.Client()

t1= threading.Thread(target=thread1)
t2= threading.Thread(target=thread2)

t1.start()
t2.start()




GPIO.setmode(GPIO.BOARD)

# Setup GPIO Pins
GPIO.setup(16, GPIO.OUT) # case hinten
GPIO.setup(18, GPIO.OUT) # cpu hinten
GPIO.setup(38, GPIO.OUT) # cpu vorne
GPIO.setup(40, GPIO.OUT) # case vorne

# Set PWM instance and their frequency
pwm_case_rear = GPIO.PWM(16, 50)
pwm_cpu_rear = GPIO.PWM(18, 50)
pwm_cpu_front = GPIO.PWM(38, 50)
pwm_case_front = GPIO.PWM(40, 50)

# Start PWM with 0% Duty Cycle
pwm_case_rear.start(0)
pwm_cpu_rear.start(0)
pwm_cpu_front.start(0)
pwm_case_front.start(0)


set_case_rear(0)
set_cpu_rear(0)
set_cpu_front(0)
set_case_front(0)
