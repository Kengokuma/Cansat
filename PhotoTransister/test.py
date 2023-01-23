import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM) #GPIO番号で指定
#GPIO.setmode(GPIO.BOARD) #Pin番号で指定

gpio_sensor = 24 #GPIO番号で指定
GPIO.setup(gpio_sensor,GPIO.IN) #inモードでGPIOを設定

time_start = time.time()

while True :

        ### ------------------- get_time ----------------- ###
        time_end = time.time()
        time_elapsed = time_end - time_start

        ### ------------------- GPIO input ----------------- ###
        input_sensor = GPIO.input(gpio_sensor)

        print('Time : {:.1f}s, GPIO_input : {}'.format(time_elapsed, input_sensor))
        print("")

        time.sleep(1)