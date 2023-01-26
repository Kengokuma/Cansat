#coding: utf-8
 
import bme280
import serial
import micropyGPS
import threading 
import time
import RPi.GPIO as GPIO
import busio
import board
import adafruit_amg88xx
import picamera
import cv2
import numpy as np
import logging
import sys
import time

from Adafruit_BNO055 import BNO055


bno = BNO055.BNO055(rst=18)
GPIO.setmode(GPIO.BCM) #GPIO番号で指定
#GPIO.setmode(GPIO.BOARD) #Pin番号で指定
gpio_sensor = 24 #GPIO番号で指定
GPIO.setup(gpio_sensor,GPIO.IN) #inモードでGPIOを設定

time_start = time.time()


#gps
gps = micropyGPS.MicropyGPS(9, 'dd') # MicroGPSオブジェクトを生成する。
                                     # 引数はタイムゾーンの時差と出力フォーマット

def rungps(): # GPSモジュールを読み、GPSオブジェクトを更新する
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline() # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    while True:
        sentence = s.readline().decode('utf-8') # GPSデーターを読み、文字列に変換する
        if sentence[0] != '$': # 先頭が'$'でなければ捨てる
            continue
        for x in sentence: # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
            gps.update(x)

gpsthread = threading.Thread(target=rungps, args=()) # 上の関数を実行するスレッドを生成
gpsthread.daemon = True
gpsthread.start() # スレッドを起動

while True:
    if gps.clean_sentences > 20: # ちゃんとしたデーターがある程度たまったら出力する
        h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24
        print('緯度経度: %2.8f, %2.8f' % (gps.latitude[0], gps.longitude[0]))
        print('海抜: %f' % gps.altitude)

        #photpreflector
        try :
            ### ------------------- GPIO input ----------------- ###
            input_sensor = GPIO.input(gpio_sensor)
            print('Photo Transister : {}'.format(input_sensor))

            time.sleep(1)
        except KeyboardInterrupt:
            print("\nCtrl+C")
            GPIO.cleanup(gpio_sensor)
            
        
         #humidity and temperature
        csv = bme280.readData()
        list = csv.split(",")
        
        press = list[0]
        temp = list[1]
        hum = list[2]
        
        #kakudo
        heading, roll, pitch = bno.read_euler()
        sys, gyro, accel, mag = bno.get_calibration_status()
        print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
            heading, roll, pitch, sys, gyro, accel, mag))



        with open('test.txt', 'a', encoding='utf-8') as f:
            f.write('緯度経度: %2.8f, %2.8f\n' % (gps.latitude[0], gps.longitude[0]))
            f.write('海抜: %f\n' % gps.altitude)
            f.write("temp = " + temp + " ℃\n")
            f.write("hum = " + hum + " %\n")
            f.write("press = " + press + " hPa\n")
            f.write('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}\n'.format(
            heading, roll, pitch, sys, gyro, accel, mag))
        print ("temp = " + temp + " ℃")
        print ("hum = " + hum + " %")
        print ("press = " + press + " hPa")

                # I2Cバスの初期化
        i2c_bus = busio.I2C(board.SCL, board.SDA)

        # センサーの初期化
        # アドレスを68に指定
        sensor = adafruit_amg88xx.AMG88XX(i2c_bus, addr=0x68)

        # センサーの初期化待ち
        time.sleep(.1)

        # 8x8の表示
        print(sensor.pixels)
        with open('test.txt', 'a', encoding='utf-8') as f:
            #txt = ','.join(str(sensor.pixels))
            f.write(str(sensor.pixels))
            
        with picamera.PiCamera() as camera:
            camera.resolution = (640, 480)
            camera.start_preview()
            time.sleep(1)
            camera.capture('(1).jpg')
        #CASCADE_FIlE
        # 画像の読み込み + グレースケール化
        img = cv2.imread("/home/pi/Run/(9).jpg")
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        template = cv2.imread("/home/pi/Run/(10).jpg")
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

        # 処理対象画像に対して、テンプレート画像との類似度を算出する
        res = cv2.matchTemplate(img_gray, template_gray, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        print(max_val)

        # 類似度の高い部分を検出する
        threshold = 0.8
        loc = np.where(res >= threshold)

        # テンプレートマッチング画像の高さ、幅を取得する
        h, w = template_gray.shape

        # 検出した部分に赤枠をつける
        for pt in zip(*loc[::-1]):
            cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), 2)

        # 画像の保存
        cv2.imwrite("/home/pi/result.jpg", img)
        print("")

    time.sleep(1.0)