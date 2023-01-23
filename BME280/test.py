#coding: utf-8
 
import bme280

data = bme280.readData()
list1 = data.split(",")
print ("Pressure: "+list1[0]+"hPa")
print ("Temperature: "+list1[1]+"℃")
print ("Hum: "+list1[2]+"%")