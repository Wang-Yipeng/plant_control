import os
import array
import serial
import threading
import numpy as np
import time
import pyqtgraph as pg
from queue import Queue

i = 0
q = Queue(maxsize=0)

def Serial():
    global i;
    global q;
    portx = '/dev/pts/2'
    bps = 115200
    # 串口执行到这已经打开 再用open命令会报错
    mSerial = serial.Serial(portx, int(bps))
#    fo = open("foo_big.txt", "w")

    while(True):
        n = mSerial.inWaiting()  #inWaitting返回接收字符串的长度值。
        if(n):
            dat = mSerial.read()
    #                q.put(dat)
            print(dat)
    #                TextArea.text=dat
    #                fo.write(dat)
#        time.sleep(0.1)

    mSerial.close()
#        fo.close()


data1=1
data2=2
data3=3
data4=4
data5=5
data6=6

def plotsetData():
#    curve1.setData(data1)
#    curve2.setData(data2)
#    curve3.setData(data3)
#    curve4.setData(data4)
#    curve5.setData(data5)
#    curve6.setData(data6)

def plotData():
    app2 = pg.mkQApp()  # 建立app
    win = pg.GraphicsWindow()  # 建立窗口
    win.setWindowTitle(u'pyqtgraph逐点画波形图')
    win.resize(2400, 1800)  # 小窗口大小
    data = array.array('i')  # 可动态改变数组的大小,double型数组
    historyLength = 2000  # 横坐标长度
    a = 0
    data1 = np.zeros(historyLength).__array__('d')  # 把数组长度定下来
    data2 = np.zeros(historyLength).__array__('d')  # 把数组长度定下来
    data3 = np.zeros(historyLength).__array__('d')  # 把数组长度定下来
    data4 = np.zeros(historyLength).__array__('d')  # 把数组长度定下来
    data5 = np.zeros(historyLength).__array__('d')  # 把数组长度定下来
    data6 = np.zeros(historyLength).__array__('d')  # 把数组长度定下来

    p1 = win.addPlot(title="Z_acc")
    curve1 = p1.plot(pen='y')
    win.nextRow()
    p2 = win.addPlot(title="Y_acc")
    curve2 = p2.plot(pen='r')
    win.nextRow()
    p3 = win.addPlot(title="X_acc")
    curve3 = p3.plot(pen='g')
    win.nextRow()
    p4 = win.addPlot(title="Z_gyo")
    curve4 = p4.plot(pen='y')
    win.nextRow()
    p5 = win.addPlot(title="Y_gyo")
    curve5 = p5.plot(pen='r')
    win.nextRow()
    p6 = win.addPlot(title="X_gyo")
    curve6 = p6.plot(pen='g')

    curve1.setData(data1)
    curve2.setData(data2)
    curve3.setData(data3)
    curve4.setData(data4)
    curve5.setData(data5)
    curve6.setData(data6)



    th1 = threading.Thread(target=Serial)  # 目标函数一定不能带（）被这个BUG搞了好久
    th1.start()

#    mSerial.close()

#    timer = pg.QtCore.QTimer()
#    timer.timeout.connect(plotData)  # 定时刷新数据显示
#    timer.start(50)  # 多少ms调用一次
#    app2.exec_()
