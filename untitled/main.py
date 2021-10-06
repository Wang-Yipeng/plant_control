# This Python file uses the following encoding: utf-8
import os
from pathlib import Path
import sys

from PySide2.QtGui import QGuiApplication
from PySide2.QtQml import QQmlApplicationEngine

# import numpy as np
import serial
# import pyqtgraph
# import time
import threading
# import array
# from wyp_serial import plotData
# import pyqtgraph as pg
from serialRT import SerialRx
from serialRT import SerialTx



if __name__ == "__main__":
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()
    engine.load(os.fspath(Path(__file__).resolve().parent / "main.qml"))

    portx = '/dev/pts/2'
    bps = 115200
    # 串口执行到这已经打开 再用open命令会报错
    mSerial = serial.Serial(portx, int(bps))
    SerialTx = threading.Thread(target=SerialTx(mSerial))  # 目标函数一定不能带（）被这个BUG搞了好久
    SerialTx.start()

    if not engine.rootObjects():
        sys.exit(-1)

    sys.exit(app.exec_())
#    app2.exec_()
