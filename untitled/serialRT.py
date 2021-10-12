import serial
import time
import struct
import math

CRC32_POLYNOMIAL = 0xEDB88320


def CRC32Value(i):

    ulCRC = i
    for j in [8, 7, 6, 5, 4, 3, 2, 1]:
        if ulCRC & 1:
            ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL
        else:
            ulCRC >>= 1

    return ulCRC


def CalculateBlockCRC32(ucBuffer):
    ulCRC = 0
    for i in ucBuffer:
        ulTemp1 = (ulCRC >> 8) & 0x00FFFFFF
        ulTemp2 = CRC32Value(((ulCRC ^ i) & 0xFF))
        ulCRC = ulTemp1 ^ ulTemp2

    return ulCRC


def SerialRx(mSerial):

    while(True):
        n = mSerial.inWaiting()  # inWaitting返回接收字符串的长度值。
        if(n):
            dat = mSerial.read()
            print('接收到数据 %c', dat)
#           TextArea.text=dat

        time.sleep(0.01)


def SerialTx(mSerial):

    i = 0
    while(True):

        HeadSB = bytearray.fromhex('AA 44 13')
        INSPVASB_HEAD_LEN = bytearray.fromhex('58')
        INSPVASB_HEAD_ID = bytearray.fromhex('FC 01')
        INSPVASB_HEAD_GPSweek = bytearray.fromhex('7C 08')

        INSPVASB_HEAD_GPSminisec = struct.pack('<L', i)

        INSPVASB_INF_GPSweek = struct.pack('<L', 2048)
        INSPVASB_INF_GPSsec       = struct.pack('<d', i/10)

        INSPVASB_INF_LATITUDE     = struct.pack('<d',23.021726+i)
        INSPVASB_INF_LONGTITUDE   = struct.pack('<d', 115.125678+i)
        INSPVASB_INF_HEIGHT       = struct.pack('<d', 50+i/100)

        INSPVASB_INF_NORTH_VEL    = struct.pack('<d', 5.4)
        INSPVASB_INF_EAST_VEL     = struct.pack('<d', 1.2)
        INSPVASB_INF_UP_VEL       = struct.pack('<d', 0.1)

        INSPVASB_INF_ROLL         = struct.pack('<d', 0.1)
        INSPVASB_INF_PITCH        = struct.pack('<d', 0.5)
        INSPVASB_INF_AZIMUTH      = struct.pack('<d', i/10000)

        INSPVASB_INF_STATE        = struct.pack('<l', 2048)

        INSPVASB = HeadSB+INSPVASB_HEAD_LEN + INSPVASB_HEAD_ID +\
            INSPVASB_HEAD_GPSweek + INSPVASB_HEAD_GPSminisec +\
            INSPVASB_INF_GPSweek + INSPVASB_INF_GPSsec +\
            INSPVASB_INF_LATITUDE+INSPVASB_INF_LONGTITUDE+INSPVASB_INF_HEIGHT+\
            INSPVASB_INF_NORTH_VEL+INSPVASB_INF_EAST_VEL+INSPVASB_INF_UP_VEL +\
            INSPVASB_INF_ROLL+INSPVASB_INF_PITCH+INSPVASB_INF_AZIMUTH +\
            INSPVASB_INF_STATE

        INSPVASB_CRC32 = struct.pack(
            '<L', CalculateBlockCRC32(INSPVASB))
        INSPVASB = INSPVASB+INSPVASB_CRC32

        result = mSerial.write(INSPVASB)  # 写数据
        print("写入INSPVASB，字节总数位为：", result)
        time.sleep(0.5)

        INSSPDSB_HEAD_LEN = bytearray.fromhex('28')
        INSSPDSB_HEAD_ID = bytearray.fromhex('43 01')

        INSSPDSB_HEAD_GPSweek = bytearray.fromhex('7C 08')
        INSSPDSB_HEAD_GPSminisec = struct.pack('<L', i)

        INSSPDSB_INF_GPSweek = struct.pack('<L', 2048)
        INSSPDSB_INF_GPSsec = struct.pack('<d', i/10)

        INSSPDSB_INF_Trk_gnd = struct.pack('<d', 0.1)
        INSSPDSB_INF_Horizontal_Speed = struct.pack('<d', 0.5)
        INSSPDSB_INF_Vertical_Speed = struct.pack(
            '<d', math.asin(i/5000*math.pi))

        INSSPDSB_INF_STATE = struct.pack('<l', 2048)

        INSSPDSB = HeadSB+INSSPDSB_HEAD_LEN + INSSPDSB_HEAD_ID +\
            INSSPDSB_HEAD_GPSweek + INSSPDSB_HEAD_GPSminisec +\
            INSSPDSB_INF_GPSweek + INSSPDSB_INF_GPSsec +\
            INSSPDSB_INF_Trk_gnd+INSSPDSB_INF_Horizontal_Speed +\
            INSSPDSB_INF_Vertical_Speed +\
            INSSPDSB_INF_STATE

        INSSPDSB_CRC32 = struct.pack(
            '<L', CalculateBlockCRC32(INSSPDSB))
        INSSPDSB = INSSPDSB+INSSPDSB_CRC32

        result2 = mSerial.write(INSSPDSB)  # 写数据
        print("写入INSSPDSB，字节总数位为：", result2)

        i = i+1
        if i > 10000:
            i = 0

        time.sleep(0.5)
