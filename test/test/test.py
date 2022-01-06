import time
import threading
import serial
# f042
#ser = serial.Serial(port='/dev/cu.usbmodem2058335047481')
# f303
#ser = serial.Serial(port='/dev/cu.usbmodem2057385756311')
# F779
ser = serial.Serial(port='/dev/cu.usbmodem3959356533361')
ser.isOpen()
# For windows
#ser.set_buffer_size(rx_size=262144, tx_size=262144)

bytesReceived = 0
minimalSpeed = 10000000
maximumSpeed = 0
counterStep = 0
blockSize = 1
shallExit = 0


def res():
    global bytesReceived
    global minimalSpeed
    global maximumSpeed
    global counterStep
    global blockSize
    global ser
    global shallExit

    if minimalSpeed > bytesReceived:
        if bytesReceived > 0:
            minimalSpeed = bytesReceived
    if maximumSpeed < bytesReceived:
        maximumSpeed = bytesReceived
    bytesReceived = 0

    counterStep = counterStep + 1
    if counterStep > 60:
        print("BlockSize:", blockSize, "Minimal:", minimalSpeed, "Maximum:", maximumSpeed,
              "Average:", round((minimalSpeed+maximumSpeed)/2048), "kb/s")
        with open("result.csv", "a") as myfile:
            myfile.write(str(blockSize) + "," + str(minimalSpeed) + "," + str(maximumSpeed) + "\n")
        ser.read(ser.inWaiting())
        counterStep = 0
        minimalSpeed = 100000000
        maximumSpeed = 0
        blockSize = blockSize * 2

    if shallExit == 0:
        threading.Timer(1, res).start()


with open("result.csv", "w") as myfile:
    myfile.close()

res()


while 1:
    if blockSize > 65536:
        shallExit = 1
        exit(0)
    s = "A" * blockSize
    b = s.encode()
    ser.write(b)
    bytesReceived = bytesReceived + ser.inWaiting()
    ser.read(ser.inWaiting())

