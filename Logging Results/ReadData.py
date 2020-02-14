import serial

ser = serial.Serial('COM3', 9600)

with open("TempData.txt", "w") as f:
    reading = True
    index = 0
    f.write('\r')
    while reading:
        line = ser.readline().decode()
        line = line.replace('\r', '')

        if(line == "END MEASUREMENTS\n"):
            reading = False
        else:
            f.write(line)
        index+=1
    f.close()

print("Reading Complete")