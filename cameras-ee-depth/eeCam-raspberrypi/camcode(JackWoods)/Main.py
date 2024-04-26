#!/usr/bin/env python3

while True:
    
    programRun = 1   #Set to 0 to prevent running on startup
                     #Set to 1 to run
    import time   
    import datetime
    import colorsys
    import sys
    import ST7735
    
    if programRun == 0:   #Ends program if programRun is 0
        
        sys.exit(0)

    try:
        
        from ltr559 import LTR559
        ltr559 = LTR559()
        
    except ImportError:
        
        import ltr559

    from bme280 import BME280
    from pms5003 import PMS5003, ReadTimeoutError as pmsReadTimeoutError, SerialTimeoutError
    from enviroplus import gas
    from subprocess import PIPE, Popen
    from PIL import Image
    from PIL import ImageDraw
    from PIL import ImageFont
    from fonts.ttf import RobotoMedium as UserFont
    import logging
    import logging.config

    import os
    import shutil
    from picamera import PiCamera
    from time import sleep
    from picamera import PiCamera, Color

    import RPi.GPIO as GPIO

    GPIO.setwarnings(False)   #Sets up GPIO pins
    GPIO.setmode(GPIO.BCM)
    
    switchPin = 25

    GPIO.setup(switchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)   #Sets parameters for detecting recording switch
    
    st7735 = ST7735.ST7735(   #Sets up display
        port=0,
        cs=1,
        dc=9,
        backlight=12,
        rotation=270,
        spi_speed_hz=10000000
    )
    
    st7735.begin()   #Starts display

    WIDTH = st7735.width
    HEIGHT = st7735.height

    img = Image.new('RGB', (WIDTH, HEIGHT), color=(0, 0, 0))   #Clears screen
    draw = ImageDraw.Draw(img)
    
    fileTransfer = 0   #Safegaurd against wrongful file transfer
    
    version = 0   #Counts version of data log

    def Camera_A():   #Sets up parameter for visual camera
            
        os.system('i2cset -y 0 0x70 0x00 0x01')

        GPIO.output(17, False)
        GPIO.output(4, False)
        
    def Camera_B():   #Sets up parameter for infrared camera
        
        os.system('i2cset -y 0 0x70 0x00 0x02')

        GPIO.output(17, False)
        GPIO.output(4, True)

    GPIO.setup(17, GPIO.OUT)   #Sets use for GPIO pins 17 & 4
    GPIO.setup(4, GPIO.OUT)
        
    screen_message = ""   #Used in shell animation
    lists = 0
    load = ["   ",".  ",".. ","..."]
    
    Camera_A()   #Select with Camera A

    camera = PiCamera()   #Sets up camera for recording
    
    camera.resolution = (1920, 1080)
    camera.framerate = 30

    camera.annotate_text_size = 18   #Sets up text for data display
    camera.annotate_background = Color('blue')
    camera.annotate_foreground = Color('white')

    def save_data(idx, data):   #Controls camera and logging
        
        global screen_message
        global lists
        global add_on
        
        variable = variables[idx]   #Grabs data from sensors
        values[variable] = values[variable][1:] + [data]
        
        unit = units[idx]
        
        if lists == 6:   #Formats data
            
            message = "{}: {:.1f} {}".format(variable[:4], data, unit) + "\n"
            
        else:
            
            message = "{}: {:.1f} {}".format(variable[:4], data, unit)
        
        logging.info(message)   #Prints and logs the data
        print(message)
        
        if add_on == 1:   #Checks for PM sensor
            
            read_out = 9
            
        else:
            
            read_out = 6   #Sets the length of data list
            
        if lists < read_out:   #Formats data from each sensor into list
            
            if lists == 0:
                
                screen_message = screen_message + "  Time: " + str(datetime.datetime.now())[:-7] + "  \n  " + str(message) + "  \n"   #Adds time to first line
                lists = lists + 1
                
            else:
                
                screen_message = screen_message + "  " + str(message) + "  \n"
                lists = lists + 1
        
        else:
            
            camera.annotate_text = (screen_message + str(message))[:-1]   #Displays new data to camera output
            screen_message = ""
            lists = 0
            
            time = str(datetime.datetime.now())[11:-7]   #Checks time
            time = time.replace(":", ".")
            
            Camera_B()   #Takes picture with infrared camera
            camera.capture(location + '/IR_Images/' + time + '.jpg')
            Camera_A()
            
    def display_everything():   #Controls screen
        
        draw.rectangle((0, 0, WIDTH, HEIGHT), (0, 0, 0))  #Creates display output
        column_count = 2
        row_count = (len(variables) / column_count)
        
        for i in range(len(variables)):   #Collects data and displays it to screen
            
            variable = variables[i]
            data_value = values[variable][-1]
            
            unit = units[i]
            
            x = x_offset + ((WIDTH / column_count) * (i / row_count))
            y = y_offset + ((HEIGHT / row_count) * (i % row_count))
            
            message = "{}: {:.1f} {}".format(variable[:4], data_value, unit)
            
            lim = limits[i]
            rgb = palette[0]
            
            for j in range(len(lim)):
                
                if data_value > lim[j]:
                    
                    rgb = palette[j + 1]
                    
            draw.text((x, y), message, font=smallfont, fill=rgb)
            
        st7735.display(img)


    def main():   #Collects data from sensors

        delay = 0.5   #Parameters for collection
        mode = 10    
        last_page = 0

        for v in variables:
            
            values[v] = [1] * WIDTH   #Formats data for screen

        try:
        
            while True:
                
                if GPIO.input(switchPin) == True:   #Checks if recording switch is off
                    
                    recordingEnd()
                    
                    img = Image.new('RGB', (WIDTH, HEIGHT), color=(0, 0, 0))   #Clears screen
                    st7735.display(img)
                    
                    return()
                
                else:
            
                    proximity = ltr559.get_proximity()   #Names light sensor

                    if mode == 0:   #Grabs temperature data

                        unit = "C"
                        
                        raw_temp = bme280.get_temperature()
                        
                        data = raw_temp

                    if mode == 1:   #Grabs air pressure data
                        
                        unit = "hPa"
                        
                        data = bme280.get_pressure()

                    if mode == 2:   #Grabs humidity data
                        
                        unit = "%"
                        
                        data = bme280.get_humidity()

                    if mode == 3:   #Grabs light data
                        
                        unit = "Lux"
                        
                        if proximity < 10:   #Fixes data for light
                            
                            data = ltr559.get_lux()
                        
                        else:
                            
                            data = 1

                    if mode == 4:   #Grabs oxidising data
                       
                        unit = "kO"
                        
                        data = gas.read_all()
                        data = data.oxidising / 1000   #Scales data

                    if mode == 5:   #Grabs reducing data
                       
                        unit = "kO"
                        
                        data = gas.read_all()
                        data = data.reducing / 1000   #Scales data

                    if mode == 6:   #Grabs nh3 data

                        unit = "kO"
                        
                        data = gas.read_all()
                        data = data.nh3 / 1000   #Scales data

                    if mode == 7:   #Grabs PM data (If PM sensor is connected)
                        
                        unit = "ug/m3"
                        
                        try:
                            
                            data = pms5003.read()
                            
                        except pmsReadTimeoutError:   #Solves error for if sensor is not connected
                            logging.warning("Failed to read PMS5003")
                            
                        else:
                            
                            data = float(data.pm_ug_per_m3(1.0))

                    if mode == 8:   #Grabs PM data (If PM sensor is connected)
                        
                        unit = "ug/m3"
                        
                        try:
                            
                            data = pms5003.read()
                            
                        except pmsReadTimeoutError:   #Solves error for if sensor is not connected
                            
                            logging.warning("Failed to read PMS5003")
                            
                        else:
                            
                            data = float(data.pm_ug_per_m3(2.5))

                    if mode == 9:   #Grabs PM data (If PM sensor is connected)
                        
                        unit = "ug/m3"
                        
                        try:
                            
                            data = pms5003.read()
                            
                        except pmsReadTimeoutError:   #Solves error for if sensor is not connected
                            
                            logging.warning("Failed to read PMS5003")
                            
                        else:
                            
                            data = float(data.pm_ug_per_m3(10))
                            
                    if mode == 10:   #sends data to save_data() function and displays it
                        
                        raw_temp = bme280.get_temperature()
                        raw_data = raw_temp
                        save_data(0, raw_data)
                        display_everything()
                        raw_data = bme280.get_pressure()
                        save_data(1, raw_data)
                        display_everything()
                        raw_data = bme280.get_humidity()
                        save_data(2, raw_data)
                        
                        if proximity < 10:   #Fixes data value for light
                            
                            raw_data = ltr559.get_lux()
                            
                        else:
                            
                            raw_data = 1
                            
                        save_data(3, raw_data)
                        display_everything()
                        gas_data = gas.read_all()
                        save_data(4, gas_data.oxidising / 1000)
                        save_data(5, gas_data.reducing / 1000)
                        save_data(6, gas_data.nh3 / 1000)
                        
                        display_everything()
                        pms_data = None
                        
                        try:   #Trys PM again
                            
                            pms_data = pms5003.read()
                            
                        except (SerialTimeoutError, pmsReadTimeoutError):
                            
                            if GPIO.input(switchPin) == False:   #Displays error if PM is not connected
                            
                                message = "Failed to read PMS5003"
                                
                                logging.warning(message)
                                print(message)
                            
                        else:
                            
                            save_data(7, float(pms_data.pm_ug_per_m3(1.0)))
                            save_data(8, float(pms_data.pm_ug_per_m3(2.5)))
                            save_data(9, float(pms_data.pm_ug_per_m3(10)))
                            
                            global add_on
                            add_on = 1   #Indicates if PM sensor is connected
                            
                            display_everything()

        except KeyboardInterrupt:
            
            programEnd()   #Offers way to end code with keyboard interupt
        
    def recordingEnd():   #Stops the camera recording and logs the completion
        
        camera.stop_preview()
        camera.stop_recording()

        message = "Recording Completed\n"

        logging.info(message)
        print(message)
            
    def programEnd():   #Terminates the program and ends the recording
        
        message = "Program Teminated\n"

        logging.info(message)
        print(message)
        
        recordingEnd()
            
        sys.exit(0)   #Ends program
            
    if __name__ == "__main__":   #Starts code
        
        while True:
            
            if GPIO.input(switchPin) == False:   #Checks if switch has been turned on for recording
                
                fileTransfer = 1  #Enables file tranfer
                    
                if os.path.exists('/media/pi/CUBEDATA/'):   #Checks for flash drive named CUBEDATA
                    
                    folder = str('/media/pi/CUBEDATA/CubeSat_Data_and_Video/')
                    
                else:
                    
                    folder = str('/home/pi/Desktop/CubeSat_Data_and_Video/')
                    
                stamp = str(datetime.datetime.now())[:-7]   #Gets time
                stamp = (stamp.replace(" ", "_")).replace(":", ".")

                location = str(folder + stamp)

                if not os.path.exists(location):   #Makes folder location with time
                    
                    os.makedirs(location + '/IR_Images')

                tempFile = (folder + 'temp.log')   #Names a temporary log
                realFile = (location + '/data.log')   #Names a log
                
                logging.basicConfig(filename=tempFile,   #Creates and formats a temporary log
                                    filemode='w',
                                    format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s',
                                    level=logging.INFO,
                                    datefmt='%Y-%m-%d %H:%M:%S')
                
                version = version + 1   #Adds 1 to the version number
                
                startMessage = ("--- Version: " + str(version) + " Start ---\n")   #Names starts and end points to temporary log
                endMessage = ("--- Version: " + str(version) + " End ---\n")
                
                message = startMessage   #Logs version start
                
                logging.info(message)
                
                message = "Program Initiated\n"   #Logs start of program
                
                logging.info(message)
                print(message)

                message = "A3SAT ORIGINAL DATALOG"   #Logs title
                
                logging.info(message)
                print(message)

                camera.start_preview(alpha = 100)   #Starts camera display & recording
                camera.start_recording(location + '/video.h264')

                message = "Recording Started\n"   #Logs start of recording

                logging.info(message)
                print(message)

                bme280 = BME280()   #Starts display with no data
                add_on = 0
                pms5003 = PMS5003()
                time.sleep(1.0)

                img = Image.new('RGB', (WIDTH, HEIGHT), color=(0, 0, 0))   #Clears screen
                draw = ImageDraw.Draw(img)

                font_size_small = 10   #Sets screen parameters
                font_size_large = 20
                font = ImageFont.truetype(UserFont, font_size_large)
                smallfont = ImageFont.truetype(UserFont, font_size_small)

                x_offset = 2
                y_offset = 2

                message = ""

                top_pos = 25

                variables = ["temperature",   #Creates variables, units,limits, and palette for data
                             "pressure",
                             "humidity",
                             "light",
                             "oxidised",
                             "reduced",
                             "nh3",
                             "pm1",
                             "pm25",
                             "pm10"]

                units = ["C",
                         "hPa",
                         "%",
                         "Lux",
                         "kO",
                         "kO",
                         "kO",
                         "ug/m3",
                         "ug/m3",
                         "ug/m3"]

                limits = [[4, 18, 28, 35],
                          [250, 650, 1013.25, 1015],
                          [20, 30, 60, 70],
                          [-1, -1, 30000, 100000],
                          [-1, -1, 40, 50],
                          [-1, -1, 450, 550],
                          [-1, -1, 200, 300],
                          [-1, -1, 50, 100],
                          [-1, -1, 50, 100],
                          [-1, -1, 50, 100]]

                palette = [(0, 0, 255),           
                           (0, 255, 255),      
                           (0, 255, 0),        
                           (255, 255, 0),         
                           (255, 0, 0)]           

                values = {}

                main()   #Runs main to collect and display data
                
            else:   #Checks if recording switch is off
                
                if fileTransfer == 1:   #Checks if file transfer is enabled
                    
                    if os.path.exists(tempFile):   #Transfers data from previos version from the temporary log to the log
                        
                        message = endMessage
                
                        logging.info(message)
                
                        tempFileread = open(tempFile, "r")   #Opens both files
                        realFilewrite = open(realFile, "a")
                        
                        lineTransfer = 0
                        
                        for line in tempFileread:  #Transfers data line by line between files
                                
                            if endMessage in line:
                                
                                break
                                
                            if lineTransfer == 1:
                                
                                realFilewrite.write(line)
                                
                            if startMessage in line:
                                
                                lineTransfer = 1
                            
                        tempFileread.close()   #Closes both files
                        realFilewrite.close()
                        
                    fileTransfer = 0   #Disables file transfer
                
                for i in load:   #Displays standby animation while checking for recording switch to be flipped
                    
                    if GPIO.input(switchPin) == False:
                        
                        break
    
                    print('\rStandby' + i,sep='', end='', flush=True)
                    time.sleep(1)
                    