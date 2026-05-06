#!/usr/bin/env/python
# File name   : WebServer.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date        : 2025/08/11
import time
import threading
import Move as move
import os
import Info as info
import RPIservo
import Functions as functions
import RobotLight as robotLight
import Switch as switch
import asyncio
import websockets
import json
import app
import Voltage
import camera_opencv
import Buzzer

#Buzzer
player = Buzzer.Player()
player.start()

speed_set = 50
rad = 0.5

scGear = RPIservo.ServoCtrl()
scGear.moveInit()
scGear.start()

init_pwm0 = scGear.initPos[0]
init_pwm1 = scGear.initPos[1]
init_pwm2 = scGear.initPos[2]
init_pwm3 = scGear.initPos[3]
init_pwm4 = scGear.initPos[4]

fuc = functions.Functions()
fuc.setup()
fuc.start()

batteryMonitor = Voltage.BatteryLevelMonitor()
batteryMonitor.start()

curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)


def servoPosInit():
    scGear.initConfig(0,init_pwm0,1)
    scGear.initConfig(1,init_pwm1,1)
    scGear.initConfig(2,init_pwm2,1)
    scGear.initConfig(3,init_pwm3,1)
    scGear.initConfig(4,init_pwm4,1)


def replace_num(initial,new_num):   #Call this function to replace data in '.txt' file
    global r
    newline=""
    str_num=str(new_num)
    with open(thisPath+"/RPIservo.py","r") as f:
        for line in f.readlines():
            if(line.find(initial) == 0):
                line = initial+"%s" %(str_num+"\n")
            newline += line
    with open(thisPath+"/RPIservo.py","w") as f:
        f.writelines(newline)


def functionSelect(command_input, response):
    if 'findColor' == command_input:
        flask_app.modeselect('findColor')
        flask_app.modeselectApp('APP')

    elif 'motionGet' == command_input:
        flask_app.modeselect('watchDog')

    elif 'stopCV' == command_input:
        flask_app.modeselect('none')
        scGear.moveServoInit([0])
        time.sleep(0.5)
        move.motorStop()

    elif 'automatic' == command_input:
        functions.last_status = 3
        fuc.automatic()

    elif 'automaticOff' == command_input:
        fuc.pause()
        time.sleep(0.5)
        move.motorStop()

    elif 'trackLine' == command_input:
        functions.last_status = None
        fuc.trackLine()

    elif 'trackLineOff' == command_input:
        fuc.pause()
        time.sleep(0.5)
        move.motorStop()

    elif 'police' == command_input:
        ws2812.police()
        pass

    elif 'policeOff' == command_input:
        ws2812.breath(70,70,255)
        pass

    elif 'keepDistance' == command_input:
        functions.last_status = 25
        fuc.keepDistance()

    elif 'keepDistanceOff' == command_input:
        fuc.pause()
        time.sleep(0.5)
        move.motorStop()

    elif 'Buzzer_Music' == command_input:
        player.start_playing()

    elif 'Buzzer_Music_Off' == command_input:
        player.pause()

def switchCtrl(command_input, response):
    if 'Switch_1_on' in command_input:
        switch.switch(1,1)

    elif 'Switch_1_off' in command_input:
        switch.switch(1,0)

    elif 'Switch_2_on' in command_input:
        switch.switch(2,1)

    elif 'Switch_2_off' in command_input:
        switch.switch(2,0)

    elif 'Switch_3_on' in command_input:
        switch.switch(3,1)

    elif 'Switch_3_off' in command_input:
        switch.switch(3,0) 


def robotCtrl(command_input, response):
    clen = len(command_input.split())
    if 'forward' in command_input and clen == 2 :
        move.move(speed_set, 1, "mid")
    
    elif 'backward' in command_input and clen == 2:
        move.move(speed_set, -1, "mid")

    elif 'left' in command_input and clen == 2:
        move.move(speed_set, 1, "left")

    elif 'right' in command_input and clen == 2:
        move.move(speed_set, 1, "right")

    elif 'DTS' in command_input:
        move.motorStop()
    
    elif 'lookleft' == command_input:
        move.move(speed_set, 1, "rotate-left")
    
    elif 'lookright' == command_input:
        move.move(speed_set, 1, "rotate-right")

    elif 'LRStop' in command_input:
        move.motorStop()

    elif 'up' == command_input:
        scGear.singleServo(0, 1, 7)

    elif 'down' == command_input:
        scGear.singleServo(0, -1, 7)

    elif 'UDstop' in command_input:
        scGear.stopWiggle()


async def recv_msg(websocket):
    global speed_set, modeSelect
    move.setup()

    while True: 
        response = {
            'status' : 'ok',
            'title' : '',
            'data' : None
        }

        data = ''
        data = await websocket.recv()
        try:
            data = json.loads(data)
        except Exception as e:
            print('not A JSON')
        print(data)
        
        if not data:
            continue

        if isinstance(data,str):
            robotCtrl(data, response)

            switchCtrl(data, response)

            functionSelect(data, response)

            if 'get_info' == data:
                response['title'] = 'get_info'
                response['data'] = [info.get_cpu_tempfunc(), info.get_cpu_use(), info.get_ram_info(), batteryMonitor.get_battery_percentage()]

            if 'wsB' in data:
                try:
                    set_B=data.split()
                    speed_set = int(set_B[1]) *10
                except:
                    pass

            #CVFL
            elif 'CVFL' == data:
                camera_opencv.FLCV_Status = 0
                flask_app.modeselect('findlineCV')

            elif 'CVFLColorSet' in data:
                color = int(data.split()[1])
                flask_app.camera.colorSet(color)

            elif 'CVFLL1' in data:
                pos = int(data.split()[1])
                flask_app.camera.linePosSet_1(pos)

            elif 'CVFLL2' in data:
                pos = int(data.split()[1])
                flask_app.camera.linePosSet_2(pos)

            elif 'CVFLSP' in data:
                err = int(data.split()[1])
                flask_app.camera.errorSet(err)

        elif(isinstance(data,dict)):
            color = data['data']
            if "title" in data and data['title'] == "findColorSet":
                flask_app.colorFindSetApp(color[0],color[1],color[2])
            elif data['lightMode'] == "breath":  
                ws2812.breath(color[0],color[1],color[2])
            elif data['lightMode'] == "flowing":
                ws2812.flowing(color[0],color[1],color[2])
            elif data['lightMode'] == "rainbow":
                ws2812.rainbow(color[0],color[1],color[2])
            elif data['lightMode'] == "police":
                ws2812.police()
        else:
            pass
        response = json.dumps(response)
        await websocket.send(response)

async def main_logic(websocket, path):
    await recv_msg(websocket)

if __name__ == '__main__':
    switch.switchSetup()
    switch.set_all_switch_off()

    global flask_app
    flask_app = app.webapp()
    flask_app.startthread()
    ws2812 = robotLight.Adeept_SPI_LedPixel(8, 255)
    try:
        if ws2812.check_spi_state() != 0:
            ws2812.start()
            ws2812.breath(70,70,255)
    except:
        ws2812.led_close()
        pass

    while  1:
        try:                  #Start server,waiting for client
            start_server = websockets.serve(main_logic, '0.0.0.0', 8888)
            asyncio.get_event_loop().run_until_complete(start_server)
            print('waiting for connection...')
            # print('...connected from :', addr)
            break
        except Exception as e:
            print(e)
            ws2812.set_all_led_color_data(0,0,0)
            ws2812.show()

        try:
            ws2812.set_all_led_color_data(0,80,255)
            ws2812.show()
        except:
            pass
    try:
        asyncio.get_event_loop().run_forever()
    except Exception as e:
        print(e)
        ws2812.led_close()
        move.destroy()
    except KeyboardInterrupt:
        ws2812.led_close()
        move.destroy()