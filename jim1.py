#Display Data from Neato LIDAR
#based on code from Nicolas "Xevel" Saugnier
#requires vpython and pyserial


import thread
import thread
import time
import sys
import traceback
import math
import serial

from visual import *

com_port_lidar = "COM5" # example: 5 == "COM6" == "/dev/tty5"
com_port_imu = "COM4" # example: 5 == "COM6" == "/dev/tty5"
baudrate = 115200

lidar = serial.Serial(com_port_lidar, baudrate)
imu = serial.Serial(com_port_imu, baudrate)

visualization = True

max = 65535.0
euler = [0,0,0]

offset = 140
init_level = 0
index = 0

lidarData = [[] for i in range(360)] #A list of 360 elements Angle, Distance , quality


if visualization:
# sample and intensity points
    point = points(pos=[(0,0,0) for i in range(360)], size=5, color=(0 , 1, 0))
    pointb = points(pos=[(0,0,0) for i in range(360)], size=5, color=(0.4, 0, 0))
    point2 = points(pos=[(0,0,0) for i in range(360)], size=3, color=(1 , 1, 0))
    point2b = points(pos=[(0,0,0) for i in range(360)], size=3, color=(0.4, 0.4, 0))
    #lines
    outer_line= curve (pos=[(0,0,0) for i in range(360)], size=5, color=(1 , 0, 0))
    lines=[curve(pos=[(offset*cos(i* pi / 180.0),0,offset*-sin(i* pi / 180.0)),(offset*cos(i* pi / 180.0),0,offset*-sin(i* pi / 180.0))], color=[(0.1, 0.1, 0.2),(1,0,0)]) for i in range(360)]
    zero_intensity_ring = ring(pos=(0,0,0), axis=(0,1,0), radius=offset-1, thickness=1, color = color.yellow)

    label_speed = label(pos = (0,-500,0), xoffset=1, box=False, opacity=0.1)
    label_errors = label(pos = (0,-1000,0), xoffset=1, text="errors: 0", visible = False, box=False)

use_points = True
use_outer_line = False
use_lines = False
use_intensity = True

def update_view( angle, data, heading ):
    """Updates the view of a sample.Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived."""
    global offset, use_outer_line, use_line
    #unpack data using the denomination used during the discussions
    x = data[0]
    x1= data[1]
    x2= data[2]
    x3= data[3]
    
	if heading <=180:
		if angle<=(180+heading):
			angle_new=180+heading-angle
		elif angle > (180 + heading):
			engle_new=540+heading-angle
	elif heading > 180:
		if angle<(heading-180):
			angle_new = 540-heading-angle
		elif angle<=heading:
			angle_new=heading+angle-180
		elif angle>heading:
			angle_new=180+heading-angle
	
    angle_rad = angle_new * math.pi / 180.0
    c = math.cos(angle_rad)
    s = -math.sin(angle_rad)

    dist_mm = x | (( x1 & 0x3f) << 8) # distance is coded on 13 bits ? 14 bits ?
    quality = x2 | (x3 << 8) # quality is on 16 bits
    lidarData[angle_new] = [dist_mm,quality]
    dist_x = dist_mm*c
    dist_y = dist_mm*s
    if visualization:
        #reset the point display
        point.pos[angle_new] = vector( 0, 0, 0 )
        pointb.pos[angle_new] = vector( 0, 0, 0 )
        point2.pos[angle_new] = vector( 0, 0, 0 )
        point2b.pos[angle_new] = vector( 0, 0, 0 )
        if not use_lines : lines[angle_new].pos[1]=(offset*c,0,offset*s)
        if not use_outer_line :
            outer_line.pos[angle_new]=(offset*c,0,offset*s)
            outer_line.color[angle_new] = (0.1, 0.1, 0.2)
        
        
        # display the sample
        if x1 & 0x80: # is the flag for "bad data" set?
            # yes it's bad data
            lines[angle_new].pos[1]=(offset*c,0,offset*s)
            outer_line.pos[angle_new]=(offset*c,0,offset*s)
            outer_line.color[angle_new] = (0.1, 0.1, 0.2)
        else:
            # no, it's cool
            if not x1 & 0x40:
                # X+1:6 not set : quality is OK
                if use_points : point.pos[angle_new] = vector( dist_x,0, dist_y)
                if use_intensity : point2.pos[angle_new] = vector( (quality + offset)*c,0, (quality + offset)*s)
                if use_lines : lines[angle_new].color[1] = (1,0,0)
                if use_outer_line : outer_line.color[angle_new] = (1,0,0)
            else:
                # X+1:6 set : Warning, the quality is not as good as expected
                if use_points : pointb.pos[angle_new] = vector( dist_x,0, dist_y)
                if use_intensity : point2b.pos[angle_new] = vector( (quality + offset)*c,0, (quality + offset)*s)
                if use_lines : lines[angle_new].color[1] = (0.4,0,0)
                if use_outer_line : outer_line.color[angle_new] = (0.4,0,0)
            if use_lines : lines[angle_new].pos[1]=( dist_x, 0, dist_y)
            if use_outer_line : outer_line.pos[angle_new]=( dist_x, 0, dist_y)




def checksum(data):
    """Compute and return the checksum as an int.

data -- list of 20 bytes (as ints), in the order they arrived in.
"""
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append( data[2*t] + (data[2*t+1]<<8) )
    
    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF # truncate to 15 bits
    return int( checksum )


def gui_update_speed(speed_rpm):
    label_speed.text = "RPM : " + str(speed_rpm)

def compute_speed(data):
    speed_rpm = float( data[0] | (data[1] << 8) ) / 64.0
    return speed_rpm

def read_Lidar():
    global init_level, angle, index, euler
    
    nb_errors = 0
    while True:
        try:
            time.sleep(0.00001) # do not hog the processor power

            if init_level == 0 :
                b = ord(lidar.read(1))
                # start byte
                if b == 0xFA :
                    init_level = 1
                    #print lidarData
                else:
                    init_level = 0
            elif init_level == 1:
                # position index
                b = ord(lidar.read(1))
                if b >= 0xA0 and b <= 0xF9 :
                    index = b - 0xA0
                    if index == 0:
                        # readIMU
                        heading=euler[0]
                    init_level = 2
                elif b != 0xFA:
                    init_level = 0
            elif init_level == 2 :
                # speed
                b_speed = [ ord(b) for b in lidar.read(2)]
                
                # data
                b_data0 = [ ord(b) for b in lidar.read(4)]
                b_data1 = [ ord(b) for b in lidar.read(4)]
                b_data2 = [ ord(b) for b in lidar.read(4)]
                b_data3 = [ ord(b) for b in lidar.read(4)]

                # for the checksum, we need all the data of the packet...
                # this could be collected in a more elegent fashion...
                all_data = [ 0xFA, index+0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3

                # checksum
                b_checksum = [ ord(b) for b in lidar.read(2) ]
                incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

                # verify that the received checksum is equal to the one computed from the data
                if checksum(all_data) == incoming_checksum:
                    speed_rpm = compute_speed(b_speed)
                    if visualization:
                        gui_update_speed(speed_rpm)
                    
                    update_view(index * 4 + 0, b_data0, heading)
                    update_view(index * 4 + 1, b_data1, heading)
                    update_view(index * 4 + 2, b_data2, heading)
                    update_view(index * 4 + 3, b_data3, heading)
                else:
                    # the checksum does not match, something went wrong...
                    nb_errors +=1
                    if visualization:
                        label_errors.text = "errors: "+str(nb_errors)
                    
                    # display the samples in an error state
                    update_view(index * 4 + 0, [0, 0x80, 0, 0], heading)
                    update_view(index * 4 + 1, [0, 0x80, 0, 0], heading)
                    update_view(index * 4 + 2, [0, 0x80, 0, 0], heading)
                    update_view(index * 4 + 3, [0, 0x80, 0, 0], heading)
                    
                init_level = 0 # reset and wait for the next packet
                
            else: # default, should never happen...
                init_level = 0
        except :
            traceback.print_exc(file=sys.stdout)

def read_imu():
	global euler
	while True:
		try:
			time.sleep(0.00001)
			byte = ord(imu.read(1))
			if byte == 0:
				byte = ord(imu.read(1))
				if byte == 0:
					byte = ord(imu.read(1))
					if byte == 0:
						raw_data = [ ord(byte) for byte in imu.read(6)]
						x = float(raw_data[0] | (raw_data[1] << 8))
						y = float(raw_data[2] | (raw_data[3] << 8))
						z = float(raw_data[4] | (raw_data[5] << 8))
						if x > 32757:
							x=(x-max)/ 16.0
						else:
							x=x/16.0
						if y > 32757:
							y=(y-max)/ 16.0
						else:
							y=y/16.0
						if z > 32757:
							z=(z-max)/ 16.0
						else:
							z=z/16.0
						euler = [x,y,z]
		except :
			traceback.print_exc(file=sys.stdout)

def checkKeys():
    global use_outer_line, use_lines, use_points, use_intensity
    if scene.kb.keys: # event waiting to be processed?
        s = scene.kb.getkey() # get keyboard info

        if s=="o": # Toggle outer line
            use_outer_line = not use_outer_line
        elif s=="l": # Toggle rays
            use_lines = not use_lines
        elif s=="p": # Toggle points
            use_points = not use_points
        elif s=="i": # Toggle intensity
            use_intensity = not use_intensity
            zero_intensity_ring.visible = use_intensity

        elif s=="n": # Toglle lidar representation
            lidar.visible = not lidar.visible

        elif s=="j": # Toggle rpm
            label_speed.visible = not label_speed.visible
        elif s=="k": # Toggle errors
            label_errors.visible = not label_errors.visible

th = thread.start_new_thread(read_Lidar, ())
th2 = thread.start_new_thread(read_imu, ())

while True:
    if visualization:
        rate(60) # synchonous repaint at 60fps
        checkKeys()
    
