# chapter 28 in python

# sudo apt-get install python3-pip
# python3 -m pip install pyserial
# sudo apt-get install python3-matplotlib

import serial
ser = serial.Serial('COM8',230400)
print('Opening port: ')
print(ser.name)

import matplotlib.pyplot as plt 
from statistics import mean 
def read_plot_matrix():
    n_str = ser.read_until(b'\n');  # get the number of data points to receive
    n_int = int(n_str) # turn it into an int
    print('Data length = ' + str(n_int))
    ref = []
    data = []
    data_received = 0
    while data_received < n_int:
        dat_str = ser.read_until(b'\n');  # get the data as a string, ints seperated by spaces
        dat_f = list(map(float,dat_str.split())) # now the data is a list
        ref.append(dat_f[0])
        data.append(dat_f[1])
        data_received = data_received + 1
    meanzip = zip(ref,data)
    meanlist = []
    for i,j in meanzip:
        meanlist.append(abs(i-j))
    score = mean(meanlist)
    t = range(len(ref)) # index array
    plt.plot(t,ref,'r*-',t,data,'b*-')
    plt.title('Score = ' + str(score))
    plt.ylabel('value')
    plt.xlabel('index')
    plt.show()

from genref import genRef

def genRef(method):
    if method == 'step':
        print('Step!')
    elif method == 'cubic':
        print('Cubic!')
    else:
        print('Not a valid method')
        return [-1]

    reflist = [0,0,1,90,2,45,3,45] # defualt refs
    
    refs_str = input('Enter times and angles, starting at t=0 (ex: 0 0 1 90 2 90 3 0): ')
    reflist = list(map(float,refs_str.split()))

    # check to see if the reflist is even and the odd values (times) are increasing
    if (len(reflist)%2!=0 or len(reflist) < 2 or reflist[0]!=0):
        print('Not a valid input: odd number of inputs or too short')
        return [-1]
    dataok = 1
    for i in range(2, len(reflist), 2):
        if reflist[i] <= reflist[i-2]:
            dataok = 0
    if dataok == 0:
        print('Not a valid input: time must increase')
        return [-1]

    MOTOR_SERVO_RATE = 200 # the position control ISR is 200Hz
    dt = 1/MOTOR_SERVO_RATE # time per control cycle

    numpos = int(len(reflist)/2)

    if method == 'step':
        sample_list = [] # time
        angle_list = [] # position
        for i in range(0, len(reflist), 2):
            sample_list.append(reflist[i]*MOTOR_SERVO_RATE)
        for i in range(1, len(reflist), 2):
            angle_list.append(reflist[i])
        ref = [0]*int(sample_list[-1])
        last_sample = 0
        for i in range(1, numpos):
            for samp in range(int(last_sample), int(sample_list[i])):
                ref[samp] = angle_list[i-1]
            last_sample = sample_list[i]
        ref[samp] = angle_list[-1]
    
    if method == 'cubic':
        ref = [] # store the output trajectory
        time_list = [] # time
        pos_list = [] # position
        for i in range(0, len(reflist), 2):
            time_list.append(reflist[i])
        for i in range(1, len(reflist), 2):
            pos_list.append(reflist[i])
        vel_list=[0]*numpos
        if numpos >= 3:
            for i in range(1, numpos-1):
                vel_list[i] = (pos_list[i+1]-pos_list[i-1])/(time_list[i+1]-time_list[i-1])
        #print(vel_list)
        refCtr = 0
        for i in range(0,numpos-1):
            timestart = time_list[i]
            timeend = time_list[i+1]
            deltaT = timeend-timestart
            posstart = pos_list[i]
            posend = pos_list[i+1]
            velstart = vel_list[i]
            velend = vel_list[i+1]
            a0 = posstart # calculate coeffs of traj pos = a0+a1*t+a2*t^2+a3*t^3
            a1 = velstart
            a2 = (3*posend - 3*posstart - 2*velstart*deltaT - velend*deltaT)/(deltaT*deltaT)
            a3 = (2*posstart + (velstart+velend)*deltaT - 2*posend)/(deltaT*deltaT*deltaT)
            while (refCtr)*dt < time_list[i+1]:
                tseg = (refCtr)*dt -time_list[i]
                ref.append(int(a0 + a1*tseg + a2*tseg*tseg + a3*tseg*tseg*tseg))
                refCtr = refCtr + 1

    return ref

has_quit = False
# menu loop
while not has_quit:
    print('PIC32 MOTOR DRIVER INTERFACE')
    # display the menu options; this list will grow
    print('a: Read current sensor (ADC counts) \tb: Read current sensor (mA) \tc:Read encoder (counts) \td: Read encoder (deg) \te: Reset encoder \tf: Set PWM (-100 to 100) \tg: Set Current Kp and Ki Gains \th: Get Current Gains \ti: Set position gains \tj: Get position gains \tk: Test Current Gains \tl: Go to Angle (deg) \tm: Load step trajectory \tn: Load cubic trajectory \to: Execute trajectory \tp: Unpower the Motor \tq: Quit \tr: Get Mode') # '\t' is a tab
    # read the user's choice
    selection = input('\nENTER COMMAND: ')
    selection_endline = selection+'\n'
     
    # send the command to the PIC32
    ser.write(selection_endline.encode()); # .encode() turns the string into a char array
    
    # take the appropriate action
    # there is no switch() in python, using if elif instead
    if (selection == 'a'): # Read current sensor (ADC counts)
        sensor_str = ser.read_until(b'\n')
        sensor_int = int(sensor_str)
        print('The motor current is ' + str(sensor_int) + ' ADC counts.\n')
    elif (selection == 'b'): # Read current sensor (mA)
        current_str = ser.read_until(b'\n')
        current_int = float(current_str)
        print('The motor current is ' + str(current_int) + ' mA.\n')
    elif (selection == 'c'): # Read encoder (counts)
        #read until get value back
        encoder_count_str = ser.read_until(b'\n')
        encoder_count_int = int(encoder_count_str)
        print('The motor angle is ' + str(encoder_count_int) + ' counts.\n') # print to the screen
    elif (selection == 'd'): # Read encoder (deg)
        encoder_deg_str = ser.read_until(b'\n')
        encoder_deg_int = int(encoder_deg_str)
        print('The motor angle is ' + str(encoder_deg_int) + ' degrees.\n') # print to the screen
    elif (selection == 'e'): # Resetting encoder
        print('The motor angle has been reset to 0 degrees.\n') # print to the screen
    elif (selection == 'f'): # Set PWM (-100 to 100)
        pwm_input = input('Enter PWM value (-100 to 100): ')
        pwm_int = int(pwm_input)
        print('Setting PWM value to ' + str(pwm_int) + '.\n')
        ser.write((str(pwm_int)+'\n').encode()) # send number to PIC
        pic_pwm_str = ser.read_until(b'\n')
        pic_pwm_int = int(pic_pwm_str)
        print('Finished setting PWM value to ' + str(pic_pwm_int)+ '.\n')
    elif (selection == 'g'): # Set current Kp and Ki gains        
        Kp_input = input('Enter a value for current Kp: ')
        Kp_float = float(Kp_input)
        ser.write((str(Kp_float) + '\n').encode()) # send number to pic
        pic_Kp_val = ser.read_until(b'\n')
        pic_Kp_val_float = float(pic_Kp_val)
        print('Finished setting current Kp to ' + str(pic_Kp_val_float) + '.\n')
        Ki_input = input('Enter a value for current Ki: ')
        Ki_float = float(Ki_input)
        ser.write((str(Ki_float) + '\n').encode()) # send number to pic
        pic_Ki_val = ser.read_until(b'\n')
        pic_Ki_val_float = float(pic_Ki_val)
        print('Finished setting current Ki to ' + str(pic_Ki_val_float) + '.\n')
    elif (selection == 'h'): # get current gains
        current_Kp_str = ser.read_until(b'\n')
        current_Kp_float = float(current_Kp_str)
        print('The current Kp value is ' + str(current_Kp_float) + '.\n')
        current_Ki_str = ser.read_until(b'\n')
        current_Ki_float = float(current_Ki_str)
        print('The current Ki value is ' + str(current_Ki_float) + '.\n')
    elif (selection == 'i'): # set position gains
        pos_Kp_input = input('Enter a value for position Kp: ')
        pos_Kp_float = float(pos_Kp_input)
        ser.write((str(pos_Kp_float) + '\n').encode()) # send number to pic
        pos_pic_Kp_val = ser.read_until(b'\n')
        pos_pic_Kp_val_float = float(pos_pic_Kp_val)
        print('Finished setting position Kp to ' + str(pos_pic_Kp_val_float) + '.\n')
        pos_Ki_input = input('Enter a value for position Ki: ')
        pos_Ki_float = float(pos_Ki_input)
        ser.write((str(pos_Ki_float) + '\n').encode()) # send number to pic
        pos_pic_Ki_val = ser.read_until(b'\n')
        pos_pic_Ki_val_float = float(pos_pic_Ki_val)
        print('Finished setting position Ki to ' + str(pos_pic_Ki_val_float) + '.\n')
        pos_Kd_input = input('Enter a value for position Kd: ')
        pos_Kd_float = float(pos_Kd_input)
        ser.write((str(pos_Kd_float) + '\n').encode()) # send number to pic
        pos_pic_Kd_val = ser.read_until(b'\n')
        pos_pic_Kd_val_float = float(pos_pic_Kd_val)
        print('Finished setting position Kd to ' + str(pos_pic_Kd_val_float) + '.\n')
    elif (selection == 'j'): # get position gains
        position_Kp_str = ser.read_until(b'\n')
        position_Kp_float = float(position_Kp_str)
        print('The position Kp value is ' + str(position_Kp_float) + '.\n')
        position_Ki_str = ser.read_until(b'\n')
        position_Ki_float = float(position_Ki_str)
        print('The position Ki value is ' + str(position_Ki_float) + '.\n')
        position_Kd_str = ser.read_until(b'\n')
        position_Kd_float = float(position_Kd_str)
        print('The position Kd value is ' + str(position_Ki_float) + '.\n')
    elif (selection == 'k'): # test current gains
        print('Testing current gains.\n') # print it to the screen
        read_plot_matrix()
    elif (selection == 'l'): # go to angle (deg)
        desired_angle_input = input('Enter the desired motor angle in degrees: ')
        desired_angle_float = float(desired_angle_input)
        ser.write((str(desired_angle_float) + '\n').encode()) # send number to pic
        pic_angle = ser.read_until(b'\n')
        pic_angle = float(pic_angle)
        print('Motor moving to ' + str(pic_angle) + ' degrees.\n')
    elif (selection == 'm'): # Load step trajectory
        ref = genRef('step')
        t = range(len(ref))
        plt.plot(t,ref,'r*-')
        plt.ylabel('ange in degrees')
        plt.xlabel('index')
        plt.show()
        # send 
        ser.write((str(len(ref))+'\n').encode()) # sending size
        for i in ref:
            ser.write((str(i)+'\n').encode())
        # printing value for debugging purposes
        # for j in range(len(ref)):
        #     data_point = ser.read_until(b'\n')
        #     data_point_int = int(data_point)
        #     print(str(data_point_int) + '\n')
    elif (selection == 'n'): # Load cubic trajectory
        ref = genRef('cubic')
        #print(len(ref))
        t = range(len(ref))
        plt.plot(t,ref,'r*-')
        plt.ylabel('ange in degrees')
        plt.xlabel('index')
        plt.show()
        # send 
        ser.write((str(len(ref))+'\n').encode())
        for i in ref:
            ser.write((str(i)+'\n').encode())
        # printing values for debugging purposes
        # for j in range(len(ref)):
        #     data_point = ser.read_until(b'\n')
        #     data_point_int = int(data_point)
        #     print(str(data_point_int) + '\n')
    elif (selection == 'o'): # Execute trajectory
        sampnum = 0
        read_samples = 10
        posVal = []
        ref = []
        while read_samples > 1:
            data_read = ser.read_until(b'\n',50)
            data_text = str(data_read,'utf-8')
            data = list(map(int,data_text.split()))

            if(len(data)==3):
                read_samples = data[0]
                posVal.append(data[1])
                ref.append(data[2])
                sampnum = sampnum + 1

        errors = [abs(posVal[i] - ref[i]) for i in range(len(posVal))]
        average_error = sum(errors) / len(errors)
        t = range(len(posVal)) # time array
        plt.plot(t,posVal,'r*-',t,ref,'b*-')
        plt.ylabel('value')
        plt.xlabel('sample')
        plt.text(0.5, 1.05, f'Average Error: {average_error:.2f}', transform=plt.gca().transAxes, ha='center')
        plt.show()
    elif (selection == 'p'): # Unpower motor
        print('Motor powered off/set to IDLE.\n')
    elif (selection == 'q'):
        print('Exiting client')
        has_quit = True; # exit client
        # be sure to close the port
        ser.close()
    elif (selection == 'r'):
        rec_mode_str = ser.read_until(b'\n')
        rec_mode_int = int(rec_mode_str)
        pos_modes = ['IDLE', 'PWM', 'ITEST', 'HOLD', 'TRACK']
        print('The PIC32 controller mode is currently ' + pos_modes[rec_mode_int] + '.\n') #prints to the screen
    else:
        print('Invalid Selection ' + selection_endline)



