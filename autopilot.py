from dronekit import connect, VehicleMode
import time
import mock


FLIGHT_MODES = ['STABILIZE', 'LAND', 'OF_LOITER', 'RTL', 'DRIFT',
                    'FLIP', 'AUTOTUNE', 'BRAKE','GUIDED_NOGPS', 'AVOID_ADSB',
                    'POSITION', 'SPORT', 'FLOWHOLD', 'POSHOLD','AUTO', 'GUIDED',
                     'ACRO', 'SMART_RTL', 'ALT_HOLD', 'LOITER', 'CIRCLE', 'THROW']

ACK_RESULT_TYPE = {0:'Command Accepted',
                    1:'Command Rejected',
                    2:'Access Denied',
                    3:'Command Not Supported',
                    4:'Command Failed'}


MAVLINK_COMMAND_ID = {'ARM/DISARM':400,
                        'TAKEOFF':22,
                        'CHANGE_FLIGHT_MODE':11}


class autopilot:

    is_waiting_for_ack = False
    ack_command_id = None
    ack_command_result = None
    is_connected = False
    is_armed = False

    def __init__(self):


        self.autopilot = None
        #
        # global is_waiting_for_ack
        # global ack_command_id
        # global ack_command_result
        # is_waiting_for_ack = False
        # ack_command_id = None
        # ack_command_result = None



    def connect(self,ip,port):
        # Connect to the Vehicle.
        global is_connected
        self.autopilot = connect(str(ip)+":"+str(port), wait_ready=True)
        if not self.autopilot is None:
            is_connected = True

            '''
            COMMAND_ACK
            '''
            @self.autopilot.on_message('COMMAND_ACK')
            def listener(self, name, message):
                global is_waiting_for_ack
                global ack_command_id
                global ack_command_result
                if is_waiting_for_ack:
                    if message.command == ack_command_id:
                        ack_command_result = message.result
                        is_waiting_for_ack = False
                    else:
                        print 'message: %s' % message,message.command,ack_command_id


            '''
            HEARTBEAT
            '''
            @self.autopilot.on_message('HEARTBEAT')
            def listener(self, name, message):
                global is_armed
                if (message.base_mode & 0b10000000)==128:
                    is_armed = True
                else:
                    is_armed = False
                # print 'message: %s' % message
                # print (message.base_mode & 0b10000000)


        else:
            self.is_connected = False


    def takeoff(self,altitude):
        global is_connected
        global is_armed
        global is_waiting_for_ack
        global ack_command_id
        global ack_command_result
        if is_connected and is_armed:
            tick = 0
            is_waiting_for_ack = True
            ack_command_id = MAVLINK_COMMAND_ID['TAKEOFF']
            self.autopilot.simple_takeoff(altitude)
            while is_waiting_for_ack:
                tick+=1
                time.sleep(0.5)
                if tick>5:
                    print 'TAKEOFF Time Out!'
                    return
            print 'TAKEOFF: ',ACK_RESULT_TYPE[ack_command_result]

            if ack_command_result==0:
                while True:
                    print " Altitude: ", self.autopilot.location.global_relative_frame.alt
                    #Break and return from function just below target altitude.
                    if self.autopilot.location.global_relative_frame.alt>=altitude*0.95:
                        print "Reached target altitude"
                        break
                    time.sleep(1)
            elif ack_command_result==4 and self.autopilot.location.global_relative_frame.alt>1:
                print 'The Vehicle is already above the ground!, use go_z command... '
                return
        else:
            print 'Vehicle is NOT armed!'


    def change_flight_mode(self,flight_mode_name):
        global is_connected
        global is_armed
        global is_waiting_for_ack
        global ack_command_id
        global ack_command_result
        if is_connected:
            if flight_mode_name.upper() in FLIGHT_MODES:
                tick = 0
                is_waiting_for_ack = True
                ack_command_id = MAVLINK_COMMAND_ID['CHANGE_FLIGHT_MODE']
                self.autopilot.mode = VehicleMode(flight_mode_name.upper())
                while is_waiting_for_ack:
                    tick+=1
                    time.sleep(0.5)
                    if tick>5:
                        print 'CHANGE_FLIGHT_MODE Time Out!'
                        return
                print 'CHANGE_FLIGHT_MODE:',ACK_RESULT_TYPE[ack_command_result]
            else:
                print 'The available flight modes: ',self.flight_modes
        else:
            print 'There is no connection with any vehicle!'



    def arm(self):
        global is_connected
        global is_armed
        global is_waiting_for_ack
        global ack_command_id
        global ack_command_result
        if self.autopilot is None:
            print "There is NO connection with any vehicle!"
            return

        if is_armed:
            print 'The vehicle is already armed!'
        else:
            tick = 0
            is_waiting_for_ack = True
            ack_command_id = MAVLINK_COMMAND_ID['ARM/DISARM']
            self.autopilot.armed = True
            while is_waiting_for_ack:
                tick+=1
                time.sleep(0.5)
                if tick>5:
                    print 'Time Out!'
                    return
            if ack_command_result==0:
                is_armed = True
            print 'ARM: ',ACK_RESULT_TYPE[ack_command_result]

    def disarm(self):
        if not self.is_armed:
            print 'The vehicle is already disarmed!'
        else:
            ack_command_id = MAVLINK_COMMAND_ID['ARM/DISARM']
            self.autopilot.armed = False
            while is_waiting_for_ack:
                tick+=1
                time.sleep(0.5)
                if tick>5:
                    print 'DISARM Time Out!'
                    return
            if ack_command_result==0:
                is_armed = False

            print 'DISARM: ',ACK_RESULT_TYPE[ack_command_result]
            #send mavlink packet

if __name__== "__main__":

    autopilot_vehicle = autopilot()
    autopilot_vehicle.connect('127.0.0.1',14559)
    #
    autopilot_vehicle.change_flight_mode('guided')
    #
    autopilot_vehicle.arm()
    autopilot_vehicle.takeoff(3)




time.sleep(10)
