from dronekit import connect, VehicleMode
import time


class autopilot:

    def __init__(self):

        self.is_connected = False
        self.is_armed = False
        self.autopilot = None
        self.flight_modes = ['STABILIZE', 'LAND', 'OF_LOITER', 'RTL', 'DRIFT',
                            'FLIP', 'AUTOTUNE', 'BRAKE','GUIDED_NOGPS', 'AVOID_ADSB',
                            'POSITION', 'SPORT', 'FLOWHOLD', 'POSHOLD','AUTO', 'GUIDED',
                             'ACRO', 'SMART_RTL', 'ALT_HOLD', 'LOITER', 'CIRCLE', 'THROW']


    def connect(self,ip,port):
        # Connect to the Vehicle.
        self.autopilot = connect(str(ip)+":"+str(port), wait_ready=True)
        if not self.autopilot is None:
            self.is_connected = True
        else:
            self.is_connected = False


    def takeoff(self,altitude):
        if self.is_connected and self.is_armed:
            self.autopilot.simple_takeoff(aTargetAltitude) # Take off to target altitude
            # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
            #  after Vehicle.simple_takeoff will execute immediately).
            while True:
                print " Altitude: ", self.autopilot.location.global_relative_frame.alt
                #Break and return from function just below target altitude.
                if self.autopilot.location.global_relative_frame.alt>=altitude*0.95:
                    print "Reached target altitude"
                    break
                time.sleep(1)
        else:
            print 'Vehicle is NOT armed!'


    def change_flight_mode(self,flight_mode_name):
        if self.is_connected:
            if flight_mode_name.upper() in self.flight_modes:
                self.autopilot.mode = VehicleMode(flight_mode_name.upper())
            else:
                print 'The available flight modes: ',self.flight_modes
        else:
            print 'There is no connection with any vehicle!'



    def arm(self):
        if self.autopilot is None:
            print "There is NO connection with any vehicle!"
            return

        if self.is_armed:
            print 'The vehicle is already armed!'
        else:
            tick = 0
            while not vehicle.is_armable:
                if tick == 3:
                    print "3 Secs passed and the drone is not armable!"
                    return
                print " Waiting for vehicle to initialise..."
                time.sleep(1)
                tick+=1
            self.autopilot.armed = True
            self.is_armed = True
            #send mavlink packet

    def disarm(self):
        if not self.is_armed:
            print 'The vehicle is already disarmed!'
        else:
            self.is_armed = False
            #send mavlink packet

if __name__== "__main__":
    autopilot_vehicle = autopilot()
    autopilot_vehicle.connect('127.0.0.1',14559)
    autopilot_vehicle.change_flight_mode('guided')
    autopilot_vehicle.takeoff(3)
