# - coding: utf-8 -
# bridgingIT
# CoE Drones and Robotics

"""
Program that  connects  to  4 crazyflies and flies them in a  ”pyramid” formation starting locations  as well as circle
positions and heights are setup in the params values.
This is designed to work with the Loco Positiong System running in TDoA mode.
"""

"""
Parameters definition...
"""

import time
import csv
import datetime
import numpy as np

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger

# Change uris according to your setup
#URI0 = 'radio://0/80/2M/E7E7E7E7E7'
URI1 = 'radio://0/80/2M/E7E7E7E7E7'
URI2 = 'radio://0/90/2M/E7E7E7E7E7'
#URI3 = 'radio://0/100/2M/E7E7E7E7E7'
#URI4 = 'radio://0/110/2M/E7E7E7E7E7'
#URI5 = 'radio://0/120/2M/E7E7E7E705'
#URI6 = 'radio://0/120/2M/E7E7E7E706'
#URI7 = 'radio://0/120/2M/E7E7E7E707'
#URI8 = 'radio://0/120/2M/E7E7E7E708'
#URI9 = 'radio://0/120/2M/E7E7E7E709'

z0 = 0.2
z = 1.0

x0 = 1.25
x1 = 2.0

y0 = 1.25
y1 = 2.0

t = 20

# r : radius , z : height , x : starting x , y : starting y , offset: degree offset for circle

params1 = {'r': 0.0, 'z': 0.8, 'x_s': 1.25, 'y_s': 1.25, 'x_l': 1.25, 'y_l': 1.25, 'offset': 0}
params2 = {'r': 0.7, 'z': 0.5, 'x_s': 1.75, 'y_s': 0.25, 'x_l': 0.25, 'y_l': 0.25, 'offset': 0}
#params3 = {'r': 0.7, 'z': 1.0, 'x_s': 1.0, 'y_s': 0.25, 'x_l': 1.0, 'y_l': 0.25, 'offset': 180}
#params4 = {'r': 0.7, 'z': 1.0, 'x_s': 0.25, 'y_s': 0.25, 'x_l': 1.75, 'y_l': 0.25, 'offset': 90}

params = {
    URI1: [params1],
    URI2: [params2],
    #URI3: [params3],
    #URI4: [params4],
}

# List of URIs. Comment out the ones you do not want to fly
uris = {
    URI1,
    URI2,
    # URI3,
    # URI4,
    # URI5,
    # URI6,
    # URI7,
    # URI8,
    # URI9,
    # URI10
}

# Path to FDR (flight data recorder)
flight_data_path = 'C://Users//akaviani//Documents//GitHub//crazyflie-repo//examples//bIT//'


"""
Functions definition...
"""

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for ', scf.cf.link_uri)

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

def take_off(cf, position):
    take_off_time = 0.8
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    print(vz)

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)

        time.sleep(sleep_time)

def land(cf, position):
    landing_time = 1.5
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position[2] / landing_time

    print(vz)

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

    cf.commander.send_setpoint(0, 0, 0, 0)

    time.sleep(0.1)

def circle_next_pos(t, r, offset):
    # t: time value, r: radius of circle, offset: position in circle in degrees
    x_c = 1.25  # Center x value
    y_c = 1.25  # Center y value
    x_pos = x_c + (r * np.cos(t/10 + offset))
    y_pos = y_c + (r * np.cos(t/10 + offset))
    # yaw = 90 + ( 5.45 * t)
    desired_pos = (x_pos, y_pos, 1.0, 0.0)

    return desired_pos

def run_sequence(scf, params):

# params: dict that holds parameter values
    try:
        cf = scf.cf
        cf.param.set_value('flightmode.posSet', '1')
        r = params['r']
        z = params['z']
        x_s = params['x_s']
        y_s = params['y_s']
        x_l = params['x_l']
        y_l = params['y_l']
        offset = params['offset']

# Takeoff sequence
        take_off(cf, (x_s, y_s, 0.5, 0)) # (x, y, z, yaw)

# Circle sequence
        for i in range(100):    # Run the circle position updates
            position = circle_next_pos(t, r, offset)
            cf.commander.send_setpoint(position[1], position[0],
                                       position[3],
                                       int(z * 1000))

            time.sleep(0.25)

# Move to landing position
        for i in range(30):
            cf.commander.send_setpoint(y_l, x_l,
                                       0,
                                       int(z * 1000))

            time.sleep(0.2)
        land(cf, (x_l, y_l, z, 0))

    except Exception as e:
        print(e)

def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))

# Write position data to .csv
# Must be updated to match the specific filesystem
    with open(flight_data_path + datetime.datetime.now().strftime('%Y-%m-%d-%H_')+'_flight_data.csv','a') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow([x, y, z, timestamp])
    csvfile.close()

def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=50)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


"""
Init the code
"""

if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:

# If drones are started in their correct positions this is probably not needed.
# The Kalman filter will have time to converge any way since it takes a while to start them all up and connect.
        swarm.parallel(reset_estimator)
        time.sleep(2.0)

# The current values of all parameters are downloaded as a part of the connection.
# Since we have 4 drones this is clogging up communication and we have to wait for it to finish before we start flying.
        print('Waiting for parameters to be downloaded...')
        swarm.parallel(wait_for_param_download)
        swarm.parallel(start_position_printing) # Activate the position printing to screen and to csv
        time.sleep(3.0)
        swarm.parallel(run_sequence, args_dict=params)  # Run the sequence and input parameters