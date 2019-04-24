import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'


# Change the sequence according to your setup
#             x    y    z  YAW
sequence = [
    (1, 1, 1, 0),
    (1.5, 1, 1, 0),
    (1, 1, 1, 0),
    (1.5, 1, 1, 0),
    (1, 2, 1, 0),
    (1, 1, 0.8, 0),
    (1, 1, 0.4, 0),
]


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

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')

def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')

def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))

def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

def run_sequence():
    commander = cf.high_level_commander

    commander.takeoff(1.0, 2.0)

    time.sleep(3.0)

    relative = False

    commander.go_to(1, 1, 1, 0, 1.0, relative)

#    commander.start_trajectory(trajectory_id, 1.0, relative)
#    time.sleep(duration)

    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

'''if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        commander = cf.high_level_commander
        #activate_high_level_commander(cf)
        #activate_mellinger_controller(cf)
        reset_estimator(scf)
        start_position_printing(scf)
        #run_sequence(scf, sequence)

        commander.takeoff(1.0, 1.0)
        #time.sleep(3.0)
        relative = False
        print("ich bin airborne!!")
        #for i in sequence:
        commander.go_to(2, 2, 0.4, 0, 2, relative)
        #commander.stop()


        #    commander.start_trajectory(trajectory_id, 1.0, relative)
        #    time.sleep(duration)

        commander.land(0.2, 1.0)
        time.sleep(1)
        cf.commander.send_stop_setpoint()
        #commander.stop()
        
'''