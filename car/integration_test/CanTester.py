import OdometryReader
from Params import i30, ioniq

if __name__ == '__main__':

    param = ioniq

    odometry_reader = OdometryReader(param['dbc_file_path'],
                                     param['arbitration_id'],
                                     param['wheel_vel_rr'],
                                     param['wheel_vel_rl'])

    while(1):
        odometry_reader.read_odom()
        print(odometry_reader.get_vel())
        