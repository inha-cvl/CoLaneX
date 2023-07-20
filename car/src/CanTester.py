from OdometryReader import OdometryReader
from Params import i30, ioniq

if __name__ == '__main__':

    odometry_reader = OdometryReader(ioniq['dbc_file_path'],
                                     ioniq['arbitration_id'],
                                     ioniq['wheel_vel_rr'],
                                     ioniq['wheel_vel_rl'])

    while(1):
        odometry_reader.read_odom()
        print(odometry_reader.get_vel())
        