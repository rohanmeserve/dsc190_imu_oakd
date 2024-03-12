import time
import math
import logging

class PositionEstimator:

    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.vx = 0.
        self.vy = 0.
        self.yaw = 0.

        self.last_pos_x = 0.
        self.last_pos_y = 0.

        self.last_time = time.time()
        self.last_gps_time = time.time()

        with open('estimations.csv', 'w') as file:
            file.write('(est_x, est_y), (act_x, act_y)\n')

    def run(self, acl_x: float, acl_y: float, gyr_x: float, pos_x: float, pos_y: float):

        curr_time = time.time()
        dt = curr_time - self.last_time

        # update orientation
        self.yaw += float(gyr_x) * dt

        # rotate accerlation vectors
        ax = float(acl_x) * math.cos(-self.yaw) - float(acl_y) * math.sin(-self.yaw)
        ay = float(acl_x) * math.sin(-self.yaw) - float(acl_y) * math.cos(-self.yaw)

        # update velocity
        self.vx += ax * dt
        self.vy += ay * dt

        # update position
        self.x += self.vx * dt
        self.y += self.vy * dt


        self.last_time = curr_time

        # if the gps has been updated, reset position and velocity to its "true" values
        if pos_x != self.last_pos_x or pos_y != self.last_pos_y:
            print('resetting with gps instruciton')
            # calculate time difference since last gps update
            gps_dt = curr_time - self.last_gps_time
            # reset position based on gps values
            pos_x = float(pos_x)
            pos_y = float(pos_y)
            self.x = pos_x
            self.y = pos_y
            self.z = 0
            # reset velocity based on gps delta
            self.vx = (self.last_pos_x - pos_x) / gps_dt
            self.vy = (self.last_pos_y - pos_y) / gps_dt
            self.vz = 0
            # reset yaw
            a_mag = math.sqrt((self.last_pos_x - pos_x)**2 + (self.last_pos_y - pos_y)**2)
            b_mag = 1
            self.yaw = math.acos((self.last_pos_y - pos_y) / (a_mag * b_mag))
            # update last gps time to current
            self.last_gps_time = curr_time

        self.last_pos_x = pos_x
        self.last_pos_y = pos_y

        # log estimation
        logging.info(f"Estimated: ({self.x},{self.y},{self.yaw})")

        # write predicted pos and last known true pos to file for later analysis
        with open('estimations.csv', 'a') as file:
            file.write(f'({self.x},{self.y}), ({pos_x}, {pos_y})\n')
        return self.x, self.y, self.yaw

if __name__ == "__main__":
    print('starting main')
    from imu_oakd import IMU
    p = IMU(sensor="oakd_BNO086")
    print('imu set')
    est = PositionEstimator()
    print('estimator set')
    while True:
        imu_data = p.run()
        data = est.run(imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5], 0, 0)
        print(data)
        time.sleep(0.05)
