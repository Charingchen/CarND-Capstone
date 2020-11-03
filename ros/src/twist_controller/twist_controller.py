import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
PRINT_FREQ = 2 # print log every how many sec


class Controller(object):
    def __init__(self,vehicle_mass,fuel_capacity,brake_deadband,decel_limit,
                accel_limit, wheel_radius, wheel_base,steer_ratio,max_lat_accel,
                max_steer_angle):
        # TODO:
        self.yaw_controller = YawController(wheel_base,steer_ratio,0.1,max_lat_accel,max_steer_angle)

        kp = 0.5
        ki = 0.001
        kd = .8 # Derivate is not required.
        mn = 0. # Minimum throttle value
        mx = 0.2 # maximum throttle value
        self.throttle_controller = PID(kp,ki,kd,mn,mx)

        tau = 0.5 # 1/(2pi*tau) = cut off freq
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau,ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = self.print_time = rospy.get_time()
         

    def control(self, current_val,dbw_enable,linear_vel, angular_vel):
        # TODO

        # Disable and reset pid if the enable is false
        if not dbw_enable:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        # Filter the high frequncy portion off
        current_val = self.vel_lpf.filt(current_val)

        # rospy.logwarn('Angular vel: {0}'.format(angular_vel))
        # rospy.logwarn('Target vel: {0}'.format(linear_vel))
        # rospy.logwarn('Current vel: {0}'.format(current_val))

        steering = self.yaw_controller.get_steering(linear_vel,angular_vel,current_val)

        # Calculate the error for the PID controller
        vel_err = linear_vel - current_val
        self.last_vel = current_val

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_err, sample_time)
        brake = 0

        # Apple brake its almost at stop
        if linear_vel == 0 and current_val < 0.1:
            throttle = 0. 
            brake = 400 #N*m to hold the car in place if we are stopped

        # If throttle come out be small, apply calculated brake torque or when vel_err is too big, slow down
        elif (throttle < 0.1 and vel_err < 0) or vel_err < -2.0 :
            throttle = 0.
            decel = max(vel_err,self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque per meter

        # Debug Print out
        # if (current_time - self.print_time) > PRINT_FREQ:
        #     self.print_time = current_time
        #     rospy.logwarn("POSE: current_vel={:.2f}, linear_vel={:.2f}, vel_error={:.2f}".format(current_val,linear_vel,vel_err))
        #     rospy.logwarn("POSE: throttle={:.2f}, brake={:.2f}, steering={:.2f}".format(throttle, brake, steering))


        
        # Return throttle, brake, steer
        return throttle, brake, steering
