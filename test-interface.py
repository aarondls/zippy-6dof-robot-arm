import numpy as np
import modern_robotics as mr

# This will be used to generate desired positions, which is then copied over to the Teensy script.
# The microcontroller handles all trajectory generations; this script only handles calculations of
# inverse kinematics since the MCU cannot handle linear algebra computations nicely.

# This is a temporary method for now. Over the summer, I will get to learning Moveit.


#### Helper functions

def CalculateHomeAndScrewAxes():
    global M
    global S

    # units in mm
    M = np.array([  [ 1,  0,  0,  530.5],
                    [ 0,  1,  0,  -39.7],
                    [ 0,  0,  1,  98.5],
                    [ 0,  0,  0,  1]])
    w1 = np.array([0,0,1])
    w2 = np.array([0,1,0])
    w3 = np.array([0,1,0])
    w4 = np.array([0,1,0])
    w5 = np.array([1,0,0])
    w6 = np.array([0,0,1])
    v1 = np.cross(-w1, np.array([0, 0, 0]))
    v2 = np.cross(-w2, np.array([21.5, 0, 141.5]))
    v3 = np.cross(-w3, np.array([251.5, 0, 141.5]))
    v4 = np.cross(-w4, np.array([450.5, 0, 141.5]))
    v5 = np.cross(-w5, np.array([0, -39.7, 141.5]))
    v6 = np.cross(-w6, np.array([530.5, -39.7, 0]))

    S1 = np.concatenate((w1, v1), axis=None)
    S2 = np.concatenate((w2, v2), axis=None)
    S3 = np.concatenate((w3, v3), axis=None)
    S4 = np.concatenate((w4, v4), axis=None)
    S5 = np.concatenate((w5, v5), axis=None)
    S6 = np.concatenate((w6, v6), axis=None)

    S = np.array([S1, S2, S3, S4, S5, S6]).T

    print(M)
    print(S)

    return M, S


# Get home and screw axes
M, S = CalculateHomeAndScrewAxes()


# calculate joint configuration achieving specific end effector position
# take a guess
initial_theta_guess = np.array([0,-9.55545795e-01,2.18960437e+00,-1.23406017e+00,0,0])
# in units of mm
T_s_desired = np.array([[1, 0, 0, 300],
                        [0, 1, 0, -39.7],
                        [0, 0, 1, 60],
                        [0, 0, 0, 1]])
thetalist, success = mr.IKinSpace(S, M, T_s_desired, initial_theta_guess, 0.01, 0.001)
print(success)
print(np.array2string(thetalist, separator='],['))
print(np.degrees(thetalist))

# point 1
# [0 -43.3937198  124.323738 -80.9300183  0  0]

# take a guess
initial_theta_guess = np.array([0,-1.15396739e+00,2.16668737e+00,-1.01271999e+00,0,0])
# in units of mm
T_s_desired = np.array([[1, 0, 0, 300],
                        [0, 1, 0, -39.7],
                        [0, 0, 1, 170],
                        [0, 0, 0, 1]])
thetalist, success = mr.IKinSpace(S, M, T_s_desired, initial_theta_guess, 0.01, 0.001)
print(success)
print(np.array2string(thetalist, separator='],['))
print(np.degrees(thetalist))

# point 2
# in degrees: [0 -73.2472242  121.615309 -48.3680854 0 0]

# take a guess
initial_theta_guess = np.array([0,-1.27840523e+00,2.12258756e+00,-8.44182343e-01,0,0])
# in units of mm
T_s_desired = np.array([[1, 0, 0, 200],
                        [0, 1, 0, 200],
                        [0, 0, 1, 170],
                        [0, 0, 0, 1]])
thetalist, success = mr.IKinSpace(S, M, T_s_desired, initial_theta_guess, 0.01, 0.001)
print(success)
print(np.array2string(thetalist, separator='],['))
print(np.degrees(thetalist))

# point 3
# in degrees: [ 53.0687190 -77.1839579  127.330756 -50.1467985 0 -53.0687190]


# take a guess
initial_theta_guess = np.array([0,-1.15396739e+00,2.16668737e+00,-1.01271999e+00,0,0])
# in units of mm
T_s_desired = np.array([[1, 0, 0, 200],
                        [0, 1, 0, -250],
                        [0, 0, 1, 170],
                        [0, 0, 0, 1]])
thetalist, success = mr.IKinSpace(S, M, T_s_desired, initial_theta_guess, 0.01, 0.001)
print(success)
print(np.array2string(thetalist, separator='],['))
print(np.degrees(thetalist))

# point 4
# in degrees: [-44.2170829 -69.8404933  116.365655 -46.5251624 0  44.2170829]

# take a guess
initial_theta_guess = np.array([-7.71733682e-01,-1.21894656e+00,2.03096382e+00,-8.12017269e-01,7.71731971e-09,7.71733682e-01])
# in units of mm
T_s_desired = np.array([[1, 0, 0, 200],
                        [0, 1, 0, -250],
                        [0, 0, 1, 60],
                        [0, 0, 0, 1]])
thetalist, success = mr.IKinSpace(S, M, T_s_desired, initial_theta_guess, 0.01, 0.001)
print(success)
print(np.array2string(thetalist, separator='],['))
print(np.degrees(thetalist))

# point 5
# in degrees: [-44.2170676 -42.3832982  118.930764 -76.5474666 0  44.2170676e+01]