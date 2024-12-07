import roboticstoolbox as rtb

def position_quintic_interpolation(initialPoint, finalPoint, numberOfSteps):

    # https://petercorke.github.io/robotics-toolbox-python/arm_trajectory.html#roboticstoolbox.tools.trajectory.mtraj
    quinticInterpolation = rtb.tools.trajectory.mtraj(rtb.tools.quintic, initialPoint, finalPoint, numberOfSteps)

    return quinticInterpolation.q





if __name__ == '__main__':

    # p = [x, y, z, pitchAngle]
    p0 = [20.47, 0, 27.71, -74.0]
    p1 = [23.4, -5.4, 4.0, 0]

    # Number of steps
    n = 70

    interp = position_quintic_interpolation(p0, p1, n)

    print(interp)