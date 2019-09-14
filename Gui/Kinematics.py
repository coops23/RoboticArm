import math

L1 = 120
L2 = 240

def ForwardKin(q0, q1, q2, q3, q4):
    a1 = 120
    a2 = 240

    q0 = math.radians(q0)
    q1 = math.radians(q1)
    q2 = math.radians(q2)
    q3 = math.radians(q3)
    q4 = math.radians(q4)

    r = ((L2 * math.cos(q1 + q2)) + (L1 * math.cos(q1))) * -1
    z = (L2 * math.sin(q1 + q2)) + (L1 * math.sin(q1))
    x = math.cos(q0) * r
    y = math.sin(q0) * r

    return [x, y, z]

def InverseKinCartesian(x, y, z, Q0_MIN, Q0_MAX, Q1_MIN, Q1_MAX, Q2_MIN, Q2_MAX):
    try:
        q1 = None
        q2 = None

        q0 = int(math.degrees(math.atan2(y, x)))
        r = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
        z = z

        if (q0 < Q0_MIN or q0 > Q0_MAX):
            raise ValueError

        tau = math.atan2(z, r)
        beta = math.acos((math.pow(L1, 2) + math.pow(L2, 2) - math.pow(r, 2) - math.pow(z, 2)) / (2 * L1 * L2))
        alpha = math.acos((math.pow(L1, 2) - math.pow(L2, 2) + math.pow(r, 2) + math.pow(z, 2)) / (
                    2 * L1 * math.sqrt(math.pow(r, 2) + math.pow(z, 2))))

        righty_q1 = int(math.degrees(tau - alpha))
        righty_q2 = int(math.degrees(math.pi - beta))
        lefty_q1 = int(math.degrees(tau + alpha))
        lefty_q2 = int(math.degrees(beta - math.pi))

        if (righty_q1 > Q1_MIN and righty_q1 < Q1_MAX and righty_q2 > Q2_MIN and righty_q2 < Q2_MAX):
            q1 = righty_q1
            q2 = -1 * righty_q2
        elif (lefty_q1 > Q1_MIN and lefty_q1 < Q1_MAX and lefty_q2 > Q2_MIN and lefty_q2 < Q2_MAX):
            q1 = lefty_q1
            q2 = -1 * lefty_q2
        else:
            raise ValueError

    except ZeroDivisionError:
        q1 = None
        q2 = None

    except ValueError:
        q1 = None
        q2 = None

    finally:
        return [q0, q1, q2]

def InverseKinPolar(r, z, q0, R_MIN, R_MAX, Z_MIN, Z_MAX, Q0_MIN, Q0_MAX, Q1_MIN, Q1_MAX, Q2_MIN, Q2_MAX):
    try:
        q1 = None
        q2 = None

        if (q0 < Q0_MIN or q0 > Q0_MAX):
            raise ValueError
        if (r < R_MIN or r > R_MAX):
            raise ValueError
        if (z < Z_MIN or z > Z_MAX):
            raise ValueError

        tau = math.atan2(z, r)
        beta = math.acos((math.pow(L1, 2) + math.pow(L2, 2) - math.pow(r, 2) - math.pow(z, 2)) / (2 * L1 * L2))
        alpha = math.acos((math.pow(L1, 2) - math.pow(L2, 2) + math.pow(r, 2) + math.pow(z, 2)) / (
                    2 * L1 * math.sqrt(math.pow(r, 2) + math.pow(z, 2))))

        righty_q1 = int(math.degrees(tau - alpha))
        righty_q2 = int(math.degrees(math.pi - beta))
        lefty_q1 = int(math.degrees(tau + alpha))
        lefty_q2 = int(math.degrees(beta - math.pi))

        if (righty_q1 > Q1_MIN and righty_q1 < Q1_MAX and righty_q2 > Q2_MIN and righty_q2 < Q2_MAX):
            q1 = righty_q1
            q2 = -1 * righty_q2
        elif (lefty_q1 > Q1_MIN and lefty_q1 < Q1_MAX and lefty_q2 > Q2_MIN and lefty_q2 < Q2_MAX):
            q1 = lefty_q1
            q2 = -1 * lefty_q2
        else:
            raise ValueError

    except ZeroDivisionError:
        q1 = None
        q2 = None

    except ValueError:
        q1 = None
        q2 = None

    finally:
        return [q1, q2]