import rospy

class ControllerRecord(object):
    def __init__(self):
        self._initialFlag = False
        self._pos_pre = None # coord (x, y)
        self._theta_pre = None
        self._time_pre = None
        self._time_start = None

        self._distanceTravelled = 0
        self._angleTurned = 0
        self._timePassed = 0

    def updateRecord(self, pos, theta):
        try:
            self._distanceTravelled += pow(pow(pos[0] - self._pos_pre[0], 2) + pow(pos[1] - self._pos_pre[1], 2), 0.5)
            self._angleTurned += abs(theta - self._theta_pre)
            cur = rospy.get_time()
            self._timePassed += cur - self._time_pre

            self._pos_pre = pos
            self._theta_pre = theta
            self._time_pre = cur

        except TypeError as e:
            if not self._initialFlag:
                self._pos_pre = pos
                self._theta_pre = theta
                self._time_pre = self._time_start = rospy.get_time()
                self._initialFlag = True
                print 'FIRST SETUP DONE, CURRENT STATUS IS {} {} {}'.format(self._pos_pre, self._theta_pre, self._time_pre) # debug del
            else:
                raise e

    # though it uses print() at the moment, remember to replace to rospy.log() for worth storing info !!!
    def printRecord(self):
        print 'CURRENT STATUS IS {} {} {}'.format(self._pos_pre, self._theta_pre, self._time_pre) # debug del
        print "Total Distance: {}, Total Angle Turned: {}, Runtime: {}".format(self._distanceTravelled, self._angleTurned, self._timePassed)
        print ''
