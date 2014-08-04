#! /usr/bin/env python3


import sys, time, math
import threading
import socket, struct
import numpy as np
from collections import deque

# ============================================================================ #

# Constants
CYCLE_LENGTH = 0.02 # cycle length in seconds

# ============================================================================ #

class PNS(object):
    """Peripheral nervous system
    Creates socket connections to the simulation server.
    Sends effector messages.
    Receives perceptor messages.
    Upon creation the agent is registered with the server.
    """
    def __init__(self, agentID, teamname, host='localhost', port=3100,
            model='rsg/agent/nao/nao.rsg', debugLevel=10):

        self.agentID    = agentID
        self.teamname   = teamname
        self.host       = host
        self.port       = port
        self.model      = model
        self.debugLevel = debugLevel

        # create socket and connect to simulation server
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))

        # create and initialize agent
        self._send_effector('(scene {})'.format(self.model))
        self.receive_perceptors()
        self._send_effector('(init (unum {})(teamname {}))'.format(self.agentID, self.teamname))
        self.receive_perceptors() 

# ==================================== #

    def _send_effector(self, message):
        """Each message is prefixed with the length of the payload message.
        The length prefix is a 32 bit unsigned integer in network order"""

        # report message
        if (self.debugLevel >= 10):
            print("S:", message)

        # convert message to ASCII encoded byte string
        length   = len(message)
        bmessage = bytes(message, 'ASCII')

        # send length of message
        lengthMessage = struct.pack("!I", length)
        bytesSent = 0
        while (bytesSent < 4):
            bytesSent += self.socket.send(lengthMessage[bytesSent:])

        # send actual message
        bytesSent = 0
        while (bytesSent < length):
            bytesSent += self.socket.send(bmessage[bytesSent:])

# ==================================== #

    def receive_perceptors(self):

        # receive length of the message (4 bytes)
        length = self._receive_message(4)
        length = struct.unpack("!I", length)[0]

        # receive actual message
        perceptors = self._receive_message(length)
        perceptors = str(perceptors, 'ASCII')

        return self._parse_perceptors(perceptors)
    
# ==================================== #

    def _receive_message(self, length):
        """Receive a message from server communication socket of given length"""

        message = b''
        while (len(message) < length):
            nextBytes = self.socket.recv(length - len(message))
            if (nextBytes == ''):
                raise OSError('Socket to simulation server was closed')
            else:
                message += nextBytes
 
        return message

# ==================================== #

    def _parse_perceptors(self, perceptors):
        """Minimal parsing of perceptor message.
        Convert message to nested python lists, substituting '[' and ']'
        for '(' and ')'"""

        return self.__str2list(perceptors)

# ==================================== #

    def __str2list(self, string):
        """Convert a string to a (nested) python list, substituting '[' and ']'
        for '(' and ')'"""
        
        l     = []
        bra   = '('
        ket   = ')'
        space = ' '
        nbra  = 0 # number of '('
        nket  = 0 # number of ')'

        nonword = [space, bra, ket]

        # begin and end indices of new sublist
        begin = 0
        end   = 0

        prevc = ''
        for i, c in enumerate(string):

            # detect word beginnings
            if (i == 0 and c != bra):
                begin = i
            elif (nbra == 0 and c not in nonword and (prevc == bra or prevc == space)):
                begin = i

            # detect beginning of new nested list
            elif (c == bra and nbra == nket):
                begin = i+1
            elif (c == ket and nbra == nket+1):
                end = i
                l.append(self.__str2list(string[begin:end]))

            # detect word endings
            if (i == len(string)-1 and c != ket):
                word = string[begin:]
                try: word = int(word)
                except:
                    try: word = float(word)
                    except: pass
                l.append(word)
            if (c == space and nbra == 0 and prevc not in nonword):
                end = i
                word = string[begin:end]
                try: word = int(word)
                except:
                    try: word = float(word)
                    except: pass 
                l.append(word)
 

            if (c == bra): nbra += 1
            if (c == ket): nket += 1

            prevc = c
                
        # return
        if len(l) == 1:
            return l[0]
        else:
            return l

# ==================================== #

    def hinge_joint_effector(self, name, rate):
        """Set the change rate in degree/cycle of the
        hinge joint with the provided name"""
        message = "({} {:.2f})".format(name, rate)
        self._send_effector(message)

# ==================================== #

    def universal_joint_effector(self, name, rate1, rate2):
        """Set the change rate in degree/cycle of axis 1 and 2 of the
        hinge joint with the provided name"""
        message = "({} {:.2f} {:.2f})".format(name, rate1, rate2)
        self._send_effector(message)

# ==================================== #

    def beam_effector(self, x, y, rotation):
        """Position the player on the field before the game starts
        and after a goal was scored.
        Position and orientation of the team playing from right to left is
        point reflected at the center point.
        x, y        Coordinates
        rotation    horizontal orientation with respect to x-axis in degree"""
        message = "(beam {:.2f} {:.2f} {:.2f})".format(x, y, rotation)
        self._send_effector(message)

# ==================================== #

    def say_effector(self, message):
        """Broadcast a message to other agents.
        At most 20 ASCII characters are allowed
        white space and normal brackets are prohibited."""
        if len(message) > 20:
            message = message[:20]
        for c in message:
            if (c == ' ' or c == '(' or c == ')'):
                print("Character not allowed for say effector: '{}'".format(c))
                print("Nothing sent.")
                return
        message = "(say {})".format(message)
        self._send_effector(message)
 

# ============================================================================ #


class MovementScheduler(deque):
    """A queue for scheduling robot movements.
    It guarantees that each function is scheduled only once at a time
    to prevent conflicts with the potential of deadlocking the bot"""

    def append(self, newitem):
        """The first element in the newitem list must be a function that gets called
        repeatedly until it returns 'done'. The second element is a dictionary
        that contains the keyword arguments passed to the function. The third
        item is optional and should contain a list (as the  simplest mutable datatype)
        whose zeroth element is set to true once the function contains 'done'
        to signal completion."""
        # check proper format of item first
        if not type(newitem) == list:
            raise QueueItemError("MovementQueue items must be lists.")
        elif not (len(newitem) == 2 or len(newitem) == 3):
            raise QueueItemError("MovementQueue items must be of format: [<function>, <kwargs dict>, <list>].")
        elif len(newitem) == 3 and type(newitem[2]) != list:
            raise QueueItemError("Third (optional) item to MovementQueue entries must be a list.")
        elif not callable(newitem[0]):
            raise QueueItemError("First item in list must be a function.")
        elif not type(newitem[1]) == dict:
            raise QueueItemError("MovementQueue items must be of format: [<function>, <kwargs dict>, <list>].")

        for item in self:
            function      = item[0]
            dictionary    = item[1]
            newFunction   = newitem[0]
            newDictionary = newitem[1]
            if function == newFunction:
                try:
                    if dictionary['hj'] == newDictionary['hj']:
                        # two functions operating on the same hj not allowed
                        raise SchedulerConflict('The function "{}" is already in the queue.'.format(item[0]))
                except KeyError:
                    pass

        super().append(newitem) 

# ==================================== #

    def run(self):
        """Execute all functions currently in msched
        Reschedule if they are not done"""

        for i in range(len(self)):
            item     = self.popleft()
            function = item[0]
            argDict  = item[1]

            returnVal = function(**argDict)

            if returnVal != "done":
                self.append(item)
            elif len(item) == 3:
                item[2][0] = True


# ============================================================================ #


class GameState(object):
    """Store game state information"""

    def __init__(self, time=0.0, gametime=0.0, scoreLeft=0, scoreRight=0,
            playmode='BeforeKickOff'):
        self.time       = time
        self.gametime   = gametime
        self.scoreLeft  = scoreLeft
        self.scoreRight = scoreRight
        self.playmode   = playmode

# ==================================== #

    def set_time(self, time):
        self.time       = time
    def set_gametime(self, gametime):
        self.gametime   = gametime
    def set_scoreLeft(self, scoreLeft):
        self.scoreLeft  = scoreLeft
    def set_scoreRight(self, scoreRight):
        self.scoreRight = scoreRight
    def set_playmode(self, playmode):
        self.playmode   = playmode 

    def get_time(self):
        return self.time
    def get_gametime(self):
        return self.gametime
    def get_scoreLeft(self):
        return self.scoreLeft
    def get_scoreRight(self):
        return self.scoreRight
    def get_playmode(self):
        return self.playmode

# ==================================== #
    
    def __str__(self):
        string = ""
        string += "time       = {}\n".format(self.time      )
        string += "gametime   = {}\n".format(self.gametime  )
        string += "scoreLeft  = {}\n".format(self.scoreLeft )
        string += "scoreRight = {}\n".format(self.scoreRight)
        string += "playmode   = {}"  .format(self.playmode  )
        return string


# ============================================================================ #


class Gyroscope(object):
    """Gyroscope perceptor holding information about the change in
    orientation of a body with respect to the global coordinate system
    The rate of change is measured in deg/s"""

    def __init__(self, name):
        self.name = name
        self.rate = np.zeros(3, dtype=np.float)

        # unit vectors of body with respect to global coordinate system
        self.x = np.array([1.0, 0.0, 0.0])
        self.y = np.array([0.0, 1.0, 0.0])
        self.z = np.array([0.0, 0.0, 1.0])

    def set(self, rate):
        for i in range(3):
            self.rate[i] = rate[i]

        # rotation in degree during the last cycle
        rotationAngle = np.linalg.norm(self.rate) / (1.0/CYCLE_LENGTH) 

        # rotate local coordinate frame
        self.x = rotate_arbitrary(self.rate, self.x, angle=rotationAngle)
        self.y = rotate_arbitrary(self.rate, self.y, angle=rotationAngle)
        self.z = rotate_arbitrary(self.rate, self.z, angle=rotationAngle)

    def get_rate(self):
        return self.rate

    def get_orientation(self):
        return self.x, self.y, self.z
    

# ============================================================================ #


class Accelerometer(object):
    """Accelerometer to measure the acceleration relative to free fall
    Will therefore indicate 1g = 9.81m/s at rest in positive z direction"""

    def __init__(self, name):
        self.name = name
        self.acceleration = np.zeros(3, dtype=np.float)

    def set(self, acceleration):
        for i in range(3):
            self.acceleration[i] = acceleration[i]
        self.acceleration[i] -= 9.81

    def get(self):
        return self.acceleration


# ============================================================================ #


class ForceResistanceSensor(object):
    """Sensor state of a Force resistance perceptor
    point is the point of origin of the force
    force is the force vector"""

    def __init__(self, name):
        self.name  = name
        self.point = np.zeros(3, dtype=np.float)
        self.force = np.zeros(3, dtype=np.float)

    def set(self, point, force):
        """Set the point of origin and the force
        Any 3 dimensional object that holds data convertible to float is valid"""
        for i in range(3):
            self.point[i] = point[i]
            self.force[i] = force[i]

    def get_point(self):
        """get the point of origin coordinates"""
        return self.point

    def get_force(self):
        """get the force vector"""
        return self.force

# ============================================================================ #


class NaoRobot(object):
    """Class that represents the Nao Soccer Robot"""

    def __init__(self, agentID, teamname, host='localhost', port=3100, debugLevel=0,
            startCoordinates=[-0.5, 0, 0]): 

        self.agentID    = agentID
        self.teamname   = teamname
        self.host       = host
        self.port       = port
        self.debugLevel = debugLevel
        self.alive      = False

        # set maximum hinge effector speed
        self.maxhjSpeed = 7.035

        # movement schedule
        # each sublist should contain a function object and
        # a dictionary of keyword arguments
        # e.g. [foo, {'kw1': val1, 'kw2', val2}]
        # the function will be executed until it returns "done"
        self.msched     = MovementScheduler()

        # games state information
        self.gamestate  = GameState()

        # gyroscope and accelerometer
        self.gyr        = Gyroscope    ('torso')
        self.acc        = Accelerometer('torso')

        # hinge joint perceptor states
        self.hj         = {'hj1' : 0.0,
                           'hj2' : 0.0,
                           'raj1': 0.0,
                           'raj2': 0.0,
                           'raj3': 0.0,
                           'raj4': 0.0,
                           'laj1': 0.0,
                           'laj2': 0.0,
                           'laj3': 0.0,
                           'laj4': 0.0,
                           'rlj1': 0.0,
                           'rlj2': 0.0,
                           'rlj3': 0.0,
                           'rlj4': 0.0,
                           'rlj5': 0.0,
                           'rlj6': 0.0,
                           'llj1': 0.0,
                           'llj2': 0.0,
                           'llj3': 0.0,
                           'llj4': 0.0,
                           'llj5': 0.0,
                           'llj6': 0.0,}

        # corresponding hinge joint effectors
        self.hjEffector = {'hj1' : 'he1',
                           'hj2' : 'he2',
                           'raj1': 'rae1',
                           'raj2': 'rae2',
                           'raj3': 'rae3',
                           'raj4': 'rae4',
                           'laj1': 'lae1',
                           'laj2': 'lae2',
                           'laj3': 'lae3',
                           'laj4': 'lae4',
                           'rlj1': 'rle1',
                           'rlj2': 'rle2',
                           'rlj3': 'rle3',
                           'rlj4': 'rle4',
                           'rlj5': 'rle5',
                           'rlj6': 'rle6',
                           'llj1': 'lle1',
                           'llj2': 'lle2',
                           'llj3': 'lle3',
                           'llj4': 'lle4',
                           'llj5': 'lle5',
                           'llj6': 'lle6',} 

        # force resistance perceptors
        self.frp        = {'rf': ForceResistanceSensor('rf'),
                           'lf': ForceResistanceSensor('lf')}

        # hinge joint effector states
        self.he         = {'he1' : 0.0,
                           'he2' : 0.0,
                           'rae1': 0.0,
                           'rae2': 0.0,
                           'rae3': 0.0,
                           'rae4': 0.0,
                           'lae1': 0.0,
                           'lae2': 0.0,
                           'lae3': 0.0,
                           'lae4': 0.0,
                           'rle1': 0.0,
                           'rle2': 0.0,
                           'rle3': 0.0,
                           'rle4': 0.0,
                           'rle5': 0.0,
                           'rle6': 0.0,
                           'lle1': 0.0,
                           'lle2': 0.0,
                           'lle3': 0.0,
                           'lle4': 0.0,
                           'lle5': 0.0,
                           'lle6': 0.0,} 


        # maxima of hinge joints
        self.hjMax      = {'hj1' :  120.0,
                           'hj2' :   45.0,
                           'raj1':  120.0,
                           'raj2':    1.0,
                           'raj3':  120.0,
                           'raj4':   90.0,
                           'laj1':  120.0,
                           'laj2':   95.0,
                           'laj3':  120.0,
                           'laj4':    1.0,
                           'rlj1':    1.0,
                           'rlj2':   25.0,
                           'rlj3':  100.0,
                           'rlj4':    1.0,
                           'rlj5':   75.0,
                           'rlj6':   45.0,
                           'llj1':    1.0,
                           'llj2':   45.0,
                           'llj3':  100.0,
                           'llj4':    1.0,
                           'llj5':   75.0,
                           'llj6':   25.0,} 

        # minima of hinge joints
        self.hjMin      = {'hj1' : -120.0,
                           'hj2' :  -45.0,
                           'raj1': -120.0,
                           'raj2':  -95.0,
                           'raj3': -120.0,
                           'raj4':   -1.0,
                           'laj1': -120.0,
                           'laj2':   -1.0,
                           'laj3': -120.0,
                           'laj4':  -90.0,
                           'rlj1':  -90.0,
                           'rlj2':  -45.0,
                           'rlj3':  -25.0,
                           'rlj4': -130.0,
                           'rlj5':  -45.0,
                           'rlj6':  -25.0,
                           'llj1':  -90.0,
                           'llj2':  -25.0,
                           'llj3':  -25.0,
                           'llj4': -130.0,
                           'llj5':  -45.0,
                           'llj6':  -45.0,}  

        # defaults (starting positions) of hinge joints in percent
        self.hjDefault  = {'hj1' : 0.0,
                           'hj2' : 0.0,
                           'raj1': 0.0,
                           'raj2': 0.0,
                           'raj3': 0.0,
                           'raj4': 0.0,
                           'laj1': 0.0,
                           'laj2': 0.0,
                           'laj3': 0.0,
                           'laj4': 0.0,
                           'rlj1': 0.0,
                           'rlj2': 0.0,
                           'rlj3': 0.0,
                           'rlj4': 0.0,
                           'rlj5': 0.0,
                           'rlj6': 0.0,
                           'llj1': 0.0,
                           'llj2': 0.0,
                           'llj3': 0.0,
                           'llj4': 0.0,
                           'llj5': 0.0,
                           'llj6': 0.0,}   
 
        # create peripheral nervous system (server communication)
        self.pns = PNS(self.agentID, self.teamname,
                host=self.host, port=self.port, debugLevel=self.debugLevel)

        self.perceive()
        self.pns.beam_effector(startCoordinates[0], startCoordinates[1], startCoordinates[2])

        # set default hing joint angles
        for hj in self.hjDefault.keys():
            self.hjDefault[hj] = self.get_hj(hj)

        self.lifeThread = threading.Thread(target=self.live)
        self.lifeThread.start()

        # put arms down
        self.msched.append([self.move_hj_to, {'hj': 'raj1', 'speed': 25, 'percent': 10}])
        self.msched.append([self.move_hj_to, {'hj': 'laj1', 'speed': 25, 'percent': 10}])


# ==================================== #

    def live(self):
        """Start the robot"""

        # only one live thread allowed!
        if self.alive:
            return

        self.alive = True

        iteration = -1
        while self.alive:
            iteration += 1

            self.perceive()

            self.msched.run()

# ==================================== #

    def die(self, timeout=0):
        """Stop robot execution and close socket connection to server
        If timeout is > 0, give the robot some time to finish scheduled movements."""
        start    = time.time()
        timeleft = timeout - time.time() + start
        while timeleft > 0:
            if len(self.msched) == 0:
                self.alive = False
                break
            timeleft = timeout - time.time() + start
        self.alive = False
        self.lifeThread.join()
        self.pns.socket.close()

# ==================================== #

    def perceive(self):
        """Receive perceptor information from server and
        update status accordingly"""

        perceptors = self.pns.receive_perceptors()

        for perceptor in perceptors:

            # time
            if perceptor[0] == 'time':
                self.gamestate.set_time(perceptor[1][1])

            # game state
            elif perceptor[0] == 'GS':
                for field in perceptor[1:]:
                    if field[0] == 'sl':
                        self.gamestate.set_scoreLeft(field[1])
                    elif field[0] == 'sr':
                        self.gamestate.set_scoreRight(field[1])
                    elif field[0] == 't':
                        self.gamestate.set_gametime(field[1])
                    elif field[0] == 'pm':
                        self.gamestate.set_playmode(field[1])

            # gyroscope
            elif perceptor[0] == 'GYR':
                self.gyr.set(perceptor[2][1:])

            # set accelerometer
            elif perceptor[0] == 'ACC':
                self.acc.set(perceptor[2][1:])

            # vision information
            elif perceptor[0] == 'See':
                pass 

            # hinge joints
            elif perceptor[0] == 'HJ':
                self.hj[perceptor[1][1]] = perceptor[2][1]

            # force resistance perceptors
            elif perceptor[0] == 'FRP':
                self.frp[perceptor[1][1]].set(perceptor[2][1:], perceptor[3][1:])

            # unknown perceptor
            else:
                if self.debugLevel >= 10:
                    print("DEBUG: unknown perceptor: {}".format(perceptor[0]))
                    print(perceptor)
         
# ==================================== #

    def rock_hj(self, hj, speed, minAngle, maxAngle):
        """Rock a hinge joint at the given speed between min and max degrees"""

        he = self.hjEffector[hj]
        speed = abs(speed)

        if abs(self.he[he]) < speed:
            self.pns.hinge_joint_effector(he, speed)
            self.he[he] = speed

        if self.hj[hj] > maxAngle and self.he[he] > 0:
            self.pns.hinge_joint_effector(he, -speed)
            self.he[he] = -speed
        elif self.hj[hj] < minAngle and self.he[he] < 0:
            self.pns.hinge_joint_effector(he,  speed)
            self.he[he] =  speed

        return "not done"

# ==================================== #

    def move_hj_to(self, hj, angle=None, percent=None, speed=25):
        """Move the given hinge joint to the specified angle
        The angle can be given in degree (angle=<degree>)
        or percent (percent=<percentage>). If both are specified, the angle keyword
        gets priority.
        Speed is specified in percent of maximum speed"""

        # get corresponding hinge effector
        he = self.hjEffector[hj]

        if angle == None and percent == None:
            raise Exception("Either angle or percent must be specified in move_hj_to()")
        elif angle != None:
            # just to give angle keyword priority over percent
            pass
        else:
            angle = self.hjMin[hj] + percent/100.0*(self.hjMax[hj]-self.hjMin[hj])
        
        if   angle > self.hjMax[hj]: angle = self.hjMax[hj]
        elif angle < self.hjMin[hj]: angle = self.hjMin[hj]
        if   speed > 100: speed = 100.0
        elif speed < 0:   speed = 0.0

        speed = speed/100.0 * self.maxhjSpeed
        accuracy = 0.1
        diff  = self.hj[hj] - angle

        if abs(diff) <= accuracy:
            self.pns.hinge_joint_effector(he, 0.0)
            self.he[he] = 0.0
            if self.debugLevel > 20:
                print(hj, "done")
            return "done"

        if abs(speed) > abs(diff)/4.0:
            speed = abs(diff)/4.0

        if self.debugLevel > 20:
            print("hj: {}, he: {} target={:.2f}, current={:.2f}, diff={:.2f}, speed={:.2f}".format(hj, he, angle, self.hj[hj], diff, speed))

        if self.hj[hj] < angle:
            self.pns.hinge_joint_effector(he, abs(speed))
            self.he[he] = abs(speed) 
        elif self.hj[hj] > angle:
            self.pns.hinge_joint_effector(he, -abs(speed))
            self.he[he] = -abs(speed)

        return "not done"

# ==================================== #

    def move_hj_by(self, hj, angle=None, percent=None, speed=25):
        """Move the given hinge joint by the specified angle
        The angle can be given in degree (angle=<degree>)
        or percent (percent=<percentage>). If both are specified, the angle keyword
        gets priority.
        Speed is specified in percent of maximum speed""" 

        if angle == None and percent == None:
            raise Exception("Either angle or percent must be specified in move_hj_by()")

        elif angle != None:
            # just to give angle keyword priority over percent
            pass

        else:
            angle = percent/100.0*(self.hjMax[hj]-self.hjMin[hj]) 

        targetAngle = self.hj[hj] + angle
        kwDict = {'hj': hj, 'angle': targetAngle, 'speed': speed}
        self.msched.append([self.move_hj_to, kwDict])

        return "done"

# ==================================== #

    def get_hj(self, hj):
        """Return hj value in percent"""

        angle    = self.hj[hj]
        minAngle = self.hjMin[hj]
        maxAngle = self.hjMax[hj]

        percent  = 100.0 * (angle - minAngle) / (maxAngle - minAngle)

        return percent

# ==================================== #

    def step_left(self):
        """Make a step with the left foot"""

        done = [False]
        print("ACC:", np.linalg.norm(self.acc.get()))

        done[0] = False
        self.msched.append([self.move_hj_to, {'hj': 'rlj3', 'percent': 50}, done])
        self.msched.append([self.move_hj_to, {'hj': 'rlj4', 'percent': 50}, done])
        while not done[0]:
#            print("ACC:", np.linalg.norm(self.acc.get()))
#            print("GYR:", self.gyr.get())
            print(self.gyr.x)
            print(self.gyr.y)
            print(self.gyr.z)
            print("")
            time.sleep(0.05)

        time = time.time()
        time.sleep(1.0)
            
#        done[0] = False
#        self.msched.append([self.move_hj_to, {'hj': 'hj1', 'percent': 0}, done])
#        while not done[0]:
#            pass 
#        done[0] = False
#        self.msched.append([self.move_hj_to, {'hj': 'hj1', 'percent': 50}, done])
#        while not done[0]:
#            pass  


# ============================================================================ #

#####################
# UTILITY FUNCTIONS #
#####################

def rotate_arbitrary(axis, point, angle=None, degree=True):
    """Rotate the 3D point about the given axis.
    If axis is not normalized and angle is None, the angle is taken as the norm
    of axis. If degree == False, angles are expected in radiant."""

    # ensure axis is unit length
    axisNorm = np.linalg.norm(axis)
    if axisNorm == 0: return point
    axisn    = axis / axisNorm
    if angle == None:
        angle = axisNorm
        
    if degree:
        # convert to radiant
        angle *= np.pi / 180.0

    u = axisn[0]
    v = axisn[1]
    w = axisn[2]
    x = point[0]
    y = point[1]
    z = point[2]

    sin = math.sin(angle)
    cos = math.cos(angle)
    dot = np.dot(axisn, point)
    tmp = dot * (1 - cos)

    result = np.array([u * tmp + x*cos + (-w*y + v*z) * sin, \
                       v * tmp + y*cos + ( w*x - u*z) * sin, \
                       w * tmp + z*cos + (-v*x + u*y) * sin])

    return result

# ============================================================================ #

##############
# EXCEPTIONS #
##############

class SchedulerConflict(Exception):
    """Raised upon trying to schedule a function that
    is already in the MovementScheduler"""
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class QueueItemError(Exception):
    """Raised if an item to be schedule with the MovementScheduler
    has the wrong format"""
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value) 
 
