import numpy as np

class RRT():
    """Simply Rapidly exploring random tree implementation for path planning
    Does not generate the most optimal path, post processing is needed.
    """

    def __init__(self,STATE_SPACE=[[0,480],[0,640]], COLL_DIST=10.0, GOAL_STOP_RADIUS=10.0,LINE_SIZE=(10,10), MAP=None):
        """Rapidly exploring random tree init 

        Args:
            STATE_SPACE (list, optional): define the scene dimensions. Defaults to [[0,480],[0,640]].
            COLL_DIST (float, optional): distance for collosion. Defaults to 10.0.
            GOAL_STOP_RADIUS (float, optional): Area to check if gola point is near enough. Defaults to 10.0.
            LINE_SIZE (tuple, optional): distance betwen current point and random point. Defaults to (10,10).
        """
        self.spXmin=STATE_SPACE[1][0]
        self.spXmax=STATE_SPACE[1][1]

        self.spYmin=STATE_SPACE[0][0]
        self.spYmax=STATE_SPACE[0][1]

        self.coll_dist = COLL_DIST
        self.goal_stop_dist = GOAL_STOP_RADIUS
        self.lineSize = LINE_SIZE
        self.last_point=None
        self.points = []
        self.map = MAP

    def ___get_point_on_a_line(self,start=[0,0],stop=[10,10], linedist=[10,10]):
        """Gnerate a point on a line knoweing the start / stop points.
        No sin / cos or polar coordinate sistem is used, to gain speed

        Args:
            start (list, optional): Start point. Defaults to [0,0].
            stop (list, optional): Stop (Goal) point. Defaults to [10,10].
            linedist (list, optional): Line distance between 2 points. Defaults to [10,10].

        Returns:
            array (1,2): point position on the line in concordance with line distance
        """
        # draw a number of points between start and stop
        pn = []
        start = np.array(start)
        stop = np.array(stop)
        
        m3d = stop-start

        # compute the number of points needed to keep the distance defined
        mx = abs(int(m3d[0][0]/linedist[0]))
        my = abs(int(m3d[0][1]/linedist[1]))
        
        if mx <= 1 or my <= 1:

            #elliminate corner case (horizontal / vertical lines)
            pn = None

        else:
            # no corner case (horizontal/ vertical line), proceed further
            spx = np.array(np.linspace(0,1,mx)).T[1]
            spy = np.array(np.linspace(0,1,my)).T[1]

            Pm = np.array((spx,spy)).T
            
            # line equation
            pn = start + np.multiply(Pm,m3d)
        return pn


    def rtt(self,start,stop, MAX_ITER=1000):
        """Expand the RRT 

        Args:
            start (array (1,2)): Start point
            stop (array(1,2)): Stop / Goal point
            MAX_ITER (int, optional): Max iteration for search. Defaults to 1000.
        """
        self.points = []

        # array to hold the rrt points, used for vector operations
        lpt = []

        # init points, lists
        startp = np.array([start[0],start[1]])
        stopp = np.array([stop[0],stop[1]])
        
        self.points.append({'prev':None,'actual':startp})

        lpt.append(startp)
        lpt = np.array(lpt)

        self.last_point = startp

        for i in range (MAX_ITER):

            # get random point in state space
            ptx = np.random.uniform(self.spXmin, self.spXmax, (1))
            pty = np.random.uniform(self.spYmin, self.spYmax, (1))
            pt = np.array([ptx,pty]).T

            # prepare for vector operation
            cp = np.ones((len(lpt),2))*pt

            # compute euclidian distance betweeen available and new point
            pdist = np.linalg.norm(lpt-cp, axis=1, keepdims=False)
            pdist = np.array(pdist)

            # get closest point to the new point
            mp = lpt[np.argmin(pdist)]

            # generate a point with the max dist constraraint
            npt = self.___get_point_on_a_line(mp,pt,self.lineSize)

            if npt is None:
                continue

            if self.map is not None:
                # creat an array for vector operation
                cp = np.ones((len(self.map),2))*npt
            else:
                print ("MAP is missing...")
                exit()

            # compute euclidian distance between environment and point
            dist = np.linalg.norm(self.map-cp, axis=1, keepdims=False)

            # test if the point is colliding
            if np.min(dist)<=self.coll_dist:
                continue

            # no collision, append new point
            lpt=np.append(lpt,npt).reshape(-1,2)

            # record currnet point pairs
            self.points.append({'prev':mp,'actual':npt[0]})

            # update last point
            self.last_point = npt

            # exit if path found
            edist = np.linalg.norm(stop-npt, axis=1, keepdims=False)

            # last point is near to destination, exit
            if np.all(edist <= self.goal_stop_dist):
                self.last_point = stopp
                self.points.append({'prev':npt[0],'actual':stopp})
                break


    def get_path(self):
        """Generate the path from start till goal point

        Returns:
            array(1,2): path points
        """

        # holds the points for path
        ret = list()
        sp = self.last_point

        ret.append(sp)

        # search backwards till start point reached
        while self.last_point is not None:

            # get the actual element from the path list, used to reviver the previous point
            v = list(filter(lambda id: np.array_equal(id['actual'],np.array(self.last_point)), self.points))

            # no point, no path 
            if len(v) <= 0:
                break
            
            # start point reached, finish the search
            if v[0]['prev'] is None:
                break

            # update current point, record the path
            self.last_point = v[0]['prev']
            ret.append (self.last_point)

        return ret

    def get_points(self):
        """Return all points generated

        Returns:
            dict of {"prev":prev point, "actual": actual point}: points generated by rrt
        """
        return self.points