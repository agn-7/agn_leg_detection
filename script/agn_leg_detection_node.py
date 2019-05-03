#! /usr/bin/env python
import rospy
import numpy

from math import *
from numpy.ma.core import abs

from sensor_msgs.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *

from agn_leg_detection.msg import *

__author__ = 'aGn'


class Cluster(object):
    X = 1

    def __init__(self, cloud, start, end):
        self.start = start
        self.end = end
        self.cloud = cloud

    @staticmethod
    def distance():
        return 123


class Leg(Cluster):
    X = 2

    @staticmethod
    def is_pair():
        print("hi")


class LegDetection(object):
    threshold = 0.07   # 2ta range  # TODO
    threshold2 = 0.05  # fasele 2ta point  # TODO
    threshold3 = 0.6
    threshold_angle = 90.0
    
    maxLenght = 0.16
    minLenght = 0.07
    farLenght = 0.35  # fasele mabeyn 2 pa
    outOfRange = 20
    togetherMin = 0.18
    togetherMax = 35
    togetherPoint_thresold = 0.015
    
    threshold_diagonal = 0.03
    
    laserResolution = 0.25
    
    counter = 0
    legsCount = []
    countTry = 20
    tedad = 0 
    tmp_legs = []
    tmp_tog = []
    tmp_clusters = []    
    crowd_sizer = 0
    z = 0
    start = False
    startResp = True
    REAL_CROWD_SIZE = []
    crowdSizer = []
    
    markerPublisher = visualization_msgs.msg.MarkerArray()    
                           
    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.down_laser_cb, None, 10)
        
        self.crowdPub = rospy.Publisher('/legs_crowd', legmsg, queue_size=10)
        self.crowdPub2 = rospy.Publisher('/legs_crowd2', Legs, queue_size=10)
        self.msg = legmsg()

        self.legPublisher = rospy.Publisher("/LegDetectionRange", Float32, queue_size=1)
        self.legPublisher2 = rospy.Publisher("/LegDetectionPoint", Float32, queue_size=1)
        self.markerPublisher = rospy.Publisher(
            "/LegMarker", visualization_msgs.msg.MarkerArray,
            queue_size=10
        )
        self.legPublisherScenario = rospy.Publisher(
            '/LegDetected', std_msgs.msg.Bool, queue_size=10
        )  # for scenario
        self.rate = rospy.Rate(100)  # 100hz
    
    def visualize(self, data):
        arr = MarkerArray()
        i = 0
        for points in data:
            m1 = self.genMarker(points[0], points[1], True, i)
            m1.lifetime = rospy.Duration(1, 1)
            m2 = self.genMarker(points[2], points[3], False, i + 1)
            m2.lifetime = rospy.Duration(1, 1)
            arr.markers.append(m1)
            arr.markers.append(m2)            
            i += 2

        self.markerPublisher.publish(arr)
        
    def genMarker(self, x, y, is_start, id):
        m = visualization_msgs.msg.Marker()         
        m.header.frame_id = '/velodyne'
        m.header.stamp = rospy.Time.now()        
        m.ns = 'leg_points'
        m.action = visualization_msgs.msg.Marker.ADD
        m.pose.orientation.w = 1.0
        m.id = id
        m.type = visualization_msgs.msg.Marker.SPHERE                       
        
        if is_start:
            m.color.r = 1.0
        else:
            m.color.g = 1.0
        
        m.color.a = 1.0
        
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0
        
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1
        
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 0.1
        
        return m
    
    def merger(self, Legs_Indexes, togetherLegs, clustersPoints):
        merged_legs = []
        temp_cluster = clustersPoints
        i = 0
        
        while i < len(Legs_Indexes):
            j = 0
            
            while j < len(togetherLegs) :                
                
                if temp_cluster[Legs_Indexes[i][0]][0] < temp_cluster[togetherLegs[j]][0] and temp_cluster[togetherLegs[j]][0] != 1000  :
                    self.crowd_sizer += 1 
                    merged_legs.append(Legs_Indexes[i]) 
                    self.z += 1
                    self.z += 1
                    temp_cluster[Legs_Indexes[i][0]][0] = 1000  # baraye inke digar baresi nashavad
                    
                elif temp_cluster[togetherLegs[j]][0] < temp_cluster[Legs_Indexes[i][0]][0] and temp_cluster[Legs_Indexes[i][0]][0] != 1000 :
                    self.crowd_sizer += 1
                    merged_legs.append([togetherLegs[j]])
                    self.z += 1
                    temp_cluster[togetherLegs[j]][0] = 1000                    
                
                j += 1
            
            i += 1        
        
        k1 = 0
             
        while k1 < len(Legs_Indexes):  # baraye anhaE k baqi mandand va digar niaz b moqayeseE nadarand va bayad ba haman tarTb b merged_legs bravand
            
            if temp_cluster[Legs_Indexes[k1][0]][0] != 1000:
                self.crowd_sizer += 1
                merged_legs.append(Legs_Indexes[k1])
                #merged_legs[self.z].append(self.crowd_sizer)
                self.z += 1
                self.z += 1
            
            k1 += 1
                
        k2 = 0
        
        while k2 < len(togetherLegs):
            
            if temp_cluster[togetherLegs[k2]][0] != 1000 :
                self.crowd_sizer += 1
                merged_legs.append([togetherLegs[k2]])

            k2 += 1
        
        self.z = 0  # reset
        self.crowd_sizer = 0  # reset
        return merged_legs        
    
    def crowd_detector(self, merged_legs, clustersPoints, laser):
        
        if len(merged_legs) > 0:
            crowd_legs = []
            crowd_legs.append([])
            crowd_legs[0].append(merged_legs[0])

        i = 0
        j = 0
        tmp_size2 = []
        tmp_size2.append([])
                     
        while i < len(merged_legs) - 1:
            a = ((clustersPoints[merged_legs[i + 1][0]][0]) - (clustersPoints[merged_legs[i][len(merged_legs[i]) - 1]][len(clustersPoints[merged_legs[i][len(merged_legs[i]) - 1]]) - 1]))  # gaEde shodam :|
                         
            b = laser.ranges[ clustersPoints[merged_legs[i][len(merged_legs[i]) - 1]][len(clustersPoints[merged_legs[i][len(merged_legs[i]) - 1]]) - 1] ] #akhareye paye2
            c = laser.ranges[ clustersPoints[merged_legs[i + 1][0]][0] ]
            
            localDegreeOfCluster = a * self.laserResolution
            rad2 = (localDegreeOfCluster * pi) / 180
            
            twoLegsDistance = (sqrt((pow(b, 2) + pow(c, 2)) - (2 * b * c * cos(rad2))))
             
            if twoLegsDistance < self.threshold3 :                
                crowd_legs[j].append(merged_legs[i + 1])                                                                
            
            else:
                crowd_legs.append([])
                tmp_size2.append([])
                j += 1

            i += 1  
         
        k = 0
        self.crowdSizer = []
        self.REAL_CROWD_SIZE = []

        while k < len(crowd_legs):
            if len(crowd_legs[k]) >= 4 and len(crowd_legs[k]) <= 11:
                self.REAL_CROWD_SIZE.append(len(crowd_legs[k]))
                self.crowdSizer = crowd_legs[k] #nabayad b surate append bashad
                print "sizee", self.REAL_CROWD_SIZE   
                print "crowd_sizer", self.crowdSizer
            
            k += 1     

        return self.REAL_CROWD_SIZE, self.crowdSizer

    def cluster(self):
        raise NotImplementedError()

    def cluster_filtering(self):
        raise NotImplementedError()

    def curve_detection(self):
        raise NotImplementedError()

    def curve_filtering(self):
        raise NotImplementedError()

    def down_laser_cb(self, laser):
        self.laserResolution = 360.0 / len(laser.ranges)
        # clustersPoints = self.cluster(laser)  # TODO
        # good_markers, good_indices = self.cluster_filtering(clustersPoints, laser)  # TODO
        # angle = self.curve_detection(good_markers, laser, clustersPoints)  # TODO
        # curv_indexes = self.curve_filtering(angle, clustersPoints, good_markers)  # TODO

        veto = True
        if self.legPublisher.get_num_connections() > 0 or self.crowdPub2.get_num_connections() or veto:
            print("Start ...")
            i = 0
            localEndPoint = len(laser.ranges) - 1
            clustersPoints= []
            clustersPoints.append([])
            appendFlag = False
            remote_points_flag = True
                   
            while i < localEndPoint:  # clustering
                rad = (self.laserResolution * pi) / 180  
                b = laser.ranges[i]
                c = laser.ranges[i + 1]                          
                Distance = sqrt((pow(b, 2) + pow(c, 2)) - (2 * b * c * cos(rad)))

                if laser.ranges[i] > 2 and remote_points_flag:
                    '''Increase threshold between two points in remote points.'''
                    self.threshold += 0.035
                    remote_points_flag = False
                elif laser.ranges[i] < 2 and not remote_points_flag:
                    '''Revert to default.'''
                    self.threshold -= 0.035
                    remote_points_flag = True

                if (Distance < self.threshold) and \
                        (laser.ranges[i] < self.outOfRange) and (laser.ranges[i] != 0):
                    '''Cluster gather.'''
                    clustersPoints[len(clustersPoints) - 1].append(i)
                    appendFlag = True  
                    
                elif (Distance > self.threshold) and appendFlag and \
                        (laser.ranges[i] < self.outOfRange) and (laser.ranges[i] != 0):
                    '''Cluster cutter.'''
                    clustersPoints.append([])
                    appendFlag = False 
                       
                i = i + 1    
                 
            del clustersPoints[len(clustersPoints) - 1]  # yeduneye akhari xalie hamishe

            i = 0
            while i < len(clustersPoints) - 1:  # better clustering
                a = (clustersPoints[i + 1][0]) - (clustersPoints[i][len(clustersPoints[i]) - 1])
                             
                b = laser.ranges[clustersPoints[i][len(clustersPoints[i]) - 1]]
                c = laser.ranges[clustersPoints[i + 1][0]]
                
                localDegreeOfCluster = a * self.laserResolution
                rad2 = (localDegreeOfCluster * pi) / 180
                
                clusterDistance = (sqrt((pow(b, 2) + pow(c, 2)) - (2 * b * c * cos(rad2))))
                 
                if clusterDistance < self.threshold2:
                    clustersPoints[i] = clustersPoints[i] + clustersPoints[i + 1]
                    del clustersPoints[i + 1]
                    i = i - 1  # baraye inke 2bare haman index ra check konad
                    
                i = i + 1   

            clustersDistance = []
            indexes = []
            markIndexes = []
            i = 0
            while i < len(clustersPoints):  # clusters Distance
               a = len(clustersPoints[i]) - 1
               b = laser.ranges[clustersPoints[i][len(clustersPoints[i]) - 1]]
               c = laser.ranges[clustersPoints[i][0]]
               localDegreeOfCluster = a * self.laserResolution
               rad3 = (localDegreeOfCluster * pi) / 180           
               clustersDistance.append(sqrt((pow(b, 2) + pow(c, 2)) - (2 * b * c * cos(rad3))))
               if (len(clustersPoints[i]) % 2) == 0:
                   middle = (len(clustersPoints[i]) / 2) - 1
               
               elif (len(clustersPoints[i]) % 2) == 1:
                   middle = (len(clustersPoints[i]) / 2)
                                   
               indexes.append(clustersPoints[i][middle])
               markIndexes.append(i)  # indexe clusterha
               i += 1

            i = 0         
            goodClusters = []
            goodIndexes  = []
            goodMarkers = []
            while i < len(clustersDistance):  # good distance choose
                if clustersDistance[i] > self.minLenght and clustersDistance[i] < self.maxLenght:
                    goodClusters.append(clustersDistance[i])  # faselehaye xub
                    goodIndexes.append(indexes[i])  # index haye mianie xube laser.
                    goodMarkers.append(markIndexes[i])  # indexe clusterhaye xub
                    
                i = i + 1
           
            i = 0
            dist1 = []
            dist2 = []
            dist3 = []
            angle = []
            while i < len(goodMarkers):  # curve detection
                gm = goodMarkers[i]
                a = laser.ranges[clustersPoints[gm][0]] 
                z = laser.ranges[clustersPoints[gm][len(clustersPoints[gm]) - 1]]
                n1 = len(clustersPoints[gm]) - 1
                localDegreeOfCluster = n1 * self.laserResolution
                rad6 = (localDegreeOfCluster * pi) / 180
                Distance1 = (sqrt((pow(a, 2) + pow(z, 2)) - (2 * a * z * cos(rad6))))
                dist3.append(Distance1) 
                dist1.append([])
                dist2.append([])
                angle.append([])  

                j = 0
                while j < len(clustersPoints[gm]) - 2:  # baraye inke avvali va akhari dar bala hesab shode and.
                    n2 = ((clustersPoints[gm][j + 1]) - (clustersPoints[gm][0]))
                    n3 = ((clustersPoints[gm][len(clustersPoints[gm]) - 1]) -
                          (clustersPoints[gm][j + 1]))
                    b = laser.ranges[clustersPoints[gm][j + 1]]                
                    localDegreeOfCluster2 = n2 * self.laserResolution
                    localDegreeOfCluster3 = n3 * self.laserResolution
                    rad7 = (localDegreeOfCluster2 * pi) / 180
                    rad8 = (localDegreeOfCluster3 * pi) / 180
                    Distance2 = (sqrt((pow(a, 2) + pow(b, 2)) - (2 * a * b * cos(rad7))))
                    Distance3 = (sqrt((pow(z, 2) + pow(b, 2)) - (2 * z * b * cos(rad8))))
                    dist1[i].append(Distance2)
                    dist2[i].append(Distance3)
                    angl = numpy.arccos((pow(dist1[i][j], 2) + pow(dist2[i][j], 2) -
                                         pow(dist3[i], 2)) / (2 * dist1[i][j] * dist2[i][j]))
                    angl = (angl * (180 / pi))  # radian to degree
                    angle[i].append(angl)

                    j = j + 1
                    
                i = i + 1
                
            ##############filter kardane clusterha ba zavie monaseb##################
            i = 0
            curv = []        
            cmp2 = 0  
            cmp3 = 1
            curv_indexes = []      
            while i < len(angle):            
                j = 0
                while j < len(angle[i]) - 1:
                    cmp3 = cmp3 + 1
                                     
                    if (abs(angle[i][j] - angle[i][j + 1]) < 25) and \
                            (abs(angle[i][j] - self.threshold_angle) < 25):
                        print('matchhhhhhhhhhhhh')
                        cmp2 = cmp2 + 1  # increase match counter
                                                
                    if ((len(angle) - cmp3) == 0) and \
                            (abs(angle[i][j + 1] - self.threshold_angle) < 25):  # baraye inke zaviye akhar ham barresi shavad
                        print('mathchhhhhhhhhh22')
                        cmp2 = cmp2 + 1  # increase match counter
                                                                                                                                          
                    j = j + 1
                    
                if (len(angle[i]) - cmp2) < ((50.0 * len(angle[i])) / 100):
                    '''If 50% is match, entered.'''
                    curv.append(clustersPoints[goodMarkers[i]])
                    curv_indexes.append(goodMarkers[i])
                        
                i = i + 1
            
            print("good_Markers", goodMarkers)
            print("curve_indexes", curv_indexes)

            ####Pair Leg Detection####wear, bayad moqayeseE shavad, yany hame ba hame, na motevali
            _Legs = []
            Legs_Indexes = []
            i = 0
            k = -1
                    
            while i < len(curv) - 1:
                print(11111111111111111111111)
                end1 = len(clustersPoints[curv_indexes[i]]) - 1
                end2 = len(clustersPoints[curv_indexes[i + 1]]) - 1                                         
                a = ((clustersPoints[curv_indexes[i + 1]][0]) - (clustersPoints[curv_indexes[i]][end1]))
                b = laser.ranges[clustersPoints[curv_indexes[i]][end1]]
                c = laser.ranges[clustersPoints[curv_indexes[i + 1]][0]]                
                localDegreeOfCluster = a * self.laserResolution
                rad8 = (localDegreeOfCluster * pi) / 180            
                twoLegsDistance = sqrt((pow(b, 2) + pow(c, 2)) - (2 * b * c * cos(rad8)))
                d = len(clustersPoints[curv_indexes[i]])
                e = laser.ranges[clustersPoints[curv_indexes[i]][0]]
                f = len(clustersPoints[curv_indexes[i + 1]])
                g = laser.ranges[clustersPoints[curv_indexes[i + 1]][end2]]
                localDegreeOfCluster2 = d * self.laserResolution
                localDegreeOfCluster3 = f * self.laserResolution
                rad9 = (localDegreeOfCluster2 * pi) / 180
                rad10 = (localDegreeOfCluster3 * pi) / 180
                diagonal1 = sqrt(pow(e, 2) + pow(b, 2) - (2 * b * e * cos(rad9)))
                diagonal2 = sqrt(pow(g, 2) + pow(c, 2) - (2 * g * c * cos(rad10)))
                
                if (twoLegsDistance < self.farLenght) and (abs(diagonal1 - diagonal2) < self.threshold_diagonal):
                    k = k + 1
                    _Legs.append(curv[i] + curv[i + 1])
                    Legs_Indexes.append([])
                    Legs_Indexes[k].append(curv_indexes[i])
                    Legs_Indexes[k].append(curv_indexes[i + 1])                
                    i = i + 1  # baraye inke agar vared if shod yany inke 1 joft pa kamel shod va Dgar niaz b moqayese paye dovvome aan joft pa ba clustere Dgar nist.           
                                                
                i = i + 1            
                        
            print("leg_indexes", Legs_Indexes)

            #######################Detect pair togetherLegs########################
            i = 0
            goodClustersTogetherLegs = []
            goodIndexesTogetherLegs = []
            goodMarkersTogetherLegs = [] 
            
            while i < len(clustersDistance):  # good distance togetherLegs choose
                if clustersDistance[i] > self.togetherMin and \
                        (clustersDistance[i] < self.togetherMax):
                    goodClustersTogetherLegs.append(clustersDistance[i]) #faselehaye xub dar in zakhire mishavand
                    goodIndexesTogetherLegs.append(indexes[i])     # index haye mianie xube laser. #pointhaye mianie cluster ha
                    goodMarkersTogetherLegs.append(markIndexes[i])  # indexe clusterhaye xub
                    
                i = i + 1                
               
            print("goodMarkersTogetherLegs", goodMarkersTogetherLegs)           
                     
            i = 0
            string = []
            togetherLegs = [] 
            #####for simple way#####
            angle11 = []
            angle22 = []
            dist1 = []
            dist2 = []
            dist3 = []
            dist4 = []
            dist5 = []
            dist6 = []
            while i < len(goodMarkersTogetherLegs):
                j = 0
                string.append([])  
                flag1 = True  # for pass the first time
                flag2 = True      
                c1 = 0
                c2 = 0    
                while j < len(clustersPoints[goodMarkersTogetherLegs[i]]) - 1:
                    a = laser.ranges[clustersPoints[goodMarkersTogetherLegs[i]][j + 1]]
                    b = laser.ranges[clustersPoints[goodMarkersTogetherLegs[i]][j]]
                    if abs(a - b) > self.togetherPoint_thresold:
                        if a < b:  # is L
                            c1 = c1 + 1
                            if not flag1:
                                if (string[i][len(string[i]) - 1] == "R") and c1 > 2:  # yany hade aqal 2 ta bayad az in halat bashe.
                                    string[i].append("L")                                
                                    
                            if flag1 and c1 > 2:
                                string[i].append("L")
                                flag1 = False        
                           
                        elif b < a:  # is R
                            c2 = c2 + 1  
                            if not flag2 :
                                if (string[i][len(string[i]) - 1] == "L") and c2 > 2:                                
                                    string[i].append("R")                                
                            
                            if flag2 and c2 > 2:
                                string[i].append("R")
                                flag2 = False        
                                
                    j = j + 1
                
                # word = ["L","R","L","R"]
                
                if len(string[i]) == 4:  # this mean LRLR number is 4
                    if string[i][0] == "L" and string[i][1] == "R" and \
                            string[i][2] == "L" and string[i][3] == "R":
                        print('Matched')
                        # TODO :: doing stuff.
                
                ###########################new and simple approach######################
                gmtl = goodMarkersTogetherLegs[i]
                a = laser.ranges[clustersPoints[gmtl][0]] 
                b1 = goodIndexesTogetherLegs[i]  # pointe mianie cluster haye ehtemalan togetherlegs
                mid = laser.ranges[b1]
                z = laser.ranges[clustersPoints[gmtl][len(clustersPoints[gmtl]) - 1]]
                n1 = int((len(clustersPoints[gmtl]) - 1) / 2)
                localDegreeOfCluster = n1 * self.laserResolution
                rad6 = (localDegreeOfCluster * pi) / 180
                Distance1 = (sqrt((pow(a, 2) + pow(mid, 2)) - (2 * a * mid * cos(rad6))))
                Distance4 = (sqrt((pow(mid, 2) + pow(z, 2)) - (2 * mid * z * cos(rad6))))
                dist3.append(Distance1)
                dist4.append(Distance4) 
                dist1.append([])
                dist2.append([])
                dist5.append([])
                dist6.append([])
                angle11.append([])
                angle22.append([])  
                k = 0
                                                    
                while k < (int(len(clustersPoints[gmtl]) - 1) / 2) - 1:  # baraye inke avvali va akhari va vasaty dar bala hesab shode and.
                    n2 = ((clustersPoints[gmtl][k + 1]) - (clustersPoints[gmtl][0]))
                    n3 = (b1 - (clustersPoints[gmtl][k + 1]))
                    b = laser.ranges[clustersPoints[gmtl][k + 1]]
                    nextMid = laser.ranges[b1 + k + 1]                
                    localDegreeOfCluster2 = n2 * self.laserResolution
                    localDegreeOfCluster3 = n3 * self.laserResolution
                    rad7 = (localDegreeOfCluster2 * pi) / 180
                    rad8 = (localDegreeOfCluster3 * pi) / 180
                    Distance2 = (sqrt((pow(a, 2) + pow(b, 2)) - (2 * a * b * cos(rad7))))
                    Distance3 = (sqrt((pow(mid, 2) + pow(b, 2)) - (2 * mid * b * cos(rad8))))
                    dist1[i].append(Distance2)
                    dist2[i].append(Distance3)
                    Distance5 = (sqrt((pow(mid, 2) + pow(nextMid, 2)) - (2 * mid * nextMid * cos(rad7))))
                    Distance6 = (sqrt((pow(z, 2) + pow(nextMid, 2)) - (2 * z * nextMid * cos(rad8))))
                    dist5[i].append(Distance5)
                    dist6[i].append(Distance6)
                    angl11 = numpy.arccos((pow(dist1[i][k], 2) + pow(dist2[i][k], 2) -
                                           pow(dist3[i], 2)) / (2 * dist1[i][k] * dist2[i][k]))
                    angl11 = (angl11 * (180 / pi))  # radian to degree
                    angl22 = numpy.arccos((pow(dist5[i][k], 2) + pow(dist6[i][k], 2) -

                                           pow(dist4[i], 2)) / (2 * dist5[i][k] * dist6[i][k]))
                    angl22 = (angl22 * (180 / pi))  # radian to degree
                    angle11[i].append(angl11)
                    angle22[i].append(angl22)
                    k = k + 1

                i = i + 1            
           
            ##############filter kardane clusterha ba zavie monasebe 2 ##################
            i = 0
            cmp2 = 0
            cmp3 = 1
            while i < len(angle11):
                j = 0
                        
                while j < len(angle11[i]) - 1:
                    cmp3 = cmp3 + 1                
                    #  TODO :: all 50(s) was 25 and 50.0%(s) was 30.0%
                    if (abs(angle11[i][j] - angle11[i][j + 1]) < 50) and \
                            (abs(angle11[i][j] - self.threshold_angle) < 50) and \
                            (abs(angle22[i][j] - angle22[i][j + 1]) < 50) and \
                            (abs(angle22[i][j] - self.threshold_angle) < 50):
                        cmp2 = cmp2 + 1                                        
                                                
                    if ((len(angle11) - cmp3) == 0) and \
                            (abs(angle11[i][j + 1] - self.threshold_angle) < 50) and \
                            (abs(angle22[i][j + 1] - self.threshold_angle) < 50):  # baraye inke zaviye akhar ham barresi shavad
                        cmp2 = cmp2 + 1                                                                                   
                    
                    j = j + 1
                    
                if (len(angle11[i]) - cmp2) < ((50.0 * len(angle11[i])) / 100) and \
                        (len(angle22[i]) - cmp2) < ((50.0 * len(angle22[i])) / 100): # yani dar suraT varede if mishavad k ekhtelafe cmp2 ba har goodcluster bayad kamtar az 40% an bashad
                    togetherLegs.append(goodMarkersTogetherLegs[i])
                        
                i = i + 1                    

            print("togetherlegs", togetherLegs)
                                   
            ##################################################################
            cmpIndexes = []        
            i = 0
            
            while i < len(goodMarkers):  # ijade 1 araye baraye pyda kardane un clustri k ruberuye robote.
                cmpIndexes.append(abs(360 - goodIndexes[i])) #360 shot roobe rooye robot
                i = i + 1

            i = 0 
            localMin = 180  # bishtarin chizi k Mkan dare bashe = 360 - 180 = 180
            while i < len(cmpIndexes):   # Ntekhabe clustere ruberuye robot.
                if cmpIndexes[i] < localMin:
                    localMin = cmpIndexes[i]

                i = i + 1

            i = 0
            positions = []
            x1 = []
            y1 = []
            x2 = []
            y2 = []                                
            # create marker
            while i < len(Legs_Indexes):
                '''crrate marker for first approach.'''
                positions.append([])
                lif = Legs_Indexes[i][0]  # LIF means, is, Legs Indexes First
                lie = Legs_Indexes[i][1]  # chon har shakhs 2 pa darad pas 1 yany paye 2vvome fard
                end = len(clustersPoints[lie]) - 1
                end2 = len(clustersPoints[lif]) - 1
                res = self.laserResolution
                ii1 = clustersPoints[lif][0]
                ii9 = clustersPoints[lie][end]
                rad4 = ((ii1 * res * pi) / 180) - pi  # "-pi" is a TF
                rad5 = ((ii9 * res * pi) / 180) - pi  # "-pi" is a TF
                                           
                x1.append(laser.ranges[ii1] * cos(rad4))  # Red
                y1.append(laser.ranges[ii1] * sin(rad4))

                x2.append(laser.ranges[ii9] * cos(rad5))  # Green
                y2.append(laser.ranges[ii9] * sin(rad5))

                positions[i].append(x1[i])
                positions[i].append(y1[i])
                positions[i].append(x2[i])
                positions[i].append(y2[i])
                #############4 human detection##############
                mid = clustersPoints[lif][end2]  # Leg Index Middle
                range2send = Float32(laser.ranges[mid])
                point2send = Float32(mid)
                self.legPublisher.publish(range2send)
                self.legPublisher2.publish(point2send)
                self.rate.sleep()

                i = i + 1
            
            if Legs_Indexes:
                self.visualize(positions)

            i2 = 0
            positions2 = []
            x12 = []
            y12 = []
            x22 = []
            y22 = []                                
            while i2 < len(togetherLegs):
                '''create marker for second approach (simpler and more powerful)'''
                positions2.append([])
                end = len(clustersPoints[togetherLegs[i2]]) - 1
                res = self.laserResolution
                ii1 = clustersPoints[togetherLegs[i2]][0]
                ii9 = clustersPoints[togetherLegs[i2]][end]
                rad4 = ((ii1 * res * pi) / 180) - pi  # "-pi" is a TF
                rad5 = ((ii9 * res * pi) / 180) - pi  # "-pi" is a TF

                x12.append(laser.ranges[ii1] * cos(rad4))  # Read
                y12.append(laser.ranges[ii1] * sin(rad4))

                x22.append(laser.ranges[ii9] * cos(rad5))  # green
                y22.append(laser.ranges[ii9] * sin(rad5))

                positions2[i2].append(x12[i2])
                positions2[i2].append(y12[i2])
                positions2[i2].append(x22[i2])
                positions2[i2].append(y22[i2])
                #############4 human detection##############
                mid2 = clustersPoints[togetherLegs[i2]][int(end) / 2]  # Leg Index Middle
                range2send2 = Float32(laser.ranges[mid2])
                point2send2 = Float32(mid2)
                self.legPublisher.publish(range2send2)
                self.legPublisher2.publish(point2send2)
                self.rate.sleep()
                              
                i2 = i2 + 1
            
            if togetherLegs:
                self.visualize(positions2)


#############test######3
            # i2 = 0
            # positions3 = []
            # x12 = []
            # y12 = []
            # x22 = []
            # y22 = []
            # while i2 < len(goodMarkersTogetherLegs):
            #     positions3.append([])
            #     end = len(clustersPoints[goodMarkersTogetherLegs[i2]]) - 1
            #     res = self.laserResolution
            #     ii1 = clustersPoints[goodMarkersTogetherLegs[i2]][0]
            #     ii9 = clustersPoints[goodMarkersTogetherLegs[i2]][end]
            #     rad4 = ((ii1 * res * pi) / 180) - pi
            #     rad5 = ((ii9 * res * pi) / 180) - pi
            #
            #     x12.append(laser.ranges[ii1] * cos(rad4))  # Read
            #     y12.append(laser.ranges[ii1] * sin(rad4))
            #
            #     x22.append(laser.ranges[ii9] * cos(rad5))  # green
            #     y22.append(laser.ranges[ii9] * sin(rad5))
            #
            #     positions3[i2].append(x12[i2])
            #     positions3[i2].append(y12[i2])
            #     positions3[i2].append(x22[i2])
            #     positions3[i2].append(y22[i2])
            #     #############4 human detection##############
            #     mid2 = clustersPoints[goodMarkersTogetherLegs[i2]][int(end) / 2]  # Leg Index Middle
            #     range2send2 = Float32(laser.ranges[mid2])
            #     point2send2 = Float32(mid2)
            #     self.legPublisher.publish(range2send2)
            #     self.legPublisher2.publish(point2send2)
            #     self.rate.sleep()
            #
            #     i2 = i2 + 1
            #
            # if goodMarkersTogetherLegs:
            #     self.visualize(positions3)
#######################3



            #################Legs Counter Call Back####################
            if self.start:
                self.legsCount.append(len(Legs_Indexes) + len(togetherLegs))
                self.tmp_legs.append(Legs_Indexes)
                self.tmp_tog.append(togetherLegs)
                self.tmp_clusters.append(clustersPoints)

                self.counter += 1
                print self.counter        

                if self.counter == self.countTry :
                    self.counter = 0
                    i = 0
                    self.tedad = max(self.legsCount)
                
                    while self.tedad != self.legsCount[i]: #baraye bdast avardane indexe an clustere ya leg haye tashkhis dade shodeye ziaddd
                        i += 1
                
                    try:
                        merged_legs = self.merger(self.tmp_legs[i], self.tmp_tog[i], self.tmp_clusters[i])                
                        print "merged_legss", merged_legs
                        
                        if len(merged_legs) > 0:  # for scenario
                            self.legPublisherScenario.publish(True)
                        else:
                            self.legPublisherScenario.publish(False)
                            
                        crowdSize, crowd_legs = self.crowd_detector(merged_legs, self.tmp_clusters[i], laser)
                        print "All-legsCount", max(self.legsCount)                        
                        print "crowd_legs", crowd_legs   
                        if len(crowd_legs) > 0:
                            self.msg.count = crowdSize           
                            self.msg.first = self.tmp_clusters[i][crowd_legs[0][0]][0] - 20 #tmp_cluster[][][] (template)
                            # print "inja", self.msg.first
                            self.msg.end = self.tmp_clusters[i][crowd_legs[len(crowd_legs) - 1][len(crowd_legs[len(crowd_legs) - 1]) - 1]][len(self.tmp_clusters[i][crowd_legs[len(crowd_legs) - 1][len(crowd_legs[len(crowd_legs) - 1]) - 1]]) - 1] + 20  #:| :| :|                         
                            self.crowdPub.publish(self.msg)                            
                            legsMsg = Legs()  # msg
                            index = 0
                            
                            while index < len(crowd_legs):
                                res = self.laserResolution
                                end = len(crowd_legs[index]) - 1
                                a = crowd_legs[index][0]
                                b = crowd_legs[index][end]
                                i1 = self.tmp_clusters[i][a][0]
                                rad1 = ((i1 * self.laserResolution * pi) / 180) - (pi / 2)
                                i2 = self.tmp_clusters[i][b][end]
                                rad2 = ((i2 * self.laserResolution * pi) / 180) - (pi / 2)
                                
                                x1 = laser.ranges[i1] * numpy.cos(rad1)
                                y1 = laser.ranges[i1] * numpy.sin(rad1)
                                
                                x2 = laser.ranges[i2] * numpy.cos(rad2)
                                y2 = laser.ranges[i2] * numpy.sin(rad2)
                                
                                x = (x1 + x2) / 2 # X e vasate paye fard
                                y = (y1 + y2) / 2 # Y e vasate paye fard
                                
                                grade = (x2 - x1) / (y2 - y1) #shibe khatte paye fard
                                grade_vertical = -1 * (1 / grade) #shibe makus qarine bar khatte paye fard
                                
                                grade_vertical = 1 * grade_vertical
                                theta = numpy.arctan(grade_vertical) # zaviyeye bordare amud nesbat b laser b dast miayad #arctan2 behtar ast

                                legMsg = Leg() #msg
                                legMsg.pose.position.x    = x
                                legMsg.pose.position.y    = y
                                legMsg.pose.position.z    = 0 #ertefae laser utm paEn 0.36 ast
                                legMsg.pose.orientation.w = 1
                                legMsg.pose.orientation.x = 0
                                legMsg.pose.orientation.y = 0
                                legMsg.pose.orientation.z = theta #rad_theta # chon bayad hole mhvare z davaran dashte bashad
                                
                                
                                legsMsg.legsCrowd.append(legMsg) #yek msg daram k dakhelesh msg e digar list shode ast                                
                                
                                index += 1
                            
                            legsMsg.first = self.tmp_clusters[i][crowd_legs[0][0]][0] - 20   
                            legsMsg.end = self.tmp_clusters[i][crowd_legs[len(crowd_legs) - 1][len(crowd_legs[len(crowd_legs) - 1]) - 1]][len(self.tmp_clusters[i][crowd_legs[len(crowd_legs) - 1][len(crowd_legs[len(crowd_legs) - 1]) - 1]]) - 1] + 20  #:| :| :|
                            legsMsg.count = crowdSize
                            self.crowdPub2.publish(legsMsg)
                        self.legsCount = []  # inha jayeshan mashkuk ast
                        self.tmp_legs = []
                        self.tmp_tog = []
                        self.tmp_clusters = []
                    
                    except Exception as exc:  # rospy.ServiceException as e:
                        print(exc)
                        print "legsCount", max(self.legsCount)
                        self.legsCount = []   
                        self.tmp_legs = []
                        self.tmp_tog = [] 
                        self.tmp_clusters = []


if __name__ == '__main__':
    rospy.init_node('agn_leg_detection_node', anonymous=True)
    LegDetection = LegDetection()
    rospy.spin() 
