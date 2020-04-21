#!/usr/bin/env python

import numpy as np
import numpy.random as npr
from random import random
import math
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion,Twist, Vector3,PoseStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker

class sensor_fusion:

    def __init__ (self,initial_pose,N):
        rospy.Subscriber("/scan", LaserScan, self.subscribe_scan, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.subscribe_odom, queue_size=1)
        rospy.Subscriber("/ORB_SLAM/pose", PoseStamped, self.subscribe_ORB_SLAM, queue_size=1)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.subscribe_amcl_pose, queue_size=1)
        self.odompose = [0.0, 0.0, 0.0]
        self.odomcov = [0.1, 0.1, 0.1] 
        self.odomtwist = [0.0, 0.0]
        self.odomtwistcov = [0.1, 0.01]
        self.orbpose = [0.0, 0.0, 0.0]
        self.amclpose = [0.0, 0.0, 0.0]
        self.amclcov = np.diag([0.1,0.1,0.1])
        self.fusion_pub = rospy.Publisher("/robot_pose",PoseWithCovarianceStamped,queue_size = 1)
        self.robotpos_pub = rospy.Publisher("/robot_marker",Marker,queue_size = 1)
        
        self.isOdomUpdate = False;
        self.isOrbUpdate = False;
        self.isamclUpdate = False;

        self.initial_pose = initial_pose
        self.numParticles = N

    # subscribes to odometry data
    def subscribe_odom(self,msg):
        rospy.loginfo("Odom Info Recieved")
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        phi = pose.orientation
        theta = euler_from_quaternion([phi.x, phi.y, phi.z, phi.w])[2] 
        theta = theta % (2*np.pi)

        self.odompose = [x, y, theta]
        self.odomcov = [msg.pose.covariance[0],msg.pose.covariance[7],msg.pose.covariance[35]]
        
        twist = msg.twist.twist
        self.odomtwist = [twist.linear.x, twist.angular.z]
        self.odomtwistcov = [msg.twist.covariance[0], msg.twist.covariance[35]]
        self.isOdomUpdate = True;

    def subscribe_scan(self,msg):
        return 0        

    def subscribe_ORB_SLAM(self,msg):
        rospy.loginfo("ORB SLAM Data Received")

        pose = msg.pose
        x = pose.position.x
        y = pose.position.y
        phi = pose.orientation
        theta = euler_from_quaternion([phi.x, phi.y, phi.z, phi.w])[2] 
        theta = theta % (2*np.pi) 
        self.orbpose = [x, y, theta]
        self.isOrbUpdate = True;

    def subscribe_amcl_pose(self,msg):
        rospy.loginfo("AMCLPose")
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        phi = pose.orientation
        theta = euler_from_quaternion([phi.x, phi.y, phi.z, phi.w])[2] 
        theta = theta % (2*np.pi)

        self.amclpose = [x, y, theta]
        cov = msg.pose.covariance
        self.amclcov = [[cov[0],cov[1],0],[cov[6],cov[7],0],[0, 0,cov[35]]]
        self.isamclUpdate = True;


    # the particle filter is initialised and executed here
    def pf(self):
        
        particles,weights = self.create_gaussian_particles(self.initial_pose,[1,1,0.1],self.numParticles)
        #weights = np.ones(self.numParticles) / self.numParticles
        rate = rospy.Rate(10)
        rospy.loginfo("Started Particle Filter")
        while not rospy.is_shutdown():
            self.predict(particles,self.odomtwist,self.odomtwistcov,0.1)
            self.update(particles,weights)
            position,cov = self.estimate(particles,weights)
            #Uncertainty in pose
            covariance = [0.0]*36
            covariance[0] = cov[0]
            covariance[7] = cov[1]
            covariance[35] = cov[2]
            rospy.loginfo("State: %s",np.array2string(position))

            yaw = position[2];
            if yaw > np.pi :
                yaw = yaw - 2*np.pi
            q = quaternion_from_euler(0, 0, yaw) 

            rospy.loginfo("State: %s",np.array2string(position))
            #Actual Pose computation
            pose = Pose(Point(position[0],position[1],0),Quaternion(q[0],q[1],q[2],q[3]))    
            poseVar = PoseWithCovarianceStamped()
            poseVar.header.stamp = rospy.Time.now()
            poseVar.header.frame_id = "map"
            poseVar.pose.pose = pose
            poseVar.pose.covariance = covariance
            self.fusion_pub.publish(poseVar)

            #resample particles
            if(self.neff(weights)<self.numParticles/2):
                #particles,weights = self.create_gaussian_particles(position,cov,self.numParticles)
                indexes = self.systematic_resample(weights)
                self.resample(particles, weights, indexes) 
            
            #weights = np.ones(self.numParticles) / self.numParticles
            
            # Create robot marker
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.pose = pose
            self.robotpos_pub.publish(marker)

            rate.sleep();
            
    # prediction set of the particles with noise addition
    def predict(self,particles,control,covariance,dt):
        particles[:, 2] += (control[1] + (npr.randn(self.numParticles) * covariance[1]))*dt
        particles[:, 2] %= 2 * np.pi

        ux = (control[0] * dt) + (npr.randn(self.numParticles) * covariance[0])
        particles[:, 0] += np.cos(particles[:, 2]) * ux
        particles[:, 1] += np.sin(particles[:, 2]) * ux
        #rospy.loginfo(np.array2string(particles))

     # creates a set of particles sampled from a gaussian posterior
    def create_gaussian_particles(self,mean,std,N):
        particles = np.empty((N, 3))
        particles[:, 0] = mean[0] + (npr.randn(N) * math.sqrt(std[0]))
        particles[:, 1] = mean[1] + (npr.randn(N) * math.sqrt(std[1]))
        particles[:, 2] = mean[2] + (npr.randn(N) * math.sqrt(std[2]))
        particles[:, 2] %= (2 * np.pi) 
        weights = np.empty(N)

        for i in range(len(particles[:,0])):
            distance = particles[i,:] - mean
            if distance[2] >np.pi:
                distance[2] = distance[2] - 2*np.pi
            weights[i] = self.getGaussProb(distance,np.array(self.amclcov))

        #rospy.loginfo(np.array2string(particles))

        return particles,weights

    # obtains the gaussian posterior for updating the weights
    def getGaussProb(self,distance,std):
        d = np.dot(std,distance.T)
        d2 = np.dot(distance,d.T)
        return math.exp(-0.5*d2)
  
     # weights are updated here and sensor information is fused here
    def update(self,particles,weights):
        rospy.loginfo("Weights:")
        #Fuse Odometry Pose
        if(self.isOdomUpdate):
            for i in range(len(particles[:,0])):
                distance = particles[i,:] - self.odompose
                if distance[2] > np.pi:
                    distance[2] = distance[2] - 2*np.pi
                weights[i] *= self.getGaussProb(distance,np.diag(self.odomcov))
                
                
        self.isOdomUpdate = False;

        #Fuse amcl pose
        if(self.isamclUpdate):
            for i in range(len(particles[:,0])):
                distance = particles[i,:] - self.amclpose
                if distance[2] >np.pi:
                    distance[2] = distance[2] - 2*np.pi
                weights[i] *= self.getGaussProb(distance,np.array(self.amclcov))
        self.isamclUpdate = False;

        #Fuse orbslam
        if(self.isOrbUpdate):
            for i in range(len(particles[:,0])):
                distance = particles[i,:] - self.orbpose
                if distance[2] > np.pi:
                    distance[2] = distance[2] - 2*np.pi
                weights[i] *= self.getGaussProb(distance,np.diag([0.01,0.01,0.01]))

        self.isOrbUpdate = False;
        #rospy.loginfo(np.array2string(weights))
        #rospy.loginfo("Weight Sum: %d",sum(weights))
        weights /= sum(weights)

    # estimates mean and variance of the new pose of the robot based on updates on particles
    def estimate(self,particles, weights):
        rospy.loginfo("Weight Sum: %lf",sum(weights))
        d = np.copy(particles)
        for i in range(len(particles[:,0])):
            if d[i,2] > np.pi:
                d[i,2] = d[i,2] - 2*np.pi

        mean = np.average(d,weights = weights,axis =0 )
        mean[2] %= (2*np.pi)

        var_s = np.empty((len(particles[:,0]),len(particles[0,:])))
        rospy.loginfo("Mean 2 %s",np.array2string(mean))
        
        for i in range(len(particles[:,0])):
            d = particles[i,:] - mean
            if d[2] > np.pi:
                d[2] = d[2] - 2*np.pi
            var_s[i,:] = d*d

        var = np.average(var_s,weights = weights,axis = 0)
        rospy.loginfo("Var 2c %s",np.array2string(var))    
        return mean, var

    # number of effectie particels contributing to the weighted sum
    def neff(self,weights):
        return 1. / np.sum(np.square(weights))
    
    # resampling particles
    def resample(self,particles, weights, indexes):
        particles[:] = particles[indexes]
        weights[:] = weights[indexes]
        weights.fill(1.0 / len(weights))
    
    def systematic_resample(self,weights):
         N = len(weights)
         positions = (random() + np.arange(N)) / N
         indexes = np.zeros(N, 'i')
         cumulative_sum = np.cumsum(weights)
         i, j = 0, 0
         while i < N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
         return indexes

#initialising and executing particle filter
rospy.init_node('sensor_fusion')
s = sensor_fusion([1.0,0.2,0],1000)
s.pf();
