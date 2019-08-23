#!/usr/bin/env python
#coding=utf-8

import roslib;
import rospy
import actionlib;
from actionlib_msgs.msg import *;
from geometry_msgs.msg import Pose,PoseWithCovarianceStamped,Point,Quaternion,Twist;
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal;
from random import sample
from math import pow,sqrt;

class NavTest():
    def __init__(self):
        rospy.init_node('random_navigation',anonymous=True);
        #rospy.on_shutdown(self.shutdown);

        #在每个目标位置暂停的时间
        self.rest_time=rospy.get_param("~rest_time",2);

        #到达目标的状态
        goal_states=['PENDING','ACTIVE','PREEMPTED',
                    'SUCCEEDED','ABORTED','REJECTED',
                    'PREEMPTING','RECALLING','RECALLED',
                    'LOST'];

        #设置目标点位置
        locations=list();

        # locations.append(Pose(Point(0.15,0.0,0),Quaternion(0.0,0.0,0.0,1.0)));
        # locations.append(Pose(Point(0.35,0,0),Quaternion(0.0,0.0,0.0,1.0)));
        # locations.append(Pose(Point(0.75,0.0,0),Quaternion(0.0,0.0,0.0,1.0)));
        # locations.append(Pose(Point(1.0,0.0,0),Quaternion(0.0,0.0,0.0,1.0)));
        # locations.append(Pose(Point(1.35,0.0,0),Quaternion(0.0,0.0,0.0,1.0)));

        locations.append(Pose(Point(8.5,1.12,0),Quaternion(0.0,0.0,0.0,1.0)));
        locations.append(Pose(Point(6.3,7.6,0),Quaternion(0.0,0.0,0.0,1.0)));
        locations.append(Pose(Point(4.9,9.2,0),Quaternion(0.0,0.0,0.0,1.0)));
        locations.append(Pose(Point(-3.4,8.5,0),Quaternion(0.0,0.0,0.0,1.0)));
        locations.append(Pose(Point(0.0,0,0),Quaternion(0.0,0.0,0.0,1.0)));

        #控制指令发布
        self.cmd_vel_pub=rospy.Publisher('cmd_vel',Twist,queue_size=5);

        '''
        尽管在ROS中已经提供了srevice机制来满足请求—响应式的使用场景，
        但是假如某个请求执行时间很长，
        在此期间用户想查看执行的进度或者取消这个请求的话，service机制就不能满足了
        message:
        1、goal - Used to send new goals to servers. 代表一个任务，可以被ActionClient发送到ActionServer。
                                    比如在MoveBase中，它的类型是PoseStamped，包含了机器人运动目标位置的信息。

        2、cancel - Used to send cancel requests to servers

        3、status - 通知客户端在每个目标的当前状态

        4、feedback - 服务端定期告知Client当前Goal执行过程中的情况。在Move Base案例中，它表示机器人当前姿态。

        5、result - Used to send clients one-time auxiliary information upon completion of a goal
        '''
        #订阅move_base消息
        self.move_base=actionlib.SimpleActionClient("move_base",MoveBaseAction);

        rospy.loginfo("Waiting for move_base action server....");

        #设置连接超时限制60s
        self.move_base.wait_for_server(rospy.Duration(60));
        rospy.loginfo("Connected to move base server");

        #保存机器人在rviz中的初始位置？
        initial_pose=PoseWithCovarianceStamped();

        #运行参数，成功率、时间、距离
        n_locations=len(locations);
        n_goals=0;
        n_successes=0;
        i=n_locations;  #5?
        distance_traveled=0;
        start_time=rospy.Time.now();
        running_time=0;
        location=Pose();
        last_location=Pose();

        #确保初始位姿已经被初始化
        while initial_pose.header.stamp=="":
            rospy.sleep(1);

        rospy.loginfo("Starting navigation test");

        #开始主循环
        while not rospy.is_shutdown():
            if i==n_locations:
                i=0;

            #获取下一目标点
            location=locations[i];

            #跟踪行驶距离
            if initial_pose.header.stamp=="":
                distance=sqrt(pow(location.position.x-last_location.position.x,2)+
                                pow(location.position.y-last_location.position.y,2));
            else:
                rospy.loginfo("Updating current pose.");
                distance=sqrt(pow(location.position.x-initial_pose.pose.pose.position.x,2)+
                                pow(location.position.y-initial_pose.pose.pose.position.y,2));
                initial_pose.header.stamp="";
        
            #储存上一次的位置
            last_location=location;

            #计数器+1
            i+=1;
            n_goals+=1;

            #设定目标点
            self.goal=MoveBaseGoal(); #初始化目标点
            self.goal.target_pose.pose=location;
            self.goal.target_pose.header.frame_id="map"; #表明该pose是在map坐标系下
            self.goal.target_pose.header.stamp=rospy.Time.now();

            #输出
            rospy.loginfo("Going to :"+str(location));

            #发送目标给move_base
            self.move_base.send_goal(self.goal);

            #5分钟时间限制
            finished_within_time=self.move_base.wait_for_result(rospy.Duration(300));

            #检查是否成功到达
            if not finished_within_time:
                self.move_base.cancel_goal(); #取消目标
                rospy.loginfo("Timed out, and cancel");
            else:
                state=self.move_base.get_state()
                #GoalStatus.SUCCEEDED 是actionlib里面的msg内容
                if state==GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!");
                    n_successes+=1;
                    distance_traveled+=distance;
                    rospy.loginfo("State:"+str(state));
                else:
                    rospy.loginfo("Goal failed with error code:"+str(goal_states[state]));
            
            #所用时间
            running_time=rospy.Time.now()-start_time;
            running_time=running_time.secs/60.0;

            #输出
            rospy.loginfo("Success so far:"+str(n_successes)+"/"+str(n_goals)+"="+str(100*n_successes/n_goals)+"%");

            #trunc()舍弃小数部分
            rospy.loginfo("Running time:"+str(trunc(running_time,1))+"min Distance:"+str(trunc(distance_traveled,1))+"m");

    def update_initial_pose(self,initial_pose):
        self.initial_pose=initial_pose;
    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...");
        self.move_base.cancel_goal();
        rospy.sleep(2);
        self.cmd_vel_pub.publish(Twist());
        rospy.sleep(1);

#输入数字和保留小数点后多少位n
def trunc(f,n):
    slen=len('%.*f' % (n,f));
    return float(str(f)[:slen]);
        

if __name__ == '__main__':
    try:
        NavTest();
        rospy.spin();

    except rospy.ROSInterruptException:
        rospy.loginfo("navigation finished");