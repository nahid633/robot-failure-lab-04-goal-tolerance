#!/usr/bin/env python3
"""One goal, unmodified sensing, independent Gazebo world-pose observations."""
import argparse
import csv
import json
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from rcl_interfaces.srv import GetParameters, ListParameters
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from core import CONDITIONS, TOLERANCES, START, GOAL, PARAMETER, endpoint_metrics


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Observer(Node):
    def __init__(self):
        super().__init__('failure_lab_04_observer')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.lock = threading.RLock()
        self.truth=[]; self.localized=[]; self.odom=[]; self.scans=[]; self.commands=[]
        self.error=None
        self.create_subscription(TFMessage, '/rfl04/truth', self.world_pose, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.motion, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.command, qos_profile_sensor_data)
        self.buffer=Buffer(cache_time=Duration(seconds=15))
        self.listener=TransformListener(self.buffer,self,spin_thread=False)
        self.create_timer(.05,self.map_pose)

    def now(self): return self.get_clock().now().nanoseconds/1e9
    @staticmethod
    def stamp(msg): return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def world_pose(self,msg):
        with self.lock:
            for tr in msg.transforms:
                if tr.child_frame_id.split('::')[-1] != 'turtlebot3_waffle': continue
                p=tr.transform.translation
                self.truth.append(dict(t=self.now(),stamp=self.stamp(tr),x=p.x,y=p.y,
                    yaw=yaw(tr.transform.rotation),frame=tr.header.frame_id,child=tr.child_frame_id))

    def map_pose(self):
        with self.lock:
            try: tr=self.buffer.lookup_transform('map','base_link',Time())
            except Exception: return
            p=tr.transform.translation
            self.localized.append(dict(t=self.now(),stamp=self.stamp(tr),x=p.x,y=p.y,yaw=yaw(tr.transform.rotation)))

    def motion(self,msg):
        with self.lock:
            v=msg.twist.twist
            self.odom.append(dict(t=self.now(),stamp=self.stamp(msg),speed=math.hypot(v.linear.x,v.linear.y),w=v.angular.z))

    def scan(self,msg):
        with self.lock: self.scans.append(dict(t=self.now(),stamp=self.stamp(msg)))

    def command(self,msg):
        with self.lock: self.commands.append(dict(t=self.now(),speed=math.hypot(msg.linear.x,msg.linear.y),w=msg.angular.z))


def parameter_snapshot(nav):
    def call(node,service,kind,request):
        client=nav.create_client(kind,f'{node}/{service}')
        if not client.wait_for_service(timeout_sec=5): raise RuntimeError(f'Missing {node}/{service}')
        future=client.call_async(request); began=time.monotonic()
        while not future.done():
            rclpy.spin_once(nav,timeout_sec=.05)
            if time.monotonic()-began>8: raise RuntimeError(f'Parameter timeout: {node}')
        result=future.result();nav.destroy_client(client);return result
    result={}
    for node in ('/controller_server','/amcl','/planner_server','/velocity_smoother','/collision_monitor',
                 '/local_costmap/local_costmap','/global_costmap/global_costmap'):
        names=call(node,'list_parameters',ListParameters,ListParameters.Request()).result.names
        response=call(node,'get_parameters',GetParameters,GetParameters.Request(names=names))
        result[node]={k:parameter_value_to_python(v) for k,v in zip(names,response.values)}
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--condition',choices=CONDITIONS,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    rclpy.init();observer=Observer()
    executor=SingleThreadedExecutor();executor.add_node(observer)
    def spin():
        try: executor.spin()
        except Exception as error: observer.error=repr(error)
    thread=threading.Thread(target=spin,daemon=True);thread.start()
    nav=BasicNavigator();nav.set_parameters([Parameter('use_sim_time',value=True)])
    def pose(x,y):
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=nav.get_clock().now().to_msg()
        p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;return p
    nav.setInitialPose(pose(*START));nav.waitUntilNav2Active()
    began=time.monotonic()
    while not observer.truth or not observer.localized:
        rclpy.spin_once(nav,timeout_sec=.05)
        if time.monotonic()-began>12: raise RuntimeError('Missing independent world or localized pose')
    effective=parameter_snapshot(nav)
    assert effective['/controller_server'][PARAMETER]==TOLERANCES[args.condition]
    (args.output/'effective-parameters-before.json').write_text(json.dumps(effective,indent=2))
    settle=observer.now()
    while observer.now()-settle<1.5:
        rclpy.spin_once(nav,timeout_sec=.05)
        if time.monotonic()-began>30: raise RuntimeError('Pre-goal simulation clock stalled')
    initial=observer.truth[-1]
    assert math.hypot(initial['x']-START[0],initial['y']-START[1])<.05, initial
    assert observer.now()-initial['stamp']<.15
    graph={topic:dict(publishers=[p.node_name for p in observer.get_publishers_info_by_topic(topic)],
                     subscribers=[s.node_name for s in observer.get_subscriptions_info_by_topic(topic)])
           for topic in ('/scan','/rfl04/truth')}
    assert set(graph['/rfl04/truth']['subscribers'])=={'failure_lab_04_observer'}
    (args.output/'READY.json').write_text(json.dumps(dict(t=observer.now(),initial_world_pose=initial,graph=graph),indent=2))
    start=observer.now();wall=time.monotonic()
    (args.output/'GOAL_SUBMITTED.json').write_text(json.dumps(dict(t=start,goal=GOAL)))
    accepted=bool(nav.goToPose(pose(*GOAL)));feedback=[];timed_out=False
    while accepted and not nav.isTaskComplete():
        now=observer.now()
        if observer.error: raise RuntimeError(observer.error)
        f=nav.getFeedback()
        if f:
            p=f.current_pose.pose.position
            feedback.append(dict(t=now,x=p.x,y=p.y,recoveries=int(f.number_of_recoveries)))
        if now-start>=90 or time.monotonic()-wall>=270:
            timed_out=True;nav.cancelTask();break
    end=observer.now()
    result=nav.getResult() if accepted else TaskResult.FAILED
    response=nav.result_future.result() if accepted and nav.result_future.done() else None
    ar=response.result if response else None
    outcome='timeout' if timed_out else ('succeeded' if result==TaskResult.SUCCEEDED else 'failed')
    (args.output/'TASK_TERMINAL.json').write_text(json.dumps(dict(t=end,outcome=outcome)))
    tail=time.monotonic()
    while observer.now()<end+1.8:
        rclpy.spin_once(nav,timeout_sec=.05)
        if observer.error: raise RuntimeError(observer.error)
        if time.monotonic()-tail>20: raise RuntimeError('Post-result observation clock stalled')
    after=parameter_snapshot(nav)
    (args.output/'effective-parameters-after.json').write_text(json.dumps(after,indent=2))
    assert after==effective,'Runtime parameters changed during trial'
    executor.shutdown();thread.join(timeout=3)
    if observer.error: raise RuntimeError(observer.error)
    observation_end=observer.now()
    def save(name,rows,fields):
        with (args.output/name).open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    save('world-pose.csv',observer.truth,['t','stamp','x','y','yaw','frame','child'])
    save('localized-pose.csv',observer.localized,['t','stamp','x','y','yaw'])
    save('odom.csv',observer.odom,['t','stamp','speed','w'])
    save('scans.csv',observer.scans,['t','stamp'])
    save('commands.csv',observer.commands,['t','speed','w'])
    save('feedback.csv',feedback,['t','x','y','recoveries'])
    metrics=endpoint_metrics(observer.truth,observer.localized,start,end,outcome)
    record=dict(condition=args.condition,xy_goal_tolerance_m=TOLERANCES[args.condition],
                start_sim_s=start,end_sim_s=end,observation_end_sim_s=observation_end,
                elapsed_sim_s=end-start,elapsed_wall_s=time.monotonic()-wall,
                outcome=outcome,goal_accepted=accepted,terminal_action_status=getattr(response,'status',None),
                action_error_code=getattr(ar,'error_code',None),action_error_msg=getattr(ar,'error_msg',None),
                initial_world_pose=initial,graph=graph,
                recoveries=max((f['recoveries'] for f in feedback),default=None),metrics=metrics)
    (args.output/'result.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record),flush=True)
    nav.destroy_node();observer.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
