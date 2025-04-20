#!/usr/bin/env python3

import argparse
import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Twist
from astar_sim import Action, Turtlebot3Waffle, State, Map, VisTree, Astar

class velocity_publisher(Node):
    def __init__(self, f, astar:Astar):
        super().__init__('AStar_Node')
        self.timer_period = 1.0/f
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.astar = astar
        self.action_list = astar.retrieve_actions()
        self.msg = Twist()
        
        

    def timer_callback(self):
        # Check whether reach the goal
        if self.i >= len(self.action_list):
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.cmd_vel_pub.publish(self.msg)
            self.get_logger().info('Achieve Goal!')
            del self.astar
            raise SystemExit
                        
        else:
            a:Action = self.action_list[self.i]

            # Compute linear and angular velocity
            ur, ul = a.v_r, a.v_l
            linear_vel = 0.5 * (ur+ul) / 1000
            ang_vel = a.ang_vel
            self.get_logger().info('Action {step:d}|{steps:d}'.format(step=self.i+1, steps=len(self.action_list)))
            self.get_logger().info('Linear Speed: {lin:.2f}, Angular Speed: {ang:.2f}'.format(lin=linear_vel, ang=ang_vel))
            
            # Publish velocities to turtlebot geometry_msgs topic
            self.msg.linear.x = linear_vel
            self.msg.angular.z = ang_vel
            self.cmd_vel_pub.publish(self.msg)
            self.astar.visualize_path(ind=self.i)
            self.i+=1

def get_goal_point(str_goal):
    xy_coord = str_goal.split('_')
    x = int(xy_coord[0])
    y = int(xy_coord[1])
    return (x,y)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dt", type=float, default=5.0,
                        help="step time")
    parser.add_argument("--rpm1", type=float, default=10.0,
                        help="rpm 1")
    parser.add_argument("--rpm2", type=float, default=20.0,
                        help="rpm 2")
    parser.add_argument('--GoalNode', type=str, default='5750_1000', 
                        help='Goal node, Default: 5750_1000')
    args = parser.parse_args()
    
    cogw = 1.00
    dt = args.dt
    rpm1 = args.rpm1
    rpm2 = args.rpm2
    goal_coord = args.GoalNode

    State.xy_res = Turtlebot3Waffle.robot_radius/10.0

    # Create Map Object
    custom_map = Map(inflate_radius=1.5*Turtlebot3Waffle.robot_radius, 
                     width=6000, 
                     height=2000)

    obs_corners = []
    obs_corners.append(custom_map.get_corners_rect(
                                            upper_left=(1500,2000),
                                            w=250,h=1000))
    obs_corners.append(custom_map.get_corners_rect(
                                            upper_left=(2500,1000),
                                            w=250,h=1000))
    obs_corners.append(custom_map.get_corners_circ(
                                            center=(4200,1200),
                                            circle_radius=600,n=30))
    
    
    # add all obstacles to map
    for c in obs_corners:
        custom_map.add_obstacle(corners_tuple=c)

    # get the inflated obstacle corners
    corners = custom_map.get_obstacle_corners_array()

    init_coord = (500,1000)
    goal_coord = get_goal_point(goal_coord)#(5750,1000)
    init_ori = 0

    vt = VisTree(corners=corners,
                 goal_coord=goal_coord,
                 boundary=custom_map.obstacle_boundary_inflate,
                 inflate_coef=cogw)
    
    # Initialize Astar solver
    a = Astar(init_coord=init_coord,
              init_ori=init_ori,
              goal_coord=goal_coord,
              rpms=[rpm1,rpm2],
              wheel_radius=Turtlebot3Waffle.wheel_radius,
              wheel_distance=Turtlebot3Waffle.wheel_distance,
              map=custom_map,
              vis_tree=vt,
              savevid=False,
              vid_res=300,
              dt=dt,)
    
    # run the algorithm
    a.run()

    rclpy.init(args=None)
    node = velocity_publisher(f=1.0/dt, astar=a)
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
      