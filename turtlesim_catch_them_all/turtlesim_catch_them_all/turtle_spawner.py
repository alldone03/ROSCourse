#!/usr/bin/env python3
from functools import partial
import rclpy
import random 
import math
from rclpy.node import Node
from turtlesim.srv import Spawn
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray



class TurtleSpawner(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("turtle_spawner") # MODIFY NAME
        self.turtle_name_prefix_ = "turtle"
        self.turtle_counter_ = 0
        self.alive_turtles_ = []
        self.spawn_turtle_timer_ = self.create_timer(2.0,self.spawn_new_turtle)
        self.alive_turtle_publisher_ = self.create_publisher(TurtleArray,"alive_turtles",10)

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtle_publisher_.publish(msg=msg)
        
    def spawn_new_turtle(self):
        self.turtle_counter_ +=1 
        name = str(self.turtle_name_prefix_ )+ str(self.turtle_counter_)
        x = random.uniform(0.0,11.0)
        y = random.uniform(0.0,11.0)
        theta = random.uniform(0.0,2*math.pi)
        self.call_spawn_server(name,x,y,theta)
        

    def call_spawn_server(self, turtle_name,x,y,theta):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server ...")
            
        request = Spawn.Request()
        request.x = x
        request.y = y 
        request.theta = theta
        request.name = turtle_name 

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn, 
                    turtle_name=turtle_name,
                    x=x,
                    y=y,
                    theta=theta,
                    ))

    def callback_call_spawn(self, future, turtle_name,x,y,theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info("turtle "+ response.name + " is now alive ")
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                self.alive_turtles_.append(new_turtle) 
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

        
        
        
    

        


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
