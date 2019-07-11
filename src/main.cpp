#include <stdio.h>
#include <iostream>
#include "../include/rrt_visualization/RRTStar.h"
#include "../include/rrt_visualization/RRT.h"
#include "../include/rrt_visualization/Bi_RRT.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

int main( int argc, char** argv )
{
	ros::init(argc, argv, "rrt_visualization");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	ros::Rate r(30);

	while (ros::ok())
	{
		RRTStar::RRTStar rrt;

		ros::Duration(1).sleep();

		// map
		visualization_msgs::Marker map_boundary;
		map_boundary.header.frame_id = "/my_frame";
		map_boundary.header.stamp = ros::Time::now();
		map_boundary.action = visualization_msgs::Marker::ADD;
		map_boundary.ns = "map";
		map_boundary.id = 0;
		map_boundary.type = visualization_msgs::Marker::LINE_STRIP;
		map_boundary.pose.orientation.w = 1.0;
		map_boundary.scale.x = 0.2;
		map_boundary.color.a = 1.0;

		// map_boundary.points.clear();


		// obstacle
		visualization_msgs::Marker obstacle_line;
		obstacle_line.header.frame_id = "/my_frame";
		obstacle_line.header.stamp = ros::Time::now();
		obstacle_line.action = visualization_msgs::Marker::ADD;
		obstacle_line.ns = "map";
		obstacle_line.pose.orientation.w = 1.0;

		obstacle_line.id = 2;
		obstacle_line.type = visualization_msgs::Marker::LINE_LIST;
		
		obstacle_line.scale.x = 0.2;
		obstacle_line.color.a = 1.0;

		// obstacle_boundary.points.clear();

		// start and goal point
		visualization_msgs::Marker start_point, goal_point;
		start_point.header.frame_id = goal_point.header.frame_id = "/my_frame";
		start_point.header.stamp = goal_point.header.stamp = ros::Time::now();
		start_point.ns = goal_point.ns = "map";
		start_point.action = goal_point.action = visualization_msgs::Marker::ADD;
		start_point.pose.orientation.w = goal_point.pose.orientation.w = 1.0;
		start_point.type = goal_point.type = visualization_msgs::Marker::POINTS;

		start_point.id = 3;
		goal_point.id = 4;

		start_point.scale.x = 0.4;
		start_point.scale.y = 0.4;

		goal_point.scale.x = 0.4;
		goal_point.scale.y = 0.4;

		start_point.color.r = 1.0;
		start_point.color.g = 1.0;
		start_point.color.a = 1.0;

		goal_point.color.g = 1.0;
		goal_point.color.b = 1.0;
		goal_point.color.a = 1.0;

		// algorithm
		visualization_msgs::Marker visited_points, path_points, path_line;
		visited_points.header.frame_id = path_points.header.frame_id = path_line.header.frame_id = "/my_frame";
		visited_points.header.stamp = path_points.header.stamp = path_line.header.stamp = ros::Time::now();
		visited_points.ns = path_points.ns = path_line.ns = "map";
		visited_points.action = path_points.action = path_line.action = visualization_msgs::Marker::ADD;
		visited_points.pose.orientation.w = path_points.pose.orientation.w = path_line.pose.orientation.w = 1.0;

		visited_points.id = 5;
		path_points.id = 6;
		path_line.id = 7;

		visited_points.type = visualization_msgs::Marker::POINTS;
		path_points.type = visualization_msgs::Marker::POINTS;
		path_line.type = visualization_msgs::Marker::LINE_STRIP;
		// line_list.type = visualization_msgs::Marker::LINE_LIST;



		// POINTS markers use x and y scale for width/height respectively
		visited_points.scale.x = 0.2;
		visited_points.scale.y = 0.2;

		path_points.scale.x = 0.2;
		path_points.scale.y = 0.2;

		path_line.scale.x = 0.1;

		// Points are green
		visited_points.color.g = 1.0f;
		visited_points.color.a = 1.0;

		path_points.color.b = 1.0;
		path_points.color.a = 1.0;

		path_line.color.r = 1.0;
		path_line.color.a = 1.0;

		marker_pub.publish(map_boundary);
		marker_pub.publish(obstacle_line);
		marker_pub.publish(start_point);
		marker_pub.publish(goal_point);
		marker_pub.publish(visited_points);
		marker_pub.publish(path_points);
		marker_pub.publish(path_line);


		geometry_msgs::Point p_map;
		p_map.x = 0.0;
		p_map.y = 0.0;
		map_boundary.points.push_back(p_map);
		p_map.x = rrt.map_width;
		p_map.y = 0.0;
		map_boundary.points.push_back(p_map);
		p_map.x = rrt.map_width;
		p_map.y = rrt.map_height;
		map_boundary.points.push_back(p_map);
		p_map.x = 0.0;
		p_map.y = rrt.map_height;
		map_boundary.points.push_back(p_map);
		p_map.x = 0.0;
		p_map.y = 0.0;
		map_boundary.points.push_back(p_map);
		marker_pub.publish(map_boundary);
		ros::Duration(1).sleep();


		for (int i=0; i<rrt.Obstacleset.size(); i++)
		{	
			geometry_msgs::Point p_obstacle1;
			p_obstacle1.x = rrt.Obstacleset[i].bottomleftx;
			p_obstacle1.y = rrt.Obstacleset[i].bottomlefty;
			// obstacle_boundary.points.push_back(p_obstacle1);
			geometry_msgs::Point p_obstacle2;
			p_obstacle2.x = rrt.Obstacleset[i].bottomleftx + rrt.Obstacleset[i].width;
			p_obstacle2.y = rrt.Obstacleset[i].bottomlefty;
			// obstacle_boundary.points.push_back(p_obstacle2);
			geometry_msgs::Point p_obstacle3;
			p_obstacle3.x = rrt.Obstacleset[i].bottomleftx + rrt.Obstacleset[i].width;
			p_obstacle3.y = rrt.Obstacleset[i].bottomlefty + rrt.Obstacleset[i].height;
			// obstacle_boundary.points.push_back(p_obstacle3);
			geometry_msgs::Point p_obstacle4;
			p_obstacle4.x = rrt.Obstacleset[i].bottomleftx;
			p_obstacle4.y = rrt.Obstacleset[i].bottomlefty + rrt.Obstacleset[i].height;
			// obstacle_boundary.points.push_back(p_obstacle4);
			obstacle_line.points.push_back(p_obstacle1);
			obstacle_line.points.push_back(p_obstacle2);
			marker_pub.publish(obstacle_line);
			obstacle_line.points.push_back(p_obstacle2);
			obstacle_line.points.push_back(p_obstacle3);
			marker_pub.publish(obstacle_line);
			obstacle_line.points.push_back(p_obstacle3);
			obstacle_line.points.push_back(p_obstacle4);
			marker_pub.publish(obstacle_line);
			obstacle_line.points.push_back(p_obstacle4);
			obstacle_line.points.push_back(p_obstacle1);
			marker_pub.publish(obstacle_line);

			// ros::Duration(1).sleep();
		}

		ros::Duration(1).sleep();




		geometry_msgs::Point p_start;
		p_start.x = rrt.path[0].x;
		p_start.y = rrt.path[0].y;
		p_start.z = 0;

		start_point.points.push_back(p_start);

		geometry_msgs::Point p_goal;
		p_goal.x = rrt.path[rrt.path.size()-1].x;
		p_goal.y = rrt.path[rrt.path.size()-1].y;
		p_goal.z = 0;

		start_point.points.push_back(p_goal);

		marker_pub.publish(start_point);
		marker_pub.publish(goal_point);
		ros::Duration(1).sleep();


		// Create the vertices for the points and lines
		for (int i = 0; i < rrt.visitednode.size(); ++i)
		{

			geometry_msgs::Point p_visited;
			p_visited.x = rrt.visitednode[i].x;
			p_visited.y = rrt.visitednode[i].y;
			p_visited.z = 0;

			visited_points.points.push_back(p_visited);
			marker_pub.publish(visited_points);
			ros::Duration(0.05).sleep();
			// line_strip.points.push_back(p);

			// // The line list needs two points for each line
			// line_list.points.push_back(p);
			// p.z += 1.0;
			// line_list.points.push_back(p);
		}
		// marker_pub.publish(visited_points);
		ros::Duration(1).sleep();


		for (int i = 0; i < rrt.path.size(); ++i)
		{

			geometry_msgs::Point p_path;
			p_path.x = rrt.path[i].x;
			p_path.y = rrt.path[i].y;
			p_path.z = 0;

			path_points.points.push_back(p_path);
			path_line.points.push_back(p_path);
			// // The line list needs two points for each line
			// line_list.points.push_back(p);
			// p.z += 1.0;
			// line_list.points.push_back(p);
		}
		marker_pub.publish(path_points);
		marker_pub.publish(path_line);
		ros::Duration(3).sleep();
		// visited_points.action = visualization_msgs::Marker::DELETEALL;
		// path_points.action = visualization_msgs::Marker::DELETEALL;
		// path_line.action = visualization_msgs::Marker::DELETEALL;

		// marker_pub.publish(line_list);

		ros::Duration(1).sleep();
		ros::spinOnce();
	}
}