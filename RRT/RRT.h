#ifndef _RRT_H_
#define _RRT_H_

#include <vector>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <set>
#include <limits>

namespace RRT
{	
	struct Vec2i
	{
		float x, y;
	};

	struct Vertex
	{
		Vec2i coordinates;
		Vertex* parent;
		Vertex(Vec2i coordinates, Vertex *parent = NULL);
	};
	struct Rectobstacle
	{
		float width;
		float height;
		float topleftx;
		float toplefty;
	};

	class RRT
	{
	private:
		float map_width;
		float map_height;
		float randompoint_size; //generate new random point from current to next random point 
		float step_size; //generate new point step_size from closest vertex towards random point
		int delta;		
		int max_iterations;
		float goal_bias; //with goal_bias% probability to generate goal point 
		float goal_radius; 
		Vertex* current;
		std::vector<Vec2i> path; 
		std::vector<Rectobstacle> Obstacleset;
		std::set<Vertex*> VertexSet;

	public:
		void setmap(float map_width_, float map_height_);
		void setstepsize(float step_size_);
		void setgoalbias(float goal_bias_);
		void addobstacle(Rectobstacle obstacle_);
		void setmaxiterations(int max_iterations_);
		void setgoalradius(float goal_radius_);
		void setrandompointsize(float randompoint_size_);
		bool isInObstacle(const Vec2i& coordinates_);
		bool isGoal(Vec2i source_, Vec2i goal_); // check if the coordinate is at goal pos
		bool isValid(Vec2i coordinates_, Vec2i closestvertex_); // check if the coordinate is valid
		float euclidean_dis(Vec2i source_, Vec2i goal_); // calculate the euclidean distance from current to goal
		Vec2i GenerateRandomPoint(Vec2i goal_); 
		Vertex* getClosestVertex(std::set<Vertex*>& Vertices_, Vec2i randompoint_);
		bool movetorandom(Vertex* closertvertex_, Vec2i randompoint_);
		void findPath(Vec2i source_, Vec2i goal_);
		void releaseVertices(std::set<Vertex*>& Vertices_);

	};
}

#endif