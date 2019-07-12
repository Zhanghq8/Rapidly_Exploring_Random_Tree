#ifndef _RRTSTAR_H_
#define _RRTSTAR_H_

#include <vector>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <set>
#include <limits>
#include <fstream>
#include <ostream>

namespace RRTStar
{	
	// coordinate
	struct Vec2i
	{
		float x, y;
	};

	struct Vertex
	{
		Vec2i coordinates;
		Vertex* parent; // address of previous vertex
		float cost;
		Vertex(Vec2i coordinates, Vertex *parent = NULL, float cost = 0); 
	};

	struct Rectobstacle
	{
		float width;
		float height;
		float bottomleftx;
		float bottomlefty; // coordinate of topleft point
	};

	class RRTStar
	{
	private:
		float near_radius;
		float randompoint_size; //generate new random point from current to next random point 
		float step_size; //generate new point step_size from closest vertex towards random point
		int delta;		
		int max_iterations;
		float goal_bias; //with goal_bias% probability to generate goal point 
		float goal_radius; 
		Vertex* current; // keep track of current vertex
		Vertex* goal;
		bool reach_goal;
		std::vector<Vec2i> smooth_path; // store the coordinate of the path after smoothed
		std::set<Vertex*> VertexSet; // store all the vertice visited
		

	public:
		RRTStar();
		float map_width;
		float map_height; // map size
		std::vector<Vec2i> visitednode;
		std::vector<Vec2i> path; // store the coordinate of the path
		std::vector<Rectobstacle> Obstacleset;
		// Set the width and height of map
		void setmap(float map_width_, float map_height_);
		// Set the stepsize to connect from current point to randompoint
		void setstepsize(float step_size_);
		// Set the probability to generate goal point instead of a random point
		void setgoalbias(float goal_bias_=7);
		// Set the near radius when rewiring
		void setnearradius(float near_radius_=2.5);
		// Add rectangular obstacle
		void addobstacle(Rectobstacle obstacle_);
		// Set the maximum iterations for RRT algorithm
		void setmaxiterations(int max_iterations_);
		// Set the bias to check if it is close to goal point
		void setgoalradius(float goal_radius_);
		// Set the stepsize to generate random point 
		void setrandompointsize(float randompoint_size_);
		void minsmoothpath(Vec2i goal_);
		void randomsmoothpath();
		void exportpath();
		// Check if the line between two points hit the obstacle
		bool isHit(Vec2i coordinates1_, Vec2i coordinates2_);
		// Check if two lines have intersecttion 
		bool islineintersect(Vec2i line1p1, Vec2i line1p2, Vec2i line2p1, Vec2i line2p2);
		// Check if the point is within the obstacle
		bool isInObstacle(const Vec2i& coordinates_);
		bool isGoal(Vec2i source_, Vec2i goal_); // check if the coordinate is at goal pos
		bool isValid(Vec2i coordinates_, Vec2i closestvertex_); // check if the coordinate is valid
		float euclidean_dis(Vec2i source_, Vec2i goal_); // calculate the euclidean distance from current to goal
		Vec2i GenerateRandomPoint(Vec2i goal_); 
		Vertex* getClosestVertex(std::set<Vertex*>& Vertices_, Vec2i randompoint_);
		bool extend(Vertex* closertvertex_, Vec2i randompoint_, Vec2i goal_);
		void rewire(Vertex* closestvertex_, Vec2i newvertex_, Vec2i goal_);
		void findPath(Vec2i source_, Vec2i goal_);
		void releaseVertices(std::set<Vertex*>& Vertices_);

	};
}

#endif