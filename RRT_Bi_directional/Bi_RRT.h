#ifndef _BI_RRT_H_
#define _BI_RRT_H_

#include <vector>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <set>
#include <limits>

namespace RRT
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
		Vertex(Vec2i coordinates, Vertex *parent = NULL); 
	};

	struct Rectobstacle
	{
		float width;
		float height;
		float bottomleftx;
		float bottomlefty; // coordinate of topleft point
	};

	class Bi_RRT
	{
	private:
		float map_width;
		float map_height; // map size
		float randompoint_size; //generate new random point from current to next random point 
		float step_size; //generate new point step_size from closest vertex towards random point
		int delta;		
		int max_iterations;
		float goal_bias; //with goal_bias% probability to generate goal point 
		float goal_radius; 
		bool searchA;
		Vertex* current_A; // keep track of current vertex of tree A
		Vertex* current_B; // keep track of current vertex of tree B
		std::vector<Vec2i> path; // store the coordinate of the path
		std::vector<Rectobstacle> Obstacleset; 
		std::set<Vertex*> VertexSetA; // store all the vertice visited for tree A
		std::set<Vertex*> VertexSetB; // store all the vertice visited for tree B

	public:
		void setmap(float map_width_, float map_height_);
		void setstepsize(float step_size_);
		void setgoalbias(float goal_bias_=7);
		void addobstacle(Rectobstacle obstacle_);
		void setmaxiterations(int max_iterations_);
		void setgoalradius(float goal_radius_);
		bool isHit(Vec2i coordinates1_, Vec2i coordinates2_);
		bool islineinsect(Vec2i line1p1, Vec2i line1p2, Vec2i line2p1, Vec2i line2p2);
		void setsearchflag(); // if set true, search tree A first
		void setrandompointsize(float randompoint_size_);
		bool isInObstacle(const Vec2i& coordinates_);
		bool isGoal(Vec2i source_, Vec2i goal_); // check if the coordinate is at goal pos
		bool isValid(Vec2i coordinates_, Vec2i closestvertex_); // check if the coordinate is valid
		float euclidean_dis(Vec2i source_, Vec2i goal_); // calculate the euclidean distance from current to goal
		Vec2i GenerateRandomPoint(Vec2i goal_); 
		Vertex* getClosestVertex(std::set<Vertex*>& Vertices_, Vec2i randompoint_);
		bool extend(Vertex* closertvertex_, Vec2i randompoint_);
		void findPath(Vec2i source_, Vec2i goal_);
		void releaseVertices(std::set<Vertex*>& Vertices_);

	};
}

#endif