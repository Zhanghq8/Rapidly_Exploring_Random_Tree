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
		float map_width;
		float map_height; // map size
		float near_radius;
		float randompoint_size; //generate new random point from current to next random point 
		float step_size; //generate new point step_size from closest vertex towards random point
		int delta;		
		int max_iterations;
		float goal_bias; //with goal_bias% probability to generate goal point 
		float goal_radius; 
		Vertex* current; // keep track of current vertex
		std::vector<Vec2i> path; // store the coordinate of the path
		std::vector<Vec2i> smooth_path; // store the coordinate of the path after smoothed
		std::vector<Rectobstacle> Obstacleset; 
		std::set<Vertex*> VertexSet; // store all the vertice visited
		

	public:
		void setmap(float map_width_, float map_height_);
		void setstepsize(float step_size_);
		void setgoalbias(float goal_bias_=7);
		void setnearradius(float near_radius_=2.5);
		void addobstacle(Rectobstacle obstacle_);
		void setmaxiterations(int max_iterations_);
		void setgoalradius(float goal_radius_);
		void setrandompointsize(float randompoint_size_);
		void minsmoothpath(Vec2i goal_);
		void randomsmoothpath();
		void exportpath();
		bool isHit(Vec2i coordinates1_, Vec2i coordinates2_);
		bool islineinsect(Vec2i line1p1, Vec2i line1p2, Vec2i line2p1, Vec2i line2p2);
		bool isInObstacle(const Vec2i& coordinates_);
		bool isGoal(Vec2i source_, Vec2i goal_); // check if the coordinate is at goal pos
		bool isValid(Vec2i coordinates_, Vec2i closestvertex_); // check if the coordinate is valid
		float euclidean_dis(Vec2i source_, Vec2i goal_); // calculate the euclidean distance from current to goal
		Vec2i GenerateRandomPoint(Vec2i goal_); 
		Vertex* getClosestVertex(std::set<Vertex*>& Vertices_, Vec2i randompoint_);
		bool extend(Vertex* closertvertex_, Vec2i randompoint_);
		Vertex* rewire(std::set<Vertex*>& Vertices_, Vec2i newvertex_);
		void findPath(Vec2i source_, Vec2i goal_);
		void releaseVertices(std::set<Vertex*>& Vertices_);

	};
}

#endif