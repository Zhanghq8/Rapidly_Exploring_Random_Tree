#include "RRT.h"


RRT::Vertex::Vertex(Vec2i coordinates_, Vertex *parent_)
{
	coordinates = coordinates_;
	parent = parent_;
}

void RRT::RRT::setmap(float map_width_, float map_height_)
{
	map_width = map_width_;
	map_height = map_height_;
}

void RRT::RRT::setgoalbias(float goal_bias_)
{
	goal_bias = goal_bias_;
}

void RRT::RRT::setstepsize(float step_size_)
{
	step_size = step_size_;
}

void RRT::RRT::setmaxiterations(int max_iterations_)
{
	max_iterations = max_iterations_;
}

void RRT::RRT::setgoalradius(float goal_radius_)
{
	goal_radius = goal_radius_;
}

void RRT::RRT::setrandompointsize(float randompoint_size_)
{
	randompoint_size = randompoint_size_;
}

bool RRT::RRT::isGoal(Vec2i source_, Vec2i goal_) // check if the coordinate is at goal pos
{
	float distance = euclidean_dis(source_, goal_);
	if (distance <= goal_radius) 
	{
		return true;
	}
	return false;
}

bool RRT::RRT::isValid(Vec2i coordinates_, Vec2i closestvertex_) // check if the coordinate is valid
{
	if (coordinates_.x > 0 && coordinates_.y > 0 && closestvertex_.x < map_width && closestvertex_.y < map_height)
	{
		return true;
	}
	return false;
}

float RRT::RRT::euclidean_dis(Vec2i source_, Vec2i goal_) // calculate the euclidean distance from current to goal
{
	float e_distance = sqrt(pow(source_.x - goal_.x, 2) + pow(source_.y - goal_.y, 2));
	return e_distance; 
}

RRT::Vec2i RRT::RRT::GenerateRandomPoint(Vec2i goal_)
{
	Vec2i randompoint;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> x(0, randompoint_size);
	std::uniform_real_distribution<> y(0, randompoint_size);

	randompoint.x = x(gen);
	randompoint.y = y(gen);

	bool setgoal = (rand() % 100 + 1) <= 7;
	if (setgoal == true )
	{
		return goal_;
	}
	return randompoint;
}

RRT::Vertex* RRT::RRT::getClosestVertex(std::set<Vertex*>& Vertices_, Vec2i randompoint_)
{	
	Vertex* closestvertex = NULL;
	float min_distance = std::numeric_limits<float>::max();
	for (auto vertex:Vertices_) {
		if (euclidean_dis(vertex->coordinates, randompoint_) < min_distance) {
			min_distance = euclidean_dis(vertex->coordinates, randompoint_);
			closestvertex = vertex;
		}
	}
	return closestvertex;

}

bool RRT::RRT::movetorandom(Vertex* closestvertex_, Vec2i randompoint_)
{
	float theta = atan2(randompoint_.y - closestvertex_->coordinates.y, randompoint_.x - closestvertex_->coordinates.x);
	Vec2i vertextemp;
	vertextemp.x = closestvertex_->coordinates.x + step_size * cos(theta);
	vertextemp.y = closestvertex_->coordinates.y + step_size * sin(theta);
	std::cout << "vertextemp: " << vertextemp.x << " " << vertextemp.y << std::endl;
	if (isValid(vertextemp, closestvertex_->coordinates) == true) 
	{	
		Vertex* newvertex = new Vertex(vertextemp, closestvertex_);
		current = newvertex;
		VertexSet.insert(newvertex);
		return true;
	}
	return false;
}

void RRT::RRT::findPath(Vec2i source_, Vec2i goal_)
{	
	lastvertex = NULL;
	bool done_flag = false;
	VertexSet.insert(new Vertex(source_));
	current = *VertexSet.begin();
	// std::cout << "path root " << current->parent << std::endl;
	int current_iterations = 0;
	while (done_flag != true && current_iterations <= max_iterations) 
	{	
		std::cout << current_iterations << std::endl;
		Vec2i randompoint = GenerateRandomPoint(goal_);
		// std::cout << "random point " << randompoint.x << std::endl;
		Vertex* closestv= getClosestVertex(VertexSet, randompoint);
		std::cout << "MOVE" << movetorandom(closestv, randompoint) << std::endl;
		if (movetorandom(closestv, randompoint) == true)
		{
			current_iterations++;
			if (isGoal(current->coordinates, goal_) == true)
			{
				done_flag = true;
				Vertex* goalvertex = new Vertex(goal_, current);
				current = goalvertex;
				std::cout << "Found a path." << std::endl;
			}
		}
		if (current_iterations == max_iterations)
		{
			std::cout << "No path found." << std::endl;
		}
	}
	
	while (current != NULL) 
	{
		path.push_back(current->coordinates);
		current = current->parent;
	}
	reverse(path.begin(), path.end());
	if (!path.empty()) 
	{
		std::cout << "path size" <<  path.size() << std::endl;
		for (auto ele:path)
		{
			std::cout << "[" << ele.x << "," << ele.y << "] "; 
		}
		std::cout << "\n";
	}
	releaseVertices(VertexSet);
}

void RRT::RRT::releaseVertices(std::set<Vertex*>& Vertices_)
{
	for (auto it = Vertices_.begin(); it != Vertices_.end();) 
	{
		delete *it;
		it = Vertices_.erase(it);
	}
}

int main()
{
	RRT::RRT temp;
	temp.setmap(30.0, 30.0);
	temp.setstepsize(3.0);
	temp.setgoalbias(0.07);
	temp.setrandompointsize(5.0);
	temp.setgoalradius(1.0);
	temp.setmaxiterations(10000);
	RRT::Vec2i start, goal;
	start.x = 10.0;
	start.y = 10.0;
	goal.x = 25.0;
	goal.y = 25.0;
	temp.findPath(start, goal);
}
