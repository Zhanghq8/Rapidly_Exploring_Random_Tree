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

void RRT::RRT::addobstacle(Rectobstacle obstacle_)
{
	Obstacleset.push_back(obstacle_);
}

// check if the coordinate is in all the rectangular obstacles
bool RRT::RRT::isInObstacle(const Vec2i& coordinates_)
{
	if (Obstacleset.size() == 0)
	{
		return false;
	}
	for (int i=0; i<Obstacleset.size(); i++)
	{
		if (coordinates_.x >= Obstacleset[i].topleftx
			&& coordinates_.x <= Obstacleset[i].topleftx + Obstacleset[i].width
			&& coordinates_.y >= Obstacleset[i].toplefty - Obstacleset[i].height
			&& coordinates_.y <= Obstacleset[i].toplefty)
		{
			return true;
		}
	}
	return false;
}

// check if the coordinate is at goal pos
bool RRT::RRT::isGoal(Vec2i source_, Vec2i goal_) 
{
	float distance = euclidean_dis(source_, goal_);
	if (distance <= goal_radius) 
	{
		return true;
	}
	return false;
}

// check if the coordinate is valid

bool RRT::RRT::isValid(Vec2i coordinates_, Vec2i closestvertex_) 
{
	if (coordinates_.x > 0 && coordinates_.y > 0 
		&& closestvertex_.x < map_width 
		&& closestvertex_.y < map_height && isInObstacle(coordinates_) == false && isInObstacle(closestvertex_) == false)
	{
		return true;
	}
	return false;
}

// calculate the euclidean distance from current to goal
float RRT::RRT::euclidean_dis(Vec2i source_, Vec2i goal_) 
{
	float e_distance = sqrt(pow(source_.x - goal_.x, 2) + pow(source_.y - goal_.y, 2));
	return e_distance; 
}

//generate new randompoint with goal_bias% probability to pick goal point 
RRT::Vec2i RRT::RRT::GenerateRandomPoint(Vec2i goal_)
{
	Vec2i randompoint;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> x(0, map_width);
	std::uniform_real_distribution<> y(0, map_height);

	randompoint.x = x(gen);
	randompoint.y = y(gen);
	// std::cout << "randomx: " << randompoint.x << std::endl;
	// std::cout << "randomy: " << randompoint.y << std::endl;
	bool setgoal = (rand() % 100 + 1) <= goal_bias;
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

//generate new point along the line contain closestvertex and randompoint
//check if the new point is valid 
bool RRT::RRT::extend(Vertex* closestvertex_, Vec2i randompoint_)
{
	float theta = atan2(randompoint_.y - closestvertex_->coordinates.y, randompoint_.x - closestvertex_->coordinates.x);
	// std::cout << "theta: " << theta << std::endl;
	Vec2i vertextemp;
	vertextemp.x = closestvertex_->coordinates.x + step_size * cos(theta);
	vertextemp.y = closestvertex_->coordinates.y + step_size * sin(theta);
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
	bool done_flag = false;
	VertexSet.insert(new Vertex(source_));
	current = *VertexSet.begin();
	int current_iterations = 0;
	while (done_flag != true && current_iterations < max_iterations) 
	{	
		// std::cout << current_iterations << std::endl;
		Vec2i randompoint = GenerateRandomPoint(goal_);
		Vertex* closestv= getClosestVertex(VertexSet, randompoint);
		
		if (extend(closestv, randompoint) == true)
		{
			current_iterations++;
			if (isGoal(current->coordinates, goal_) == true)
			{
				done_flag = true;
				Vertex* goalvertex = new Vertex(goal_, current);
				current = goalvertex;
				std::cout << "Found a path ";
			}
		}
		if (current_iterations == max_iterations)
		{
			std::cout << "No path found." << std::endl;
			current = NULL;
			releaseVertices(VertexSet);
			return;
		}
	}
	
	while (current != NULL) 
	{
		path.push_back(current->coordinates);
		current = current->parent;
	}
	reverse(path.begin(), path.end());
	float final_cost = 0;
	if (!path.empty()) 
	{
		std::cout << "with " <<  path.size() << " vertices. " << std::endl;
		for (int i=1; i<path.size()-1; i++)
		{
			std::cout << "[" << path[i].x << "," << path[i].y << "] ";
			final_cost += euclidean_dis(path[i], path[i-1]); 
		}
		std::cout << "\n";
	}
	std::cout << "Final cost: " << final_cost << std::endl;
	releaseVertices(VertexSet);
}

void RRT::RRT::releaseVertices(std::set<Vertex*>& Vertices_)
{	
	std::cout << "Visited vertices: " << Vertices_.size() << std::endl;
	for (auto it = Vertices_.begin(); it != Vertices_.end();) 
	{
		delete *it;
		it = Vertices_.erase(it);
	}
}

int main()
{
	RRT::RRT temp;
	temp.setmap(50, 50);
	temp.setstepsize(2.0);
	temp.setgoalbias(0.07);
	temp.setrandompointsize(5.0);
	temp.setgoalradius(1.0);
	temp.setmaxiterations(1000);
	RRT::Vec2i start, goal;
	start.x = 10.0;
	start.y = 10.0;
	goal.x = 20.0;
	goal.y = 20.0;
	// RRT::Rectobstacle obstacle1{5,30,15,30};
	// temp.addobstacle(obstacle1);
	// std::cout << "obstacle: " << temp.Obstacleset[0].topleftx << " " << temp.Obstacleset[0].toplefty << std::endl;

	temp.findPath(start, goal);
}
