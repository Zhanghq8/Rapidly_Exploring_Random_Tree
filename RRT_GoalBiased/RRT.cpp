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

bool RRT::RRT::isHit(Vec2i coordinates1_, Vec2i coordinates2_)
{	
	if (Obstacleset.size() == 0)
	{
		return false;
	}
	for (int i=0; i<Obstacleset.size(); i++)
	{	
		Vec2i bottomleft = {Obstacleset[i].bottomleftx, Obstacleset[i].bottomlefty};
		Vec2i bottomright = {Obstacleset[i].bottomleftx + Obstacleset[i].width, Obstacleset[i].bottomlefty};
		Vec2i topleft = {Obstacleset[i].bottomleftx, Obstacleset[i].bottomlefty + Obstacleset[i].height};
		Vec2i topright = {Obstacleset[i].bottomleftx + Obstacleset[i].width, Obstacleset[i].bottomlefty + Obstacleset[i].height};
		bool top = islineinsect(coordinates1_, coordinates2_, topleft, topright);
		bool bottom = islineinsect(coordinates1_, coordinates2_, bottomleft, bottomright);
		bool left = islineinsect(coordinates1_, coordinates2_, topleft, bottomleft);
		bool right = islineinsect(coordinates1_, coordinates2_, topright, bottomright);
		if (top || bottom || left || right)
		{
			return true;
		}
		return false;
	}
}

bool RRT::RRT::islineinsect(Vec2i line1p1_, Vec2i line1p2_, Vec2i line2p1_, Vec2i line2p2_)
{
	// calculate the distance to intersection point
	float uA = ((line2p2_.x-line2p1_.x)*(line1p1_.y-line2p1_.y) - (line2p2_.y-line2p1_.y)*
		(line1p1_.x-line2p1_.x)) / ((line2p2_.y-line2p1_.y)*(line1p2_.x-line1p1_.x) - 
		(line2p2_.x-line2p1_.x)*(line1p2_.y-line1p1_.y));
	float uB = ((line1p2_.x-line1p1_.x)*(line1p1_.y-line2p1_.y) - (line1p2_.y-line1p1_.y)*
		(line1p1_.x-line2p1_.x)) / ((line2p2_.y-line2p1_.y)*(line1p2_.x-line1p1_.x) - 
		(line2p2_.x-line2p1_.x)*(line1p2_.y-line1p1_.y));

	// if uA and uB are between 0-1, lines are colliding
	if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) 
	{
		return true;
  	}
  	return false;
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
		if (coordinates_.x >= Obstacleset[i].bottomleftx
			&& coordinates_.x <= Obstacleset[i].bottomleftx + Obstacleset[i].width
			&& coordinates_.y >= Obstacleset[i].bottomlefty 
			&& coordinates_.y <= Obstacleset[i].bottomlefty + Obstacleset[i].height)
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
		&& closestvertex_.x < map_width && closestvertex_.y < map_height 
		&& isInObstacle(coordinates_) == false && isInObstacle(closestvertex_) == false
		&& isHit(coordinates_, closestvertex_) == false)
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
		std::cout << "[" << path[0].x << "," << path[0].y << "] ";
		for (int i=1; i<path.size(); i++)
		{
			std::cout << "[" << path[i].x << "," << path[i].y << "] ";
			final_cost += euclidean_dis(path[i], path[i-1]); 
		}
		std::cout << "\n";
	}
	std::cout << "Final cost(without smooth): " << final_cost << std::endl;

	smoothpath(goal_);
	float final_cost_s = 0;
	if (!smooth_path.empty()) 
	{
		std::cout << "After smooth, find " <<  smooth_path.size() << " vertices. " << std::endl;
		std::cout << "[" << smooth_path[0].x << "," << smooth_path[0].y << "] ";
		for (int i=1; i<smooth_path.size(); i++)
		{
			std::cout << "[" << smooth_path[i].x << "," << smooth_path[i].y << "] ";
			final_cost_s += euclidean_dis(smooth_path[i], smooth_path[i-1]); 
		}
		std::cout << "\n";
	}
	std::cout << "Final cost(after smooth): " << final_cost_s << std::endl;
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

void RRT::RRT::smoothpath(Vec2i goal_)
{	
	if (path.size() <= 2) {
		smooth_path = path;
		return;
	}
	smooth_path.push_back(path[0]);
	int index1 = 0;
	int index2 = 1;
	while (index1 <= path.size() && index2 <= path.size())
	{
		if (isValid(path[index1], path[index2]) == true)
		{
			index2++;
		}
		else
		{	
			if (index2 - 1 != 0)
			{
				smooth_path.push_back(path[index2-1]);
			}
			index1 = index2;
			index2 = index1 + 1;
		}
	}
	if (smooth_path.back().x != goal_.x && smooth_path.back().y != goal_.y)
	{
		smooth_path.push_back(path.back());
	}
}

int main()
{
	RRT::RRT temp;
	temp.setmap(50, 50);
	temp.setstepsize(3.0);
	temp.setgoalbias(0.07);
	temp.setrandompointsize(5.0);
	temp.setgoalradius(1.0);
	temp.setmaxiterations(10000);
	RRT::Vec2i start, goal;
	start.x = 10.0;
	start.y = 10.0;
	goal.x = 30.0;
	goal.y = 30.0;
	RRT::Rectobstacle obstacle1{10,10,15,20};
	temp.addobstacle(obstacle1);
	// std::cout << "obstacle: " << temp.Obstacleset[0].topleftx << " " << temp.Obstacleset[0].toplefty << std::endl;

	temp.findPath(start, goal);
}
