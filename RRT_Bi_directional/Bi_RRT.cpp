#include "Bi_RRT.h"


RRT::Vertex::Vertex(Vec2i coordinates_, Vertex *parent_)
{
	coordinates = coordinates_;
	parent = parent_;
}

void RRT::Bi_RRT::setmap(float map_width_, float map_height_)
{
	map_width = map_width_;
	map_height = map_height_;
}

void RRT::Bi_RRT::setgoalbias(float goal_bias_)
{
	goal_bias = goal_bias_;
}

void RRT::Bi_RRT::setstepsize(float step_size_)
{
	step_size = step_size_;
}

void RRT::Bi_RRT::setmaxiterations(int max_iterations_)
{
	max_iterations = max_iterations_;
}

void RRT::Bi_RRT::setgoalradius(float goal_radius_)
{
	goal_radius = goal_radius_;
}

void RRT::Bi_RRT::setrandompointsize(float randompoint_size_)
{
	randompoint_size = randompoint_size_;
}

void RRT::Bi_RRT::setsearchflag()
{
	searchA = true;
}

void RRT::Bi_RRT::addobstacle(Rectobstacle obstacle_)
{
	Obstacleset.push_back(obstacle_);
}

// check if the coordinate is in all the rectangular obstacles
bool RRT::Bi_RRT::isInObstacle(const Vec2i& coordinates_)
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
bool RRT::Bi_RRT::isGoal(Vec2i source_, Vec2i goal_) 
{
	float distance = euclidean_dis(source_, goal_);
	if (distance <= goal_radius) 
	{
		return true;
	}
	return false;
}

// check if the coordinate is valid

bool RRT::Bi_RRT::isValid(Vec2i coordinates_, Vec2i closestvertex_) 
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
float RRT::Bi_RRT::euclidean_dis(Vec2i source_, Vec2i goal_) 
{
	float e_distance = sqrt(pow(source_.x - goal_.x, 2) + pow(source_.y - goal_.y, 2));
	return e_distance; 
}

//generate new randompoint with goal_bias% probability to pick goal point 
RRT::Vec2i RRT::Bi_RRT::GenerateRandomPoint(Vec2i goal_)
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

RRT::Vertex* RRT::Bi_RRT::getClosestVertex(std::set<Vertex*>& Vertices_, Vec2i randompoint_)
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
bool RRT::Bi_RRT::extend(Vertex* closestvertex_, Vec2i randompoint_)
{
	float theta = atan2(randompoint_.y - closestvertex_->coordinates.y, randompoint_.x - closestvertex_->coordinates.x);
	// std::cout << "theta: " << theta << std::endl;
	Vec2i vertextemp;
	vertextemp.x = closestvertex_->coordinates.x + step_size * cos(theta);
	vertextemp.y = closestvertex_->coordinates.y + step_size * sin(theta);
	if (isValid(vertextemp, closestvertex_->coordinates) == true) 
	{	
		Vertex* newvertex = new Vertex(vertextemp, closestvertex_);
		if (searchA == true)
		{
			current_A = newvertex;
			VertexSetA.insert(newvertex);
		}
		else
		{
			current_B = newvertex;
			VertexSetB.insert(newvertex);
		}
		return true;
	}
	return false;
}

void RRT::Bi_RRT::findPath(Vec2i source_, Vec2i goal_)
{	
	bool done_flag = false;
	VertexSetA.insert(new Vertex(source_));
	current_A = *VertexSetA.begin();
	VertexSetB.insert(new Vertex(goal_));
	current_B = *VertexSetB.begin();
	int current_iterations = 0;
	while (done_flag != true && current_iterations < max_iterations) 
	{	
		// std::cout << current_iterations << std::endl;
		std::cout << " searchA: " << searchA;
		Vec2i randompoint;
		Vertex* closestv;
		if (searchA == true)
		{
			randompoint = GenerateRandomPoint(goal_);
			closestv= getClosestVertex(VertexSetA, randompoint);
		}
		else 
		{
			randompoint = GenerateRandomPoint(source_);
			closestv= getClosestVertex(VertexSetB, randompoint);
		}
		std::cout << " extend: " << extend(closestv, randompoint) << std::endl;
		if (extend(closestv, randompoint) == true)
		{
			searchA = ! searchA;
			current_iterations++;
			if (isGoal(current_A->coordinates, current_B->coordinates) == true)
			{
				done_flag = true;
				// current_B->parent = current_A;
				std::cout << "Found a path ";
			}
		}
		if (current_iterations == max_iterations)
		{
			std::cout << "No path found." << std::endl;
			current_A = NULL;
			current_B = NULL;
			releaseVertices(VertexSetA);
			releaseVertices(VertexSetB);
			return;
		}
	}
	
	while (current_A != NULL) 
	{
		path.push_back(current_A->coordinates);
		current_A = current_A->parent;
	}
	reverse(path.begin(), path.end());
	while (current_B != NULL)
	{
		path.push_back(current_B->coordinates);
		current_B = current_B->parent;
	}
	if (!path.empty()) 
	{
		std::cout << "with " <<  path.size() << " vertices. " << std::endl;
		for (auto ele:path)
		{
			std::cout << "[" << ele.x << "," << ele.y << "] "; 
		}
		std::cout << "\n";
	}
	releaseVertices(VertexSetA);
	releaseVertices(VertexSetB);
}

void RRT::Bi_RRT::releaseVertices(std::set<Vertex*>& Vertices_)
{
	for (auto it = Vertices_.begin(); it != Vertices_.end();) 
	{
		delete *it;
		it = Vertices_.erase(it);
	}
}

int main()
{
	RRT::Bi_RRT temp;
	temp.setmap(50, 50);
	temp.setsearchflag();
	temp.setstepsize(2.0);
	temp.setgoalbias(0.07);
	temp.setrandompointsize(5.0);
	temp.setgoalradius(1.0);
	temp.setmaxiterations(1000);
	RRT::Vec2i start, goal;
	start.x = 10.0;
	start.y = 10.0;
	goal.x = 35.0;
	goal.y = 35.0;
	RRT::Rectobstacle obstacle1{5,30,15,30};
	temp.addobstacle(obstacle1);
	// std::cout << "obstacle: " << temp.Obstacleset[0].topleftx << " " << temp.Obstacleset[0].toplefty << std::endl;

	temp.findPath(start, goal);
}
