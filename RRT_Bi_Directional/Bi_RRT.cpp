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

bool RRT::Bi_RRT::isHit(Vec2i coordinates1_, Vec2i coordinates2_)
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
	}
	return false;
}

bool RRT::Bi_RRT::islineinsect(Vec2i line1p1_, Vec2i line1p2_, Vec2i line2p1_, Vec2i line2p2_)
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
bool RRT::Bi_RRT::isInObstacle(const Vec2i& coordinates_)
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
		&& coordinates_.x < map_width && coordinates_.y < map_height
		&& closestvertex_.x > 0 && closestvertex_.y > 0 
		&& closestvertex_.x < map_width && closestvertex_.y < map_height 
		&& isInObstacle(coordinates_) == false && isInObstacle(closestvertex_) == false
		&& isHit(coordinates_, closestvertex_) == false)
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
		// std::cout << " searchA: " << searchA;
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
		// std::cout << " extend: " << extend(closestv, randompoint) << std::endl;
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
	exportpath();
	releaseVertices(VertexSetA);
	releaseVertices(VertexSetB);
}

void RRT::Bi_RRT::smoothpath(Vec2i goal_)
{	
	if (path.size() <= 2) {
		smooth_path = path;
		return;
	}
	smooth_path.push_back(path[0]);
	int index1 = 0;
	int index2 = 1;
	// std::cout << "path size: " << path.size() << std::endl;
	while (true)
	{
		// std::cout << "Index: " << index1 << " " << index2 << std::endl;
		if (isValid(path[index1], path[index2]) == true)
		{	
			// std::cout << index1 << " " << index2 << " " << std::endl;
			index2++;
		}
		else
		{	
			if (index2 - 1 != 0)
			{
				smooth_path.push_back(path[index2-1]);
			}
			index1 = index2 - 1;
			index2 = index1 + 1;
		}
		if (index1 + 1 ==  path.size() || index2 == path.size())
		{
			break;
		}
	}

	if (smooth_path.back().x != goal_.x && smooth_path.back().y != goal_.y)
	{
		smooth_path.push_back(path.back());
	}
}

void RRT::Bi_RRT::exportpath()
{
	std::ofstream file_path;
	file_path.open("../path.txt",std::ios::trunc);
	for (int i=0; i<path.size(); i++)
	{
		file_path << path[i].x << " "; 
	}
	file_path << "\n";
	for (int i=0; i<path.size(); i++)
	{
		file_path << path[i].y << " "; 
	}
	file_path << "\n";
	file_path.close();

	std::ofstream file_smoothpath;
	file_smoothpath.open("../smoothpath.txt",std::ios::trunc);
	for (int i=0; i<smooth_path.size(); i++)
	{
		file_smoothpath << smooth_path[i].x << " "; 
	}
	file_smoothpath << "\n";
	for (int i=0; i<smooth_path.size(); i++)
	{
		file_smoothpath << smooth_path[i].y << " "; 
	}
	file_smoothpath << "\n";

	file_smoothpath.close();
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
	temp.setstepsize(3.0);
	temp.setgoalbias(0.07);
	temp.setrandompointsize(5.0);
	temp.setgoalradius(1.0);
	temp.setmaxiterations(10000);
	RRT::Vec2i start, goal;
	start.x = 10.0;
	start.y = 10.0;
	goal.x = 40.0;
	goal.y = 25.0;
	RRT::Rectobstacle obstacle1{5,20,15,20};
	RRT::Rectobstacle obstacle2{5,30,30,0};
	temp.addobstacle(obstacle1);
	temp.addobstacle(obstacle2);
	// std::cout << "obstacle: " << temp.Obstacleset[0].topleftx << " " << temp.Obstacleset[0].toplefty << std::endl;

	temp.findPath(start, goal);
}
