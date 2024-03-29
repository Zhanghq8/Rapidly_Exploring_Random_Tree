#include "../include/rrt_visualization/RRTStar.h"

RRTStar::RRTStar::RRTStar()
{	
	reach_goal = false;
	setmap(50, 50);
	setstepsize(3.0);
	setnearradius(5);
	setgoalbias(0.07);
	setrandompointsize(5.0);
	setgoalradius(3.0);
	setmaxiterations(5000);
	Vec2i start, goal;
	start.x = 10.0;
	start.y = 10.0;
	goal.x = 40.0;
	goal.y = 25.0;
	Rectobstacle obstacle1{5,20,15,20};
	Rectobstacle obstacle2{5,30,30,0};
	addobstacle(obstacle1);
	addobstacle(obstacle2);
	findPath(start, goal);
}


RRTStar::Vertex::Vertex(Vec2i coordinates_, Vertex *parent_, float cost_)
{
	coordinates = coordinates_;
	parent = parent_;
	cost = cost_;
}

void RRTStar::RRTStar::setmap(float map_width_, float map_height_)
{
	map_width = map_width_;
	map_height = map_height_;
}

void RRTStar::RRTStar::setgoalbias(float goal_bias_)
{
	goal_bias = goal_bias_;
}

void RRTStar::RRTStar::setstepsize(float step_size_)
{
	step_size = step_size_;
}

void RRTStar::RRTStar::setmaxiterations(int max_iterations_)
{
	max_iterations = max_iterations_;
}

void RRTStar::RRTStar::setgoalradius(float goal_radius_)
{
	goal_radius = goal_radius_;
}

void RRTStar::RRTStar::setnearradius(float near_radius_)
{
	near_radius = near_radius_;
}

void RRTStar::RRTStar::setrandompointsize(float randompoint_size_)
{
	randompoint_size = randompoint_size_;
}

void RRTStar::RRTStar::addobstacle(Rectobstacle obstacle_)
{
	Obstacleset.push_back(obstacle_);
}

bool RRTStar::RRTStar::isHit(Vec2i coordinates1_, Vec2i coordinates2_)
{	
	// std::cout << "Found " << Obstacleset.size() << " obstacle. " << std::endl;
	if (Obstacleset.size() == 0)
	{
		return false;
	}
	for (int i=0; i<Obstacleset.size(); i++)
	{	
		// std::cout << "Obstacle index: " << i << std::endl;	
		Vec2i bottomleft = {Obstacleset[i].bottomleftx, Obstacleset[i].bottomlefty};
		Vec2i bottomright = {Obstacleset[i].bottomleftx + Obstacleset[i].width, Obstacleset[i].bottomlefty};
		Vec2i topleft = {Obstacleset[i].bottomleftx, Obstacleset[i].bottomlefty + Obstacleset[i].height};
		Vec2i topright = {Obstacleset[i].bottomleftx + Obstacleset[i].width, Obstacleset[i].bottomlefty + Obstacleset[i].height};
		// std::cout << "point" << bottomleft.x << " " << bottomleft.y << " " << topleft.x << " " << topleft.y << std::endl;
		bool top = islineintersect(coordinates1_, coordinates2_, topleft, topright);
		bool bottom = islineintersect(coordinates1_, coordinates2_, bottomleft, bottomright);
		bool left = islineintersect(coordinates1_, coordinates2_, topleft, bottomleft);
		bool right = islineintersect(coordinates1_, coordinates2_, topright, bottomright);
		// std::cout << "line" << top << bottom << left << right << std::endl;
		if (top || bottom || left || right)
		{
			return true;
		}
	}
	return false;
}


bool RRTStar::RRTStar::islineintersect(Vec2i line1p1_, Vec2i line1p2_, Vec2i line2p1_, Vec2i line2p2_)
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
bool RRTStar::RRTStar::isInObstacle(const Vec2i& coordinates_)
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
bool RRTStar::RRTStar::isGoal(Vec2i source_, Vec2i goal_) 
{
	float distance = euclidean_dis(source_, goal_);
	if (distance <= goal_radius) 
	{
		return true;
	}
	return false;
}

// check if the coordinate is valid

bool RRTStar::RRTStar::isValid(Vec2i coordinates_, Vec2i closestvertex_) 
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
float RRTStar::RRTStar::euclidean_dis(Vec2i source_, Vec2i goal_) 
{
	float e_distance = sqrt(pow(source_.x - goal_.x, 2) + pow(source_.y - goal_.y, 2));
	return e_distance; 
}

//generate new randompoint with goal_bias% probability to pick goal point 
RRTStar::Vec2i RRTStar::RRTStar::GenerateRandomPoint(Vec2i goal_)
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

RRTStar::Vertex* RRTStar::RRTStar::getClosestVertex(std::set<Vertex*>& Vertices_, Vec2i randompoint_)
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
bool RRTStar::RRTStar::extend(Vertex* closestvertex_, Vec2i randompoint_, Vec2i goal_)
{
	float theta = atan2(randompoint_.y - closestvertex_->coordinates.y, randompoint_.x - closestvertex_->coordinates.x);
	// std::cout << "theta: " << theta << std::endl;
	Vec2i vertextemp;
	vertextemp.x = closestvertex_->coordinates.x + step_size * cos(theta);
	vertextemp.y = closestvertex_->coordinates.y + step_size * sin(theta);
	// std::cout << "isvalid " << isValid(vertextemp, closestvertex_->coordinates) << std::endl;
	if (isValid(vertextemp, closestvertex_->coordinates) == true) 
	{	
		visitednode.push_back(vertextemp);
		rewire(closestvertex_, vertextemp, goal_);
		// Vertex* newparent_ = rewire(VertexSet, vertextemp);
		// // std::cout << "isvalid " << isValid(vertextemp, newparent_->coordinates) << std::endl;

		// if (isValid(vertextemp, newparent_->coordinates) == true)
		// {	
		// 	visitednode.push_back(vertextemp);
		// 	Vertex* newvertex = new Vertex(vertextemp, newparent_, newparent_->cost + step_size);
		// 	// std::cout << "cost: " << newvertex->cost << std::endl;
		// 	current = newvertex;
		// 	VertexSet.insert(newvertex);
		return true;
		// }	
	}
	return false;
}

// Search in Vertices set to find all vertex which the distance to newvertex is less than near radius
// Find the vertex with minimum cost
void RRTStar::RRTStar::rewire(Vertex* closestvertex_, Vec2i newvertex_, Vec2i goal_)
{	
	//find nearby node
	std::set<Vertex*> Nearset;
	for (auto vertex:VertexSet) 
	{
		if (euclidean_dis(vertex->coordinates, newvertex_) <= near_radius) {
			Nearset.insert(vertex);
		}
	}

	Vertex* newparent = closestvertex_;

	// std::cout << "Nearset size: " << Nearset.size() << std::endl;
	//get parent with minimum cost
	float mincost = closestvertex_->cost + step_size;
	for (auto nearvertex:Nearset) 
	{	
		float tempcost = nearvertex->cost + euclidean_dis(nearvertex->coordinates, newvertex_);
		if (tempcost < mincost && isValid(newvertex_, nearvertex->coordinates))
		{
			mincost = tempcost;
			newparent = nearvertex;
		}
	}
	// std::cout << "new parent: " << newparent << std::endl;
	// std::cout << "Minimum cost: " << mincost << std::endl;

	// Build edge
	Vertex* newvertex = new Vertex(newvertex_, newparent, mincost);
	current = newvertex;
	if (isGoal(current->coordinates, goal_) == true && reach_goal == false && isValid(current->coordinates, goal_))
	{
		goal->parent = current;
		goal->cost = current->cost + euclidean_dis(current->coordinates, goal->coordinates);
		reach_goal = true;
		std::cout << "Found a path. " << std::endl;
	}
	if (isGoal(current->coordinates, goal_) == true && reach_goal == true && isValid(current->coordinates, goal_))
	{	
		if (current->cost + euclidean_dis(current->coordinates, goal->coordinates) < goal->cost)
		{
			goal->parent = current;
			goal->cost = current->cost + euclidean_dis(current->coordinates, goal->coordinates) < goal->cost;
		}
		// std::cout << "Found a path. " << std::endl;
	}
	VertexSet.insert(newvertex);

	for (auto nearvertex:Nearset) 
	{	
		if (nearvertex != newparent)
		{
			if (isValid(nearvertex->coordinates, newvertex->coordinates) && 
				nearvertex->cost > (newvertex->cost + euclidean_dis(nearvertex->coordinates, newvertex->coordinates)))
			{
				nearvertex->parent = newvertex;
				nearvertex->cost = newvertex->cost + euclidean_dis(nearvertex->coordinates, newvertex->coordinates);
			}
		}
	}
	// return newparent;
}

void RRTStar::RRTStar::findPath(Vec2i source_, Vec2i goal_)
{	
	VertexSet.insert(new Vertex(source_));
	current = *VertexSet.begin();
	int current_iterations = 0;
	goal = new Vertex(goal_);
	while (current_iterations < max_iterations) 
	{	
		// std::cout << current_iterations << std::endl;
		Vec2i randompoint = GenerateRandomPoint(goal_);
		Vertex* closestv= getClosestVertex(VertexSet, randompoint);
		// std::cout << "Closestv: " << closestv << std::endl;
		if (extend(closestv, randompoint, goal_) == true)
		{
			current_iterations++;
			// std::cout << current_iterations << std::endl;
			// std::cout << "Reach goal?" << reach_goal << std::endl;
			if (reach_goal == true && current_iterations%500 == 0)
			{
				// Vertex* temp = goal;
				// while (temp != NULL) 
				// {
				// 	path.push_back(temp->coordinates);
				// 	temp = temp->parent;
				// }
				// float final_cost = 0;
				// reverse(path.begin(), path.end());
				// if (!path.empty()) 
				// {
				// 	std::cout << "with " <<  path.size() << " vertices. " << std::endl;
				// 	std::cout << "[" << path[0].x << "," << path[0].y << "] ";
				// 	for (int i=1; i<path.size(); i++)
				// 	{
				// 		std::cout << "[" << path[i].x << "," << path[i].y << "] ";
				// 		final_cost += euclidean_dis(path[i], path[i-1]); 
				// 	}
				// 	std::cout << "\n";
				// }
				// std::cout << "Final cost(without smooth): " << final_cost << std::endl;
				// path.clear();
			}
		}
		if (current_iterations == max_iterations)
		{
			std::cout << "Finished." << std::endl;
			// current = NULL;
			// releaseVertices(VertexSet);
		}
	}
	
	while (goal != NULL) 
	{
		path.push_back(goal->coordinates);
		goal = goal->parent;
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

	randomsmoothpath();
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
	releaseVertices(VertexSet);
}

void RRTStar::RRTStar::minsmoothpath(Vec2i goal_)
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

void RRTStar::RRTStar::randomsmoothpath()
{	
	smooth_path = path;
	if (path.size() <= 2) 
	{
		return;
	}
	int iteration = smooth_path.size() * 1.5;

	for (int i=0; i<iteration; i++)
	{	
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<int> x(0, smooth_path.size()-1);
		std::uniform_int_distribution<int> y(0, smooth_path.size()-1);

		auto index1 = x(gen);
		auto index2 = y(gen);
		// std::cout << "index " << index1 << " " << index2 << std::endl;
		if (isValid(smooth_path[index1], smooth_path[index2]))
		{
			if (abs(index1 - index2) > 1)
			{
				smooth_path.erase(smooth_path.begin() + std::min(index1, index2) + 1,
				 smooth_path.begin() + std::max(index1, index2));				
			}
		}
	}
}

void RRTStar::RRTStar::releaseVertices(std::set<Vertex*>& Vertices_)
{	
	std::cout << "Visited vertices: " << Vertices_.size() << std::endl;
	for (auto it = Vertices_.begin(); it != Vertices_.end();) 
	{
		delete *it;
		it = Vertices_.erase(it);
	}
}

void RRTStar::RRTStar::exportpath()
{
	std::ofstream file_path;
	file_path.open("/home/han/catkin_ws/src/rrt_visualization/path.txt",std::ios::trunc);
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
	file_smoothpath.open("/home/han/catkin_ws/src/rrt_visualization/smoothpath.txt",std::ios::trunc);
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

// int main()
// {
// 	RRTStar::RRTStar temp;
// 	temp.setmap(50, 50);
// 	temp.setstepsize(3.0);
// 	temp.setnearradius(3.5);
// 	temp.setgoalbias(0.07);
// 	temp.setrandompointsize(5.0);
// 	temp.setgoalradius(1.0);
// 	temp.setmaxiterations(10000);
// 	RRT::Vec2i start, goal;
// 	start.x = 10.0;
// 	start.y = 10.0;
// 	goal.x = 40.0;
// 	goal.y = 25.0;
// 	RRT::Rectobstacle obstacle1{5,20,15,20};
// 	RRT::Rectobstacle obstacle2{5,30,30,0};
// 	temp.addobstacle(obstacle1);
// 	temp.addobstacle(obstacle2);
// 	// std::cout << "obstacle: " << temp.Obstacleset[0].topleftx << " " << temp.Obstacleset[0].toplefty << std::endl;

// 	temp.findPath(start, goal);
// }
