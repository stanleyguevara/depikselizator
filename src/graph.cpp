#include "graph.h"
#include <stack>
#include <utility>

#include <iostream>
#include <sstream>

//Checks if the requested pixel is in range of the image
bool isValid(int x,int y, int w, int h)
{
	return (x>=0 && x<w && y>=0 && y<h);
}


Graph::Graph(Image& imageI)
{
	//Innitializing variables from Image
	this->image = &imageI;
	int h = image->getHeight();
	int w = image->getWidth();

	//Semantic
	// edges[i][j][k] -> denotes whether there is a an edge from (i,j) in kth direction in the graph
	//Preallocating structures
	for(int i = 0; i < w; i++)
	{
		std::vector<std::vector<int>> v2(h);
		for(int j = 0; j < h; j ++)
		{
			v2[j] = std::vector<int>(8,0);
		}
		weights.push_back(v2);
	}

	//Add edge in kth direction of (x,y) of (x,y)+k is valid cell and has similar color
	for(int i = 0 ; i < w ; i++) for(int j = 0; j < h; j++) for(int k = 0 ; k < 8; k ++) {
		Pixel* p = (*image)(i, j);
		Pixel* adjP = image->getAdjacent(p, (Direction)k);
		if (adjP) {
			if(p->color() == adjP->color())
				edges.insert(make_pair(IntPoint(i, j), (Direction)k));
		};
	}
}

void Graph::remove_cross()
{
	//Take current pixel as top-left of a 4 pixel square and check whether the colors are same in all
	for(int i = 0 ; i < this->image->getWidth() - 1; i++) for(int j = 0 ; j < this->image->getHeight() - 1; j++)
	{
		Pixel* topLeft = (*image)(i,j);
		Pixel* topRight = (*image).getAdjacent(i, j, RIGHT);
		Pixel* bottomLeft = (*image).getAdjacent(i, j, BOTTOM);
		Pixel* bottomRight = (*image).getAdjacent(i, j, BOTTOM_RIGHT);
		if(
			topLeft->color() == topRight->color() &&
			topLeft->color() == bottomLeft->color() &&
			topLeft->color() == bottomRight->color() &&
			bottomLeft->color() == topRight->color())
		{
			//All colors are same in the square, remove diagonal edges
			delete_edge(topLeft, BOTTOM_RIGHT);
			delete_edge(bottomRight, TOP_LEFT);
			delete_edge(topRight, BOTTOM_LEFT);
			delete_edge(bottomLeft, TOP_RIGHT);
		}
	}

}


int Graph::valence(int x,int y)
{
	//Checks for out of range input
	if(x < 0 || x >= this->image->getWidth()) return -1;
	if(y < 0 || y >= this->image->getHeight()) return -1;

	//Count the edges around the pixel
	int cnt = 0;
	for(int i = 0; i < 8 ; i++) if(edge(x,y, (Direction)i)) cnt ++;
	return cnt;
}

//Checks if (x,y) are inclusively inside the given cell
bool insideBounds(int x, int y, int row_st, int row_end, int col_st, int col_end)
{
	return (x>=row_st && x <= row_end && y >= col_st && y <= col_end);
}

//Curves Heuristic
void Graph::curves_heuristic(IntPoint ip)
{
	int x = X(ip);
	int y = Y(ip);
	//Directions are (x,y) BOTTOM_RIGHT, and Right(x,y) BOTTOM_LEFT
	//A : feature for (x,y) BOTTOM_RIGHT and B : (x,y) + Right BOTTOM_LEFT
	int featureA = 0;
	int featureB = 0;

	//Find curve lengths in both directions for A
	int p = x, q = y;
	int dir = BOTTOM_RIGHT;
	while(true)
	{
		featureA++;
		if(valence(p,q) != 2) break;
		int i;

		for(i = dir+1; !edge(p, q, (Direction)i); i = (i+1)%8);
		if((i+dir)==7) break;
		dir = i;
		p = p + direction[dir][0];
		q = q + direction[dir][1];
	}
	p = x+direction[BOTTOM_RIGHT][0];
	q = y+direction[BOTTOM_RIGHT][1];
	dir = TOP_LEFT;
	while(true)
	{
		featureA++;
		if(valence(p,q) != 2) break;

		int i;
		for(i = dir+1; !edge(p, q, (Direction)i); i = (i+1)%8);
		if((i+dir)==7) break;
		dir = i;
		p = p + direction[dir][0];
		q = q + direction[dir][1];
	}

	//Find curve lengths in both directions for B
	p = x+direction[RIGHT][0];
	q = y+direction[RIGHT][1];
	dir = BOTTOM_LEFT;
	while(true)
	{
		featureB++;
		if(valence(p,q) != 2) break;

		int i;
		for(i = dir+1; !edge(p, q, (Direction)i); i = (i+1)%8);
		if((i+dir)==7) break;
		dir = i;
		p = p + direction[dir][0];
		q = q + direction[dir][1];
	}

	p = x+direction[RIGHT][0] + direction[BOTTOM_LEFT][0];
	q = y+direction[RIGHT][1] + direction[BOTTOM_LEFT][1];
	dir = TOP_RIGHT;

	while(true)
	{
		featureB++;
		if(valence(p,q) != 2) break;

		int i;
		for(i = dir+1; !edge(p, q, (Direction)i); i = (i+1)%8);
		if((i+dir)==7) break;
		dir = i;
		p = p + direction[dir][0];
		q = q + direction[dir][1];
	}

	//Bigger curve is better, add difference to corresponding edge weight
	if(featureA < featureB) {
		weights[x+direction[RIGHT][0]][y+direction[RIGHT][1]][BOTTOM_LEFT] += (featureB - featureA);
		weights[x+direction[BOTTOM][0]][y+direction[BOTTOM][1]][TOP_RIGHT] += (featureB - featureA);
	}
	else
	{
		weights[x][y][BOTTOM_RIGHT] += (featureA - featureB);
		weights[x+direction[BOTTOM_RIGHT][0]][y+direction[BOTTOM_RIGHT][1]][TOP_LEFT] += (featureA - featureB);
	}
}

//Checks for Isolated pixels
void Graph::islands_heuristic(IntPoint ip)
{
	int x = X(ip);
	int y = Y(ip);
	for(int i = 0; i < 8; i++) if(edge(x, y, (Direction)i))
	{
		weights[x][y][i] = 5 * ((valence(x,y) == 1) + (valence(x+direction[BOTTOM_RIGHT][0],y+direction[BOTTOM_RIGHT][1]) == 1));
		weights[x+direction[i][0]][y+direction[i][1]][7-i] = 5 * ((valence(x,y) == 1) + (valence(x+direction[BOTTOM_RIGHT][0],y+direction[BOTTOM_RIGHT][1]) == 1));
	}
}

void Graph::sparse_pixels_heuristic(IntPoint ip)
{
	int x = X(ip);
	int y = Y(ip);
	//Directions are BOTTOM_RIGHT, and BOTTOM_LEFT
	//Measure the size of the connected component in a 8x8 box
	if(!insideBounds(x+direction[RIGHT][0],y+direction[RIGHT][1],0,image->getWidth()-1,0, image->getHeight()-1)) return;
	int labels[8][8] = {0};
	std::stack<std::pair<int,int>> st;
	//Do DFS from (x,y) labeling 1 to each connected node.
	st.push(std::make_pair(x,y));
	labels[3][3] = 1; // Position of (X,Y) in the label array
	while(!st.empty())
	{
		std::pair<int,int> point = st.top();
		int p = point.first;
		int q = point.second;
		st.pop();
		for(int i = 0 ; i < 8; i++)
		{
			//See in all directions, scan for points that are in the not yet visited, in the 8x8 box, and have an edge from the current point.
			if(!insideBounds(p+direction[i][0],q+direction[i][1],0,image->getWidth()-1,0, image->getHeight()-1) || !insideBounds(p+direction[i][0],q+direction[i][1],x-3,x+4,y-3,y+4)) continue;
			if(labels[3+p+direction[i][0]-x][3+q+direction[i][1]-y] != 0) continue;
			if(!edge(p, q, (Direction)i)) continue;
			st.push(std::make_pair(p+direction[i][0],q+direction[i][1]));
			labels[3+p+direction[i][0]-x][3+q+direction[i][1]-y] = 1;
		}
	}

	st.push(std::make_pair(x+direction[RIGHT][0],y+direction[RIGHT][1]));
	labels[4][3] = 2;

	while(!st.empty())
	{
		std::pair<int,int> point = st.top();
		int p = point.first;
		int q = point.second;
		st.pop();
		for(int i = 0 ; i < 8; i++)
		{
			//See in all directions, scan for points that are in the not yet visited, in the 8x8 box, and have an edge from the current point.
			if(!insideBounds(p+direction[i][0],q+direction[i][1],0,image->getWidth()-1,0, image->getHeight()-1) || !insideBounds(p+direction[i][0],q+direction[i][1],x-3,x+4,y-3,y+4)) continue;
			if(labels[3+p+direction[i][0]-x][3+q+direction[i][1]-y] != 0) continue;
			if(!edge(p, q, (Direction)i)) continue;
			st.push(std::make_pair(p+direction[i][0],q+direction[i][1]));
			labels[3+p+direction[i][0]-x][3+q+direction[i][1]-y] = 2;
		}
	}

	//Find the size of the connected components
	int componentA = 0, componentB = 0;
	for(int i = 0 ; i < 8 ; i++) for(int j = 0 ; j < 8; j++)
	{
		if(labels[i][j] == 1) componentA++;
		else if(labels[i][j] == 2) componentB++;
	}

	//Smaller component is better
	if(componentA > componentB)
	{
		weights[x+direction[RIGHT][0]][y+direction[RIGHT][1]][BOTTOM_LEFT] += (componentA-componentB);
		weights[x+direction[BOTTOM][0]][y+direction[BOTTOM][1]][TOP_RIGHT] += (componentA-componentB);
	}
	else
	{
		weights[x][y][BOTTOM_RIGHT] += (componentB-componentA);
		weights[x+direction[BOTTOM_RIGHT][0]][y+direction[BOTTOM_RIGHT][1]][TOP_LEFT] += (componentB-componentA);
	}
}

// Define a mapping from Direction values to their corresponding constant names
//const std::map<Direction, std::string> DIRECTION_NAMES = {
//	{TOP, "↑,"},
//	{TOP_RIGHT, "↗,"},
//	{TOP_LEFT, "↖,"},
//	{LEFT, "←,"},
//	{RIGHT, "→,"},
//	{BOTTOM, "↓,"},
//	{BOTTOM_RIGHT, "↘,"},
//	{BOTTOM_LEFT, "↙,"}
//};

const std::map<Direction, std::string> DIRECTION_ICONS = {
	{TOP, "·"},
	{TOP_RIGHT, "·"},
	{TOP_LEFT, "·"},
	{LEFT, "·"},
	{RIGHT, "·"},
	{BOTTOM, "·"},
	{BOTTOM_RIGHT, "·"},
	{BOTTOM_LEFT, "·"}
};

const std::map<Direction, std::string> DIRECTION_NAMES = {
	{TOP, "TOP"},
	{TOP_RIGHT, "TOP_RIGHT"},
	{TOP_LEFT, "TOP_LEFT"},
	{LEFT, "LEFT"},
	{RIGHT, "RIGHT"},
	{BOTTOM, "BOTTOM"},
	{BOTTOM_RIGHT, "BOTTOM_RIGHT"},
	{BOTTOM_LEFT, "BOTTOM_LEFT"}
};

// Define a helper function to convert a Direction value to its constant name
std::string directionToString(Direction dir) {
	return DIRECTION_NAMES.at(dir);
}

std::string intToOneCharString(int num) {
	if (num >= 0 && num <= 9) {
		return std::to_string(num);
	}
	else {
		return "s";
	}
}

std::string intToString(int num) {
	std::stringstream ss;
	if (num == 0) {
		ss << "░░";
	} else if (num > 0 && num <= 9) {
		ss << "0" << num;
	} else {
		ss << num;
	}
	return ss.str();
}

void printNonZeroWeights(const std::vector<std::vector<std::vector<int>>>& weights) {
	for (size_t x = 0; x < weights.size(); ++x) {
		for (size_t y = 0; y < weights[x].size(); ++y) {
			for (size_t dir = 0; dir < weights[x][y].size(); ++dir) {
				int weight = weights[x][y][dir];
				if (weight != 0) {
					Direction direction = static_cast<Direction>(dir);
					std::string directionName = DIRECTION_NAMES.at(direction);
					std::cout << "x=" << x << ", y=" << y << ", direction=" << directionName << std::endl;
				}
			}
		}
	}
}

void printEdges2(
	const std::set<std::pair<IntPoint, Direction>>& edges,
	const std::vector<std::vector<std::vector<int>>>& weights,
	int width, int height) {
	// Initialize a 2D vector to hold the cells for each pixel in the image
	std::vector<std::vector<std::string>> cellGrid(height * 3, std::vector<std::string>(width * 3, "  "));

	// Populate the cell grid with the x and edge arrows
	for (const auto& edge : edges) {
		IntPoint point = edge.first;
		Direction dir = edge.second;
		int x = point.first;
		int y = point.second;

		// Determine the grid coordinates corresponding to the point
		int row = y;
		int col = x;

		// Determine the cell coordinates for the middle of the 3x3 grid
		int midRow = row * 3 + 1;
		int midCol = col * 3 + 1;

		// Set the middle cell to "x"
		cellGrid[midRow][midCol] = "▒▒";

		//std::cout << "weights[" << x << "][" << y << "][TOP]" << weights[x][y][TOP] << std::endl;

		// Set the edge arrow cells if the edge exists
		if (edges.find({ point, TOP }) != edges.end()) {
			cellGrid[midRow - 1][midCol] = intToString(weights[x][y][TOP]);
		}
		if (edges.find({ point, TOP_RIGHT }) != edges.end()) {
			cellGrid[midRow - 1][midCol + 1] = intToString(weights[x][y][TOP_RIGHT]);
		}
		if (edges.find({ point, TOP_LEFT }) != edges.end()) {
			cellGrid[midRow - 1][midCol - 1] = intToString(weights[x][y][TOP_LEFT]);
		}
		if (edges.find({ point, LEFT }) != edges.end()) {
			cellGrid[midRow][midCol - 1] = intToString(weights[x][y][LEFT]);
		}
		if (edges.find({ point, RIGHT }) != edges.end()) {
			cellGrid[midRow][midCol + 1] = intToString(weights[x][y][RIGHT]);
		}
		if (edges.find({ point, BOTTOM }) != edges.end()) {
			cellGrid[midRow + 1][midCol] = intToString(weights[x][y][BOTTOM]);
		}
		if (edges.find({ point, BOTTOM_RIGHT }) != edges.end()) {
			cellGrid[midRow + 1][midCol + 1] = intToString(weights[x][y][BOTTOM_RIGHT]);
		}
		if (edges.find({ point, BOTTOM_LEFT }) != edges.end()) {
			cellGrid[midRow + 1][midCol - 1] = intToString(weights[x][y][BOTTOM_LEFT]);
		}
	}

	// Print the cell grid to std::cout
	for (int row = 0; row < height * 3; row++) {
		for (int col = 0; col < width * 3; col++) {
			std::cout << cellGrid[row][col];
		}
		std::cout << std::endl;
	}
}

void Graph::planarize()
{
	//Remove Crosses for obvious planarization
	remove_cross();
	//For Internal Pixels, process via heuristic if edges are crossing
	//A Pixel is the topLeft of a 2x2 box
	for(int i = 0 ; i < this->image->getWidth() - 1; i++) for(int j = 0 ; j < this->image->getHeight() - 1; j++)
	{
		Pixel* topLeft = (*image)(i, j);
		Pixel* topRight = (*image).getAdjacent(i, j, RIGHT);
		Pixel* bottomLeft = (*image).getAdjacent(i, j, BOTTOM);
		Pixel* bottomRight = (*image).getAdjacent(i, j, BOTTOM_RIGHT);
		if (!(topLeft && topRight && bottomLeft && bottomRight)) continue;
		if(edge(topLeft, BOTTOM_RIGHT) && edge(topRight, BOTTOM_LEFT))
		{
			//Edges are crossing and the pixels are dissimilar, need to discard atleast one.
			//Check if there are horizontal/vertical connections
			if(edge(topLeft, BOTTOM)) continue;
			if(edge(topLeft, RIGHT)) continue;
			if(edge(bottomRight, TOP)) continue;
			if(edge(bottomRight, LEFT)) continue;

			//Run heuristics for weight
			islands_heuristic(*topLeft);
			islands_heuristic(*topRight);
			curves_heuristic(*topLeft);
			sparse_pixels_heuristic(*topLeft);

			//Remove the lighter edge. And both if they are equal
			if(this->weights[topLeft->X()][topLeft->Y()][BOTTOM_RIGHT] <= this->weights[topRight->X()][topRight->Y()][BOTTOM_LEFT])
			{
				delete_edge(topLeft, BOTTOM_RIGHT);
				delete_edge(bottomRight, TOP_LEFT);
			}
			if(this->weights[topLeft->X()][topLeft->Y()][BOTTOM_RIGHT] >= this->weights[topRight->X()][topRight->Y()][BOTTOM_LEFT])
			{
					delete_edge(topRight, BOTTOM_LEFT);
					delete_edge(bottomLeft, TOP_RIGHT);
				}
		}
	}
}