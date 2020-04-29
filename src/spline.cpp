#include "spline.h"
#include "voronoi.h"

//Returns the darker pixel by Y luminescence value
Pixel* darker(Pixel* a, Pixel* b)
{
	ColorYUV color1 = a->color();
	ColorYUV color2 = b->color();
    if(color1.Y < color2.Y) return a;
    else return b;
}

//Extracts active edges from voronoi diagrams
void Spline::extractActiveEdges()
{
	if(this->diagram == nullptr) return;
	
	//For keeping the edges detected till now.
	std::map<Edge,Pixel*> edgeEnum;

	//Get image dimensions
	Image* imageRef = this->diagram->getImage(); 
	int width = imageRef->getWidth();
	int height = imageRef->getHeight();

	for(int x=0; x <width; x++)
	{
		for(int y=0; y< height; y++)
		{
			for(int i = 0 ; i < (*this->diagram)(x,y).size(); i++) 
			{

				//Looping over the voronoi hull, of point (x,y), we find the edges which have different colored pixels on either side, and add to active edges.
				int l = i;
				int r = (i+1)%((*this->diagram)(x,y).size());
				if(edgeEnum.find(make_pair((*this->diagram)(x,y)[r],(*this->diagram)(x,y)[l])) != edgeEnum.end()) 
				{
					auto p = edgeEnum[make_pair((*this->diagram)(x,y)[r],(*this->diagram)(x,y)[l])];
					if(p != (*imageRef)(x,y) && !(p->color() == (*imageRef)(x,y)->color())) activeEdges.push_back(make_pair(make_pair((*this->diagram)(x,y)[l],(*this->diagram)(x,y)[r]),darker(p,(*imageRef)(x,y)))); 
				}
				else edgeEnum[std::make_pair((*this->diagram)(x,y)[l],(*this->diagram)(x,y)[r])] = (*imageRef)(x,y);
			}
		}
	}
}

void Spline::calculateGraph()
{
	//Convert edge list to adjacency list
	for(auto edge : activeEdges)
	{
		graph[edge.first.first].insert(std::make_pair(edge.first.second,edge.second->color()));
		graph[edge.first.second].insert(std::make_pair(edge.first.first,edge.second->color()));
	}
}

std::vector<std::pair<std::vector<Point>,Color> > Spline::printGraph()
{
	//Tracing curves. Starting with a random node, We trace out a curve with same colors
	std::vector<std::pair<std::vector<Point>, Color> > mainOutLine;
	std::map<Point, std::set<std::pair<Point,Color> > >::iterator vertexPt = graph.begin();
	while(vertexPt != graph.end())
	{
		while((vertexPt->second).size() > 0)
		{
			Point src = (vertexPt->second).begin()->first;
			Color c = (vertexPt->second).begin()->second;
			std::vector<Point> v = traverseGraph(src, c);
			mainOutLine.push_back(std::make_pair(v,c));
		}
		vertexPt ++;
	}
	return mainOutLine;
}

std::vector<Point > Spline::traverseGraph(const Point& p, const Color& c)
{
	//Contains nodes that have been visited
	std::vector<Point> points;
	Point x = p;
	Point prev = Point(-1,-1);
	Color curr = c;
	bool found = true;
	while(true)
	{
		points.push_back(x);
		for(auto it = graph[x].begin(); it != graph[x].end(); it++) 
		{
			if(it->first == prev) continue;
			//If color of a node is similar to that of one vertex in the adj list, then connect that node.
			if(it->second == c) {
				Point p2 = it->first;
				//cerr << x << "->" << graph[x] << " " << it->first<< "->" << " " << graph[it->first] << endl;
				std::set<std::pair<Point,Color> >::iterator it1;
				for(it1 = graph[p2].begin(); it1 != graph[p2].end(); it1++) {
					if(curr == it1->second && it1->first == x) break;
				}
				
				if(it1 == graph[p2].end()) break;
				curr = it->second;
				graph[x].erase(it);
				graph[p2].erase(it1);
				//cerr << x << "->" << graph[x] << " " << p2<< "->" << " " << graph[p2] << endl;
				//cerr << "======\n";
				x = p2;
				found = true;
				break;
			}
		}
		if(!found) break;
		found = false;
	}
	if(points.size() > 2 && *points.begin() == *points.rbegin()) 
	{
		points.push_back(points[1]);
	}
	return points;
}

//REF: http://math.stackexchange.com/questions/115241/manually-deducing-the-quadratic-uniform-b-spline-basis-functions
std::vector<std::vector<float> > Spline::getSpline(std::vector<Point > points) // For 3 points
{
	//assert(points.size() == 3);
	//Basis Matrix for quadratic uniform b-spline
	float B[3][3] = {{1, 1, 0},{ -2, 2, 0},{1, -2, 1}};

	// 3x2 vector matrix initialized with 0
	std::vector<std::vector<float> > a(3,std::vector<float>(2,0));

	// multiply it with B-splines basis matrix
	for(int i = 0 ; i < 3; i++)
	{
		for(int j = 0; j < 2; j++)
		{
			for(int k = 0 ; k < 3; k ++) 
			{
				if(j == 0) a[i][j] += 0.5*B[i][k]*points[k].first;
				else a[i][j] += 0.5*B[i][k]*points[k].second;
			}
		}
	}
	return a;
}