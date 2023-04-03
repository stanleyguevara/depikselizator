#include "voronoi.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

int height, width;

void Voronoi::createDiagram(Graph& graph)
{
	int h = imageRef->getHeight();
	int w = imageRef->getWidth();

	height = h;
	width = w;

	for(int i = 0; i < w; i++)
	{
		vector<vector<Point>> v2(h);
		voronoiPts.push_back(v2);
	}
	createRegions(graph);
	collapseValence2();
}

bool Voronoi::onBoundary(Point p)
{
	return p.first == 0 || p.first == width || p.second == 0 || p.second == height;
}

pair<float, float> findCentroid(vector<pair<float, float>>& polygon) {
	float xsum = 0;
	float ysum = 0;
	float area = 0;

	for (int i = 0; i < polygon.size(); i++) {
		int j = (i + 1) % polygon.size(); // Next vertex index
		float crossProduct = (polygon[i].first * polygon[j].second) - (polygon[j].first * polygon[i].second);

		xsum += (polygon[i].first + polygon[j].first) * crossProduct;
		ysum += (polygon[i].second + polygon[j].second) * crossProduct;
		area += crossProduct;
	}

	area /= 2;

	float xcm = xsum / (6 * area);
	float ycm = ysum / (6 * area);

	return make_pair(xcm, ycm);
}

string pairToJson(pair<float, float> pair) {
	ostringstream json;
	json << "{\"x\":" << pair.first << ",\"y\":" << pair.second << "}";
	return json.str();
}

string polyToJson(const vector<pair<float, float>>& polygon) {
	// Format the polygon as a JSON array of objects
	ostringstream json;
	json << "\"vertices\":[" << endl;
	for (int i = polygon.size()-1; i>=0; i--) {
		json << pairToJson(polygon[i]);
		if (i != 0) json << ",";
		json << endl;
	}
	json << "]";
	return json.str();
}

void Voronoi::printVoronoi(string json_path)
{
	Pixel* pixel;
	pair<float, float> centroid;
	string vertices;
	ofstream outfile(json_path);
	outfile << "{\"width\":" << width << ",\"height\":" << height << "," << endl;
	outfile << "\"polygons\":[" << endl;

	for(int i=0; i<width; i++)
	{
		for(int j=0; j<height; j++)
		{
			pixel = (*imageRef)(i, j);
			centroid = findCentroid(voronoiPts[i][j]);
			vertices = polyToJson(voronoiPts[i][j]);
			
			outfile << "{" << vertices << "," << endl;
			outfile << "\"centroid\":" << pairToJson(centroid) << "," << endl;
			outfile << "\"color\":\"" << pixel->getHexColor() << "\"" << endl;
			outfile << "}";
			if (i != width - 1 || j != height - 1) outfile << ",";
			outfile << endl;
		}
	}
	outfile << "]}";
	outfile.close();
}

/*
/	Collapse the valence-2 nodes to further simplify the voronoi diagrams
/	valency - pair<float, float> will be mapped to it's valency
/	Removed all those having valency 4 (added both sides), but  ignored boundary points
*/
void Voronoi::collapseValence2()
{

	for(int x=0; x <width; x++)
	{
		for(int y=0; y< height; y++)
		{
			for(int i = 0 ; i < voronoiPts[x][y].size(); i++)
			{
				int l = i;
				int r = (i+1)%(voronoiPts[x][y].size());
				if(valency.find(voronoiPts[x][y][l]) != valency.end()) {valency[voronoiPts[x][y][l]] += 1;}
				else valency[voronoiPts[x][y][l]] = 1;
				if(valency.find(voronoiPts[x][y][r]) != valency.end()) {valency[voronoiPts[x][y][r]] += 1;}
				else valency[voronoiPts[x][y][r]] = 1;
			}
		}
	}
	for(int x = 0; x < width; x++)
	{
		for(int y=0; y< height; y++)
		{
			vector<Point> temp;
			for(int i = 0 ; i < voronoiPts[x][y].size(); i++)
			{
				if(valency[voronoiPts[x][y][i]] != 4 || onBoundary(voronoiPts[x][y][i])) temp.push_back(voronoiPts[x][y][i]);
			}
			voronoiPts[x][y].clear();
			voronoiPts[x][y] = vector<Point>(temp);
		}
	}
}

/*
/	Create voronoi region around each pixel
/	voronoiPts[x][y].vector<pair<float,float>> will contain all the coordinates(float, float)
/	that creates the voronoi region
*/
void Voronoi::createRegions(Graph& graph)
{
	int x, y, z;
	float xcenter, ycenter;
	for(x = 0; x < width; x++)
	{
		for(y = 0; y < height; y++)
		{
			Pixel* p = graph.getImage()->operator()(x, y);
			xcenter = x + 0.5;
			ycenter = y + 0.5;

			// VORONOI DIAGRAM CALCULATION
			// Each corner can do three things:
			// 1. Gain space from other cells (via type 1 and 2 points) (if an edge exists in that direction)
			// 2. Lose space to other cells (via type 3) (if an external edge cuts that corner)
			// 3. Keep the same space (there are no edges) (original corner point)
			// All the edge midpoints are kept to complete the diagram
			// Edges through the edge midpoints don't affect this diagram
			// This addition of points is done in anti-clowise order
			// TOPLEFT -> LEFT -> BOTTOMLEFT -> BOTTOM -> BOTTOMRIGHT -> RIGHT -> TOPRIGHT -> TOP

			//TOPLEFT
			if(graph.edge(p, TOP_LEFT))
			{
				voronoiPts[x][y].emplace_back(xcenter - 0.25, ycenter - 0.75); // 1
				voronoiPts[x][y].emplace_back(xcenter - 0.75, ycenter - 0.25); // 2
			}
			else if(graph.edge(p->A(TOP),BOTTOM_LEFT))
				voronoiPts[x][y].emplace_back(xcenter - 0.25, ycenter - 0.25); // 3
			else voronoiPts[x][y].emplace_back(xcenter - 0.5, ycenter - 0.5); // 4

			//LEFT
			voronoiPts[x][y].emplace_back(xcenter - 0.5, ycenter); // Mid-point

			//BOTTOMLEFT
			if(graph.edge(p,BOTTOM_LEFT))
			{
				voronoiPts[x][y].emplace_back(xcenter - 0.75, ycenter + 0.25); // 1
				voronoiPts[x][y].emplace_back(xcenter - 0.25, ycenter + 0.75); // 2
			}
			else if(graph.edge(p->A(BOTTOM),TOP_LEFT))
				voronoiPts[x][y].emplace_back(xcenter - 0.25, ycenter + 0.25); // 3
			else voronoiPts[x][y].emplace_back(xcenter - 0.5, ycenter + 0.5); // 4

			//BOTTOM
			voronoiPts[x][y].emplace_back(xcenter, ycenter + 0.5); // Mid-point

			//BOTTOMRIGHT
			if(graph.edge(p, BOTTOM_RIGHT))
			{
				voronoiPts[x][y].emplace_back(xcenter + 0.25, ycenter + 0.75); // 1
				voronoiPts[x][y].emplace_back(xcenter + 0.75, ycenter + 0.25); // 2
			}
			else if(graph.edge(p->A(BOTTOM),TOP_RIGHT))
				voronoiPts[x][y].emplace_back(xcenter + 0.25, ycenter + 0.25); // 3
			else voronoiPts[x][y].emplace_back(xcenter + 0.5, ycenter + 0.5); // 4

			//RIGHT
			voronoiPts[x][y].emplace_back(xcenter + 0.5, ycenter); // Mid-point

			//TOPRIGHT
			if(graph.edge(p, TOP_RIGHT))
			{
				voronoiPts[x][y].emplace_back(xcenter + 0.75, ycenter - 0.25); // 1
				voronoiPts[x][y].emplace_back(xcenter + 0.25, ycenter - 0.75); // 2
			}
			else if(graph.edge(p->A(TOP), BOTTOM_RIGHT))
				voronoiPts[x][y].emplace_back(xcenter + 0.25, ycenter - 0.25); // 3
			else voronoiPts[x][y].emplace_back(xcenter + 0.5, ycenter - 0.5); // 4

			//TOP
			voronoiPts[x][y].emplace_back(xcenter, ycenter - 0.5); // Mid-point

		}
	}
}

vector<Point> Voronoi::operator()(int i,int j)
{
	return voronoiPts[i][j];
}