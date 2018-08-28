#include <iostream>
#include <array>
#include <vector>

#include "kdtree.h"

using namespace std;

class map
{
public:
	map() {
		std::vector<vector<double>> points;
		points.push_back({1,1});
		
		// build k-d tree
		kdtree.MakeKDTree(points);
	}

	kdt::KDTree<vector<double>> kdtree;
	
};

int main(int argc, char **argv)
{
	
	// generate points
	// std::vector<vector<double>> points;
	// points.push_back({1,1});
	
	// // build k-d tree
	// kdt::KDTree<vector<double>> kdtree(points);

	vector<double> query = {5,5};
	// const std::vector<int> knnIndices = kdtree.knnSearch(query, 1);
	// cout << knnIndices[0] << endl;
	// cout << knnIndices.size()<< endl;
	
	// // radius search
	const double radius = 20;
	// std::vector<int> radIndices = kdtree.radiusSearch(query, radius);
	// cout << radIndices.size()<< endl;
	map mp;
	std::vector<int> d = mp.kdtree.radiusSearch(query, radius);
	cout << d.size()<< endl;

	return 0;
}