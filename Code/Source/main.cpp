/*
	If you have any question or problem to compile the code, you can contact me at:
	axel(dot)paris(at)liris(dot)cnrs(dot)fr
*/

#include "graph.h"

int main()
{
	srand(1234);

	// Initial heightfield and key points
	GeologicalParameters params;
	params.heightfield = ScalarField2D(256, 256, Box2D(Vector2(0), 500), 0.0);
	std::vector<KeyPoint> keyPoints;
	keyPoints.push_back(KeyPoint(Vector3(0), KeyPointType::Sink));
	keyPoints.push_back(KeyPoint(Vector3(0, -150.0, -100.0), KeyPointType::Spring));

	// Geological parameters of the scene: horizons, permeability, fractures
	params.poissonRadius = 10.0;
	params.horizons.push_back(-50.0);

	// Compute karstic skeleton
	VolumetricGraph graph;
	graph.ComputeCostGraph(keyPoints, params);

	KarsticSkeleton skel = graph.ComputeKarsticSkeleton(keyPoints);
	skel.Save("test");

	return 0;
}
