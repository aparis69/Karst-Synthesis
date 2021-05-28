/*
	If you have any questions or problem to compile the code, you can contact me at:
	axel(dot)paris(at)liris(dot)cnrs(dot)fr
*/

#include "graph.h"

int main()
{
	srand(1234);

	ScalarField2D hf = ScalarField2D(256, 256, Box2D(Vector2(0), 500), 0.0);
	std::vector<KeyPoint> keyPoints;
	keyPoints.push_back(KeyPoint(Vector3(0), KeyPointType::Sink));
	keyPoints.push_back(KeyPoint(Vector3(0, -150.0, -100.0), KeyPointType::Spring));

	VolumetricGraph graph;
	graph.ComputeCostGraph(keyPoints, hf);

	KarsticSkeleton skel = graph.ComputeKarsticSkeleton(keyPoints);
	skel.Save("test");

	return 0;
}
