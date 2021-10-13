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
	params.elevationOffsetMin = 200;
	params.elevationOffsetMax = 50;
	
	std::vector<KeyPoint> keyPoints;
	for (int i = 0; i < 4; i++)
	{
		Vector2 p = params.heightfield.GetBox().RandomInside();
		double y = params.heightfield.GetValueBilinear(p);
		keyPoints.push_back(KeyPoint(Vector3(p.x, y, p.y), KeyPointType::Sink));
	}
	for (int i = 0; i < 4; i++)
	{
		Vector2 p = params.heightfield.GetBox().RandomInside();
		double y = params.heightfield.GetValueBilinear(p) - Random::Uniform(-30, -200);
		keyPoints.push_back(KeyPoint(Vector3(p.x, y, p.y), KeyPointType::Spring));
	}
	for (int i = 0; i < 4; i++)
	{
		Vector2 p = params.heightfield.GetBox().RandomInside();
		double y = params.heightfield.GetValueBilinear(p) - Random::Uniform(-30, -100);
		keyPoints.push_back(KeyPoint(Vector3(p.x, y, p.y), KeyPointType::Waypoint));
	}

	// Geological parameters of the scene: horizons, permeability, fractures
	params.poissonRadius = 10.0;
	params.horizons.push_back(-75.0);

	// Compute karstic skeleton
	VolumetricGraph graph;
	graph.InitializeCostGraph(keyPoints, params);
	//graph.SaveSamples("samples.ply");

	KarsticSkeleton skel = graph.ComputeKarsticSkeleton(keyPoints);
	skel.Save("test");

	return 0;
}
