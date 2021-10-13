/*
	If you have any question or problem to compile the code, you can contact me at:
	axel(dot)paris(at)liris(dot)cnrs(dot)fr
*/

#include "graph.h"

int main()
{
	srand(1234);

	// Initial heightfield
	GeologicalParameters params;
	params.heightfield = ScalarField2D(256, 256, Box2D(Vector2(0), 500), 0.0);
	params.elevationOffsetMin = 200;
	params.elevationOffsetMax = 50;
	
	// Key points
	std::vector<KeyPoint> baseKeyPoints;
	for (int i = 0; i < 4; i++)
	{
		Vector2 p = params.heightfield.GetBox().RandomInside();
		double y = params.heightfield.GetValueBilinear(p);
		baseKeyPoints.push_back(KeyPoint(Vector3(p.x, y, p.y), KeyPointType::Sink));
	}
	for (int i = 0; i < 4; i++)
	{
		Vector2 p = params.heightfield.GetBox().RandomInside();
		double y = params.heightfield.GetValueBilinear(p) - Random::Uniform(-30, -200);
		baseKeyPoints.push_back(KeyPoint(Vector3(p.x, y, p.y), KeyPointType::Spring));
	}
	for (int i = 0; i < 4; i++)
	{
		Vector2 p = params.heightfield.GetBox().RandomInside();
		double y = params.heightfield.GetValueBilinear(p) - Random::Uniform(-30, -100);
		baseKeyPoints.push_back(KeyPoint(Vector3(p.x, y, p.y), KeyPointType::Waypoint));
	}

	// Geological parameters of the scene: horizons, permeability, fractures
	params.poissonRadius = 10.0;
	params.horizons.push_back(-75.0);

	// Compute 3D cost graph
	VolumetricGraph graph;
	graph.InitializeCostGraph(baseKeyPoints, params);
	//graph.SaveSamples("samples.ply");

	// Compute karstic skeleton
	KarsticSkeleton skel = graph.ComputeKarsticSkeleton(baseKeyPoints);

	// Procedural amplification
	std::vector<KeyPoint> newKeyPts;
	skel.Amplify(&graph, baseKeyPoints, newKeyPts);

	skel.Save("test");

	std::cin.get();
	return 0;
}
