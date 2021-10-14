/*
	If you have any question or problem to compile the code, you can contact me at:
	axel(dot)paris(at)liris(dot)cnrs(dot)fr
*/

#include "graph.h"

static void GorgeNetwork(std::vector<KeyPoint>& keyPts, GeologicalParameters& params)
{
	keyPts.push_back(KeyPoint(Vector3(-213.311, 36.3838, 329.722), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(146.786, 39.7847, -284.607), KeyPointType::Spring));

	params.sceneName = "gorge";

	params.heightfield = ScalarField2D(256, 256, Box2D(Vector2(0), 750.0), 0.0);
	params.elevationOffsetMin = 200.0;
	params.elevationOffsetMax = 50.0;

	params.distanceCost = CostTerm(true, 1.0);
	params.horizonCost = CostTerm(false, 0.0);
	params.permeabilityCost = CostTerm(false, 0.0);
	params.fractureCost = CostTerm(false, 0.0);
	params.gamma = 2.0;

	params.poissonRadius = 10.0;
}

static void SuperimposedNetwork(std::vector<KeyPoint>& keyPts, GeologicalParameters& params)
{
	keyPts.push_back(KeyPoint(Vector3(-187.617, 36.9296, 346.747), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(-101.864, -15, -146.959), KeyPointType::Spring));
	keyPts.push_back(KeyPoint(Vector3(-238.081, -120, -355.443), KeyPointType::Spring));
	keyPts.push_back(KeyPoint(Vector3(-247.45, -15, 227.741), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(-247.45, -120, 227.741), KeyPointType::Waypoint));

	params.additionalKeyPts.push_back(KeyPoint(Vector3(-74.1156, -20, 191.096), KeyPointType::Deadend));
	params.additionalKeyPts.push_back(KeyPoint(Vector3(-107.803, -18.5, 135.67), KeyPointType::Deadend));
	params.additionalKeyPts.push_back(KeyPoint(Vector3(-74.327, -10.0, 91.419), KeyPointType::Deadend));
	params.additionalKeyPts.push_back(KeyPoint(Vector3(-88.0683, -16, -67.8587), KeyPointType::Deadend));
	params.additionalKeyPts.push_back(KeyPoint(Vector3(-262.721, -115, -123.816), KeyPointType::Deadend));
	params.additionalKeyPts.push_back(KeyPoint(Vector3(-275.736, -125, -161.977), KeyPointType::Deadend));
	params.additionalKeyPts.push_back(KeyPoint(Vector3(-251.231, -122, -225.205), KeyPointType::Deadend));
	params.additionalKeyPts.push_back(KeyPoint(Vector3(-263.919, -121, 161.113), KeyPointType::Deadend));
	params.additionalKeyPts.push_back(KeyPoint(Vector3(-233.894, -8, 87.3259), KeyPointType::Deadend));
	params.additionalKeyPts.push_back(KeyPoint(Vector3(-240.857, -111, 17.8932), KeyPointType::Deadend));
	params.additionalKeyPts.push_back(KeyPoint(Vector3(-143.636, -120, -19.1732), KeyPointType::Deadend));

	params.sceneName = "superimposed";

	params.heightfield = ScalarField2D(256, 256, Box2D(Vector2(0), 1000.0), 0.0);
	params.elevationOffsetMin = 200;
	params.elevationOffsetMax = 50;

	params.horizons.push_back(-15);
	params.horizons.push_back(-120.0);

	params.distanceCost = CostTerm(true, 1.0);
	params.horizonCost = CostTerm(true, 20.0);
	params.permeabilityCost = CostTerm(false, 0.0);
	params.fractureCost = CostTerm(false, 0.0);
	params.gamma = 2.0;

	params.poissonRadius = 10.0;
}

static void SpongeworkNetwork(std::vector<KeyPoint>& keyPts, GeologicalParameters& params)
{
	keyPts.push_back(KeyPoint(Vector3(-191.418, 38.9, 246.751), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(35.3645, 46.0193, 310.083), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(113.912, 45.784, -39.8546), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(-113.556, 46.9681, -253.197), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(-176.425, 23.203, -32.8452), KeyPointType::Sink));

	keyPts.push_back(KeyPoint(Vector3(-101.864, -15, -146.959), KeyPointType::Spring));
	keyPts.push_back(KeyPoint(Vector3(-238.081, -120, -355.443), KeyPointType::Spring));

	std::vector<Vector3> waypoints;
	Box2D(Vector2(0), 200.0).ToBox(-200.0, 0.0).Poisson(waypoints, 15.0, 50);
	for (auto p : waypoints)
		keyPts.push_back(KeyPoint(p, Random::Uniform() > 0.65 ? KeyPointType::Waypoint : KeyPointType::Deadend));

	params.sceneName = "spongework";

	params.heightfield = ScalarField2D(256, 256, Box2D(Vector2(0), 750.0), 0.0);
	params.elevationOffsetMin = 200.0;
	params.elevationOffsetMax = 50.0;

	params.distanceCost = CostTerm(true, 1.0);
	params.horizonCost = CostTerm(false, 0.0);
	params.permeabilityCost = CostTerm(false, 0.0);
	params.fractureCost = CostTerm(false, 0.0);
	params.gamma = 1.05;

	params.poissonRadius = 8.0;
}

static void RectilinearMazeNetwork(std::vector<KeyPoint>& keyPts, GeologicalParameters& params)
{
	// TODO
}


void ComputeAndSaveSkeleton(GeologicalParameters params, std::vector<KeyPoint>& keyPts)
{
	// Compute 3D cost graph
	VolumetricGraph graph;
	graph.InitializeCostGraph(keyPts, params);

	// Compute karstic skeleton
	KarsticSkeleton skel = graph.ComputeKarsticSkeleton(keyPts);

	// Procedural amplification
	skel.Amplify(&graph, keyPts, params.additionalKeyPts);

	// Save
	skel.Save(params.sceneName);
}

int main()
{
	srand(1234);

	GeologicalParameters params;
	std::vector<KeyPoint> keyPts;

	//GorgeNetwork(baseKeyPoints, params);
	//SuperimposedNetwork(keyPts, params);
	SpongeworkNetwork(keyPts, params);

	ComputeAndSaveSkeleton(params, keyPts);

	std::cin.get();
	return 0;
}
