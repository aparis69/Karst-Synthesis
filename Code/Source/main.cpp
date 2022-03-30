/*
	If you have any question or problem to compile the code, you can contact me at:
	axel(dot)paris(at)liris(dot)cnrs(dot)fr
*/

#include "graph.h"
#include <fstream>

static void BakePoissonDistributionFiles()
{
	{
		std::vector<Vector3> pts;
		Box2D(Vector2(0), 1000.0).ToBox(-200, 50).Poisson(pts, 15.0, 1000000);
		std::cout << pts.size() << std::endl;

		std::ofstream fout("data3d.dat", std::ios::out | std::ios::binary);
		size_t size = pts.size();
		fout.write((char*)&size, sizeof(size));
		fout.write((char*)&pts[0], size * sizeof(Vector3));
		fout.close();
	}

	{
		std::vector<Vector2> pts;
		Box2D(Vector2(0), 1000.0).Poisson(pts, 15.0, 1000000);
		std::cout << pts.size() << std::endl;

		std::ofstream fout("data2d.dat", std::ios::out | std::ios::binary);
		size_t size = pts.size();
		fout.write((char*)&size, sizeof(size));
		fout.write((char*)&pts[0], size * sizeof(Vector2));
		fout.close();
	}

	std::cin.get();
}

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

	params.graphNeighbourCount = 32;
	params.graphNeighbourRadius = 100.0;
	params.graphPoissonRadius = 10.0;
}

static void GorgePermeability(std::vector<KeyPoint>& keyPts, GeologicalParameters& params)
{
	keyPts.push_back(KeyPoint(Vector3(-213.311, 36.3838, 329.722), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(146.786, 39.7847, -284.607), KeyPointType::Spring));

	params.sceneName = "permeability";

	params.heightfield = ScalarField2D(256, 256, Box2D(Vector2(0), 750.0), 0.0);
	params.elevationOffsetMin = 200.0;
	params.elevationOffsetMax = 50.0;

	params.permeabilityVols.push_back(PermeabilitySphere(Vector3(96.8492, 0.0, 126.804), 100.0, -1000.0));

	params.distanceCost = CostTerm(true, 1.0);
	params.horizonCost = CostTerm(false, 0.0);
	params.permeabilityCost = CostTerm(true, 1.0);
	params.fractureCost = CostTerm(false, 0.0);
	params.gamma = 2.0;

	params.graphNeighbourCount = 32;
	params.graphNeighbourRadius = 100.0;
	params.graphPoissonRadius = 10.0;
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

	params.graphNeighbourCount = 32;
	params.graphNeighbourRadius = 100.0;
	params.graphPoissonRadius = 10.0;
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
	for (const auto& p : waypoints)
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

	params.graphNeighbourCount = 32;
	params.graphNeighbourRadius = 100.0;
	params.graphPoissonRadius = 10.0;
}

static void RectilinearMazeNetwork(std::vector<KeyPoint>& keyPts, GeologicalParameters& params)
{
	keyPts.push_back(KeyPoint(Vector3(60.4311, 52.6541, 261.446), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(-101.237, 52.1485, 219.241), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(-255.377, 44.3283, 138.243), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(-289.637, 37.8691, 358.579), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(-86.1126, 37.8371, 382.429), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(-446.468, 30.5388, 383.381), KeyPointType::Sink));
	keyPts.push_back(KeyPoint(Vector3(328.728, 48.0858, 414.128), KeyPointType::Sink));

	keyPts.push_back(KeyPoint(Vector3(225.946, 37.5715, -324.888), KeyPointType::Spring));
	keyPts.push_back(KeyPoint(Vector3(95.9676, 37.9612, -337.74), KeyPointType::Spring));
	keyPts.push_back(KeyPoint(Vector3(-110.653, 35.7709, -373.92), KeyPointType::Spring));
	keyPts.push_back(KeyPoint(Vector3(-275.73, 34.0925, -314.632), KeyPointType::Spring));
	keyPts.push_back(KeyPoint(Vector3(472.035, 46.3118, -456.167), KeyPointType::Spring));

	keyPts.push_back(KeyPoint(Vector3(-40, 45, -42), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(-262.777, 30.5283, -35), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(181.195, 163.99, -55), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(-149.457, 301.661, -44), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(311.746, -129.463, -35), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(0, 4.88059e-313, -25), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(412.401, 57.8162, -14), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(0, 4.88059e-313, -65), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(99.8656, 1.7859, -17), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(0, 4.88059e-313, -26), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(184.712, 336.452, -26), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(0, 4.88059e-313, -40), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(272.419, 266.214, -32), KeyPointType::Waypoint));
	keyPts.push_back(KeyPoint(Vector3(0, 4.88059e-313, -29), KeyPointType::Waypoint));

	params.sceneName = "rectilinear";

	params.heightfield = ScalarField2D(256, 256, Box2D(Vector2(0), 750.0), 0.0);
	params.elevationOffsetMin = 200.0;
	params.elevationOffsetMax = 50.0;

	params.fractures.push_back(FractureOrientation(Vector3(1, 0, 0), 1.0));
	params.fractures.push_back(FractureOrientation(Vector3(0, 1, 0), 1.0));
	params.fractures.push_back(FractureOrientation(Vector3(0, 0, 1), 1.0));
	params.fractures.push_back(FractureOrientation(Vector3(-1, 0, 0), 1.0));
	params.fractures.push_back(FractureOrientation(Vector3(0, -1, 0), 1.0));
	params.fractures.push_back(FractureOrientation(Vector3(0, 0, -1), 1.0));

	params.distanceCost = CostTerm(true, 1.0);
	params.horizonCost = CostTerm(false, 0.0);
	params.permeabilityCost = CostTerm(false, 0.0);
	params.fractureCost = CostTerm(true, 10.0);
	params.gamma = 2.0;

	params.graphNeighbourCount = 32;
	params.graphNeighbourRadius = 100.0;
	params.graphPoissonRadius = 10.0;
}


static long long timeInit = 0;
static long long timeSkel = 0;
static long long timeAmpl = 0;

void ComputeAndSaveSkeleton(GeologicalParameters params, std::vector<KeyPoint>& keyPts)
{
	// Compute 3D cost graph
	MyChrono chrono;
		VolumetricGraph graph;
		graph.InitializeCostGraph(keyPts, params);
	timeInit += chrono.ElapsedMs();

	// Compute karstic skeleton
	chrono.Restart();
		KarsticSkeleton skel = graph.ComputeKarsticSkeleton(keyPts);
	timeSkel += chrono.ElapsedMs();

	// Procedural amplification
	chrono.Restart();
		skel.Amplify(&graph, params.additionalKeyPts);
	timeAmpl += chrono.ElapsedMs();

	// Save
	skel.SaveDAT(params.sceneName);
	skel.SaveObj(params.sceneName);
}

int main()
{
	srand(1234);

	//BakePoissonDistributionFiles();
	VolumetricGraph::LoadPoissonSampleFile();

	{
		GeologicalParameters params;
		std::vector<KeyPoint> keyPts;
		GorgeNetwork(keyPts, params);
		ComputeAndSaveSkeleton(params, keyPts);
	}
	std::cout << "Scene gorge -- ok" << std::endl;

	{
		GeologicalParameters params;
		std::vector<KeyPoint> keyPts;
		GorgePermeability(keyPts, params);
		ComputeAndSaveSkeleton(params, keyPts);
	}
	std::cout << "Scene permeability -- ok" << std::endl;

	{
		GeologicalParameters params;
		std::vector<KeyPoint> keyPts;
		SuperimposedNetwork(keyPts, params);
		ComputeAndSaveSkeleton(params, keyPts);
	}
	std::cout << "Scene superimposed -- ok" << std::endl;

	{
		GeologicalParameters params;
		std::vector<KeyPoint> keyPts;
		SpongeworkNetwork(keyPts, params);
		ComputeAndSaveSkeleton(params, keyPts);
	}
	std::cout << "Scene spongework -- ok" << std::endl;

	{
		GeologicalParameters params;
		std::vector<KeyPoint> keyPts;
		RectilinearMazeNetwork(keyPts, params);
		ComputeAndSaveSkeleton(params, keyPts);
	}
	std::cout << "Scene rectilinear maze -- ok" << std::endl;

	std::cout << std::endl;
	std::cout << "--------------------" << std::endl;
	std::cout << "Total Time init: " << timeInit << "ms" << std::endl;
	std::cout << "Total Time skeleton computation: " << timeSkel << "ms" << std::endl;
	std::cout << "Total Time amplification: " << timeAmpl << "ms" << std::endl;
	//std::cin.get();

	return 0;
}
