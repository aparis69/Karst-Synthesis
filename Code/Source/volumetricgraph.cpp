#include "graph.h"
#include <algorithm> // for std::sort
#include <fstream>	 // for saving sample file

std::vector<Vector2> VolumetricGraph::bakedPoissonDistribution2D;
std::vector<Vector3> VolumetricGraph::bakedPoissonDistribution3D;

/*
\brief Default Constructor.
*/
VolumetricGraph::VolumetricGraph() : CostGraph(0)
{
}

/*
\brief Compute the cost between two points.
\param p start position
\param pn end position
*/
double VolumetricGraph::ComputeEdgeCost(const Vector3& p, const Vector3& pn) const
{
	Vector3 d = Normalize(pn - p);

	// Distance cost
	double cost = 0.0;
	if (params.distanceCost.used)
		cost += Magnitude(p - pn) * params.distanceCost.weight;

	// Horizon cost
	if (params.horizonCost.used)
	{
		double nearestDist = 1e6;
		for (int i = 0; i < params.horizons.size(); i++)
		{
			double dist = Math::Abs((p[2] - params.horizons[i]));
			if (dist < nearestDist)
				nearestDist = dist;
		}

		nearestDist = Math::Clamp(nearestDist, 0.0, 50.0);
		double w = (1.0 - Math::CubicSmooth(nearestDist, 50.0));
		cost += w * params.horizonCost.weight;
	}

	// Permeability
	if (params.permeabilityCost.used)
	{
		double costPerm = 0.0;
		for (const auto& sphere : params.permeabilityVols)
			costPerm += sphere.Intensity(p);
		cost += costPerm * params.permeabilityCost.weight;
	}

	// Fracture orientation
	if (params.fractureCost.used)
	{
		double costFrac = 0.0;
		for (const auto& f : params.fractures)
			costFrac += f.Cost(d);
		cost += costFrac * params.fractureCost.weight;
	}

	// Inside/outside
	{
		bool is_out = (params.heightfield.GetValueBilinear(Vector2(p.x, p.z)) < p.y);
		cost += is_out ? 100000.0 : 0;
	}

	cost = Math::Clamp(cost, 0.0, cost); // make sure to keep a positive cost for Dijsktra
	return cost;
}

/*
\brief Computes the internal index of a sample of the graph, or -1 if the sample does not exist.
\param p sample position
*/
int VolumetricGraph::NodeIndex(const Vector3& p) const
{
	for (int i = 0; i < samples.size(); i++)
	{
		if (samples[i] == p)
			return i;
	}
	return -1;
}

/*!
\brief Performs an adaptive sampling of the space based on provided geological features.
Inception horizons are first sampled, then permeability volumes.
Rest of space is then filled with a Poisson sphere distribution.
*/
void VolumetricGraph::SampleSpace()
{
	// Inception horizon sampling
	Box2D horizonBox = params.heightfield.GetBox();
	for (const auto& horizonZ : params.horizons)
	{
		// Procedural (slower)
		// std::vector<Vector2> horizonSamples;
		//horizonBox.Poisson(horizonSamples, params.graphPoissonRadius, 50000);

		// Baked
		for (const auto& p : bakedPoissonDistribution2D)
		{
			if (horizonBox.Contains(p) == false)
				continue;
			samples.push_back(Vector3(p.x, horizonZ, p.y));
		}
	}

	// Permeability volumes sampling
	for (const auto& permeabilitySphere : params.permeabilityVols)
		permeabilitySphere.Poisson(samples, params.graphPoissonRadius, 5000);

	// Poisson sampling for the rest of space
	double zMin = params.heightfield.Min();
	double zMax = params.heightfield.Max();
	Box box = params.heightfield.GetBox().ToBox(zMin - params.elevationOffsetMin, zMax + params.elevationOffsetMax);
		
	// Procedural distribution (slower)
	//box.Poisson(samples, params.graphPoissonRadius, 100000);

	// From baked distribution (faster)
	double c = 4.0 * params.graphPoissonRadius * params.graphPoissonRadius;
	for (int i = 0; i < bakedPoissonDistribution3D.size(); i++)
	{
		Vector3 t = bakedPoissonDistribution3D[i];
		bool hit = false;
		for (int j = 0; j < samples.size(); j++)
		{
			if (SquaredMagnitude(t - samples.at(j)) < c)
			{
				hit = true;
				break;
			}
		}
		if (hit == false)
			samples.push_back(t);
	}

	//std::cout << "Total sample count: " << samples.size() << std::endl;
}

/*!
\brief Builds the nearest neighbour graph from the set of samples.
*/
void VolumetricGraph::BuildNearestNeighbourGraph()
{
	// Build nearest neighbor graph
	struct Neighbour
	{
		int i;
		double d;
		bool operator<(const Neighbour& nei)
		{
			return d < nei.d;
		}
	};
	adj.resize(samples.size());
	const double R = params.graphNeighbourRadius * params.graphNeighbourRadius;
	const int N = params.graphNeighbourCount;

#pragma omp parallel for
	for (int i = 0; i < samples.size(); i++)
	{
		Vector3 p = samples[i];
		std::vector<Neighbour> candidates;
		for (int j = 0; j < samples.size(); j++)
		{
			if (i == j) continue;
			Vector3 pn = samples[j];
			double d = SquaredMagnitude(p - pn);
			if (d < R)
				candidates.push_back({ j, d });
		}

		std::sort(candidates.begin(), candidates.end());
		int n = Math::Min((int)candidates.size(), N);
		for (int j = 0; j < n; j++)
		{
			Vector3 pn = samples[candidates[j].i];
			SetEdge(i, candidates[j].i, ComputeEdgeCost(p, pn));
		}
	}
}

/*!
\brief Load baked poisson sphere/disc distribution from binary files.
*/
void VolumetricGraph::LoadPoissonSampleFile()
{
	{
		size_t size = 0;
		std::ifstream rf("../Data/poissonSamples3d.dat", std::ios::out | std::ios::binary);
		rf.read((char*)&size, sizeof(size));
		bakedPoissonDistribution3D.resize(size);
		for (int i = 0; i < size; i++)
			rf.read((char*)&bakedPoissonDistribution3D[i], sizeof(Vector3));
	}

	{
		size_t size = 0;
		std::ifstream rf("../Data/poissonSamples2d.dat", std::ios::out | std::ios::binary);
		rf.read((char*)&size, sizeof(size));
		bakedPoissonDistribution2D.resize(size);
		for (int i = 0; i < size; i++)
			rf.read((char*)&bakedPoissonDistribution2D[i], sizeof(Vector2));
	}
}


/*
\brief Initialize the volumetric cost graph from a set of key points and geological constraints.
In our system, the graph of the domain is a nearest neighbour graph constructed from a Poisson sphere distribution in R^3.
\param keyPts the key points
\param geologicalParams the set of geological constraints provided by the user
*/
void VolumetricGraph::InitializeCostGraph(const std::vector<KeyPoint>& keyPts, const GeologicalParameters& geologicalParams)
{
	params = geologicalParams;

	// Key points are added to sample set
	for (int i = 0; i < keyPts.size(); i++)
		samples.push_back(keyPts[i].p);

	// Adaptive sampling of space
	SampleSpace();

	// Nearest neighbour graph initialization
	BuildNearestNeighbourGraph();
}

/*
\brief Computes a karstic skeleton from a set of key points. 
This function computes the complete graph between all key points, then prunes the edges with a 3D gamma-skeleton approach.
Note that InitializeCostGraph() must be called before this function.
\param pts key points
*/
KarsticSkeleton VolumetricGraph::ComputeKarsticSkeleton(const std::vector<KeyPoint>& pts) const
{
	// Internal representation of key points, storing their index.
	std::vector<InternalKeyPoint> keyPts;
	for (int i = 0; i < pts.size(); i++)
		keyPts.push_back({ NodeIndex(pts[i].p), pts[i].p, pts[i].type });
	
	// Allocates all temporary arrays
	std::vector<std::vector<double>> all_distances;
	std::vector<std::vector<std::vector<int>>> all_paths;
	all_paths.resize(keyPts.size());
	all_distances.resize(keyPts.size());
	for (int i = 0; i < keyPts.size(); i++)
	{
		all_distances[i].resize(keyPts.size(), 1e6);
		all_paths[i].resize(keyPts.size());
	}

	// Compute complete graph between all key points
	for (int i = 0; i < keyPts.size(); i++)
	{
		// We don't compute path starting from springs, which is actually not geomorphologically correct as
		// two springs can be linked together by a path sometimes. But, the resulting networks look better this way :-)
		if (keyPts[i].type == KeyPointType::Spring)
			continue;
		// We don't compute path starting from deadends
		if (keyPts[i].type == KeyPointType::Deadend)
			continue;

		int source = keyPts[i].index;
		std::vector<double> distances;
		std::vector<int> previous;
		DijkstraComputePaths(source, distances, previous);

		for (int j = 0; j < keyPts.size(); j++)
		{
			// Ignore same node
			if (i == j) continue;
			// Ingore path between two sinks
			if (keyPts[i].type == KeyPointType::Sink && keyPts[j].type == KeyPointType::Sink) continue;

			int target = keyPts[j].index;
			double pathSize;
			std::vector<int> path = DijkstraGetShortestPathTo(target, previous, distances, pathSize);
			if (path.size() <= 1)
				continue;

			all_paths[i][j] = path;
			all_distances[i][j] = pathSize;
		}
	}

	// Remove duplicates
	for (int i = 0; i < all_paths.size(); i++)
	{
		for (int j = 0; j < all_paths.size(); j++)
		{
			if (all_paths[i][j].size() <= 1) continue;
			if (all_paths[j][i].size() <= 1) continue;

			// At this point, there is a path from i to j, and a path from j to i.
			// Delete the most expensive one
			if (all_distances[j][i] > all_distances[i][j])
				all_paths[j][i].clear();
			else
				all_paths[i][j].clear();
		}
	}

	// Path pruning based on an anisotropic empty region criterion
	std::vector<std::vector<int>> pathsFinal;
	for (int i = 0; i < all_paths.size(); i++)
	{
		for (int j = 0; j < all_paths[i].size(); j++)
		{
			if (all_paths[i][j].size() <= 1)
				continue;

			// Second, check if there is a cheaper path from nodes[i] going through a node[k] to arrive at nodes[j]
			double d_ij = Math::Pow(all_distances[i][j], params.gamma);
			bool keep = true;
			for (int k = 0; k < all_paths[i].size(); k++)
			{
				if (i == k || j == k)			 continue;
				if (all_paths[i][k].size() <= 1) continue;
				if (all_paths[k][j].size() <= 1) continue;

				double d_ik = Math::Pow(all_distances[i][k], params.gamma);
				double d_kj = Math::Pow(all_distances[k][j], params.gamma);
				if (d_ik + d_kj < d_ij)
				{
					keep = false;
					break;
				}
			}
			if (keep)
				pathsFinal.push_back(all_paths[i][j]);
		}
	}
	//std::cout << "Path count: " << pathsFinal.size() << std::endl;

	// Build karstic skeleton structure
	return KarsticSkeleton(this, pathsFinal);
}

/*!
\brief Amplify the graph from a new set of key points. The new points are linked to the existing base key points.
\param baseSkeletonNodes the existing network
\param newKeyPts the new set of key point to connect to the existing network
\return the set of added paths
*/
std::vector<std::vector<int>> VolumetricGraph::AmplifyKarsticSkeleton(const std::vector<KarsticNode>& baseSkeletonNodes, const std::vector<KeyPoint>& newKeyPts)
{
	// Our goal is to connect the new key points to the original network
	std::vector<std::vector<int>> pathsFinal;
	for (int i = 0; i < newKeyPts.size(); i++)
	{
		int startIndex = NodeIndex(newKeyPts[i].p);
		std::vector<double> distances;
		std::vector<int> previous;

		DijkstraComputePaths(startIndex, distances, previous);
		
		double minDist = 1e6;
		std::vector<int> bestPath;
		for (int j = 0; j < baseSkeletonNodes.size(); j++)
		{
			if (i == j)
				continue;
			int endIndex = baseSkeletonNodes[j].index;
			double totalDistance = 0.0;
			std::vector<int> path = DijkstraGetShortestPathTo(endIndex, previous, distances, totalDistance);
			if (path.size() <= 1)
				continue;
			if (totalDistance < minDist)
			{
				minDist = totalDistance;
				bestPath = path;
			}
		}
		pathsFinal.push_back(std::vector<int>(bestPath.begin(), bestPath.end()));
	}
	//std::cout << "Additional path count: " << pathsFinal.size() << std::endl;
	return pathsFinal;
}

/*!
\brief Add new samples to the nearest neighbour graph structure. 
Existing samples are not removed, but their neighbours are modified.
\param samples new samples passed as key points
*/
std::vector<VolumetricGraph::InternalKeyPoint> VolumetricGraph::AddNewSamples(const std::vector<KeyPoint>& newSamples)
{
	struct Neighbour
	{
		int i;
		double d;
		bool operator<(const Neighbour& nei)
		{
			return d < nei.d;
		}
	};
	const double R = params.graphNeighbourRadius * params.graphNeighbourRadius;
	const int N = params.graphNeighbourCount;
	std::vector<InternalKeyPoint> ret;
	for (int i = 0; i < newSamples.size(); i++)
	{
		Vector3 p = newSamples[i].p;
		samples.push_back(p);
		adj.push_back({});
		int index = (int)samples.size() - 1;

		std::vector<Neighbour> candidates;
		for (int j = 0; j < samples.size(); j++)
		{
			if (index == j) continue;
			Vector3 pn = samples[j];
			double d = SquaredMagnitude(p - pn);
			if (d < R)
				candidates.push_back({ j, d });
		}

		std::sort(candidates.begin(), candidates.end());
		int n = Math::Min((int)candidates.size(), N);
		for (int j = 0; j < n; j++)
		{
			Vector3 pn = samples[candidates[j].i];
			Vector3 d = Normalize(pn - p);

			// Forward cost (from new key point to existing sample)
			SetEdge(index, candidates[j].i, ComputeEdgeCost(p, d));

			// Backward cost (from existing sample to new key point)
			SetEdge(candidates[j].i, index, ComputeEdgeCost(pn, -d));
		}

		ret.push_back({ index, p, newSamples[i].type });
	}
	return ret;
}


/*!
\brief Export the sample set as a point set file.
\param path file path
*/
void VolumetricGraph::SaveSamples(const std::string& path) const
{
	std::ofstream out;
	out.open(path);

	// ply header
	out << "ply\n";
	out << "format ascii 1.0\n";
	out << "element vertex " << samples.size() << "\n";
	out << "property float x\n";
	out << "property float y\n";
	out << "property float z\n";
	out << "end_header\n";

	// data
	for (const auto& p : samples)
		out << p.x << " " << p.y << " " << p.z << "\n";

	out.close();
}
