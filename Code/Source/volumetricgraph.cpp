#include "graph.h"
#include <algorithm> // for std::sort

/*
\brief Default Constructor.
*/
VolumetricGraph::VolumetricGraph() : CostGraph(0)
{
	distanceCost = CostTerm(true, 1.0);
}

/*
\brief Compute the cost at a given point, in a given direction.
*/
double VolumetricGraph::ComputeEdgeCost(const Vector3& p, const Vector3& pn) const
{
	Vector3 d = Normalize(pn - p);

	// Distance cost
	double cost = 0.0;
	if (distanceCost.used)
		cost += Magnitude(p - pn) * distanceCost.weight;

	// Horizon cost
	if (horizonCost.used)
	{
		double nearestDist = 1e6;
		for (int i = 0; i < params.horizons.size(); i++)
		{
			double d = Math::Abs((p[2] - params.horizons[i])) + Math::Abs((pn[2] - params.horizons[i]));
			if (d < nearestDist)
				nearestDist = d;
		}

		// TODO(axel): falloff with radius
		cost += nearestDist * horizonCost.weight;
	}

	// Permeability
	if (permeabilityCost.used)
	{
		double costPerm = 0.0;
		for (int i = 0; i < params.permeability.size(); i++)
			costPerm += params.permeability[i].Intensity(p);
		cost += costPerm * permeabilityCost.weight;
	}

	// Inside/outside
	{
		// TODO(Axel)
	}

	// Orientation
	{
		// TODO(Axel)
	}

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
Inception horizons are first sampled (as they are most important), then permeability volumes.
Rest of space is then filled with a Poisson sphere distribution.
*/
void VolumetricGraph::SampleSpace()
{
	// Horizon sampling
	for (int i = 0; i < params.horizons.size(); i++)
	{
		// TODO(Axel)
	}

	// Permeability sampling
	for (int i = 0; i < params.permeability.size(); i++)
	{
		// TODO(Axel)
	}

	// Poisson sampling 
	double zMin = params.heightfield.Min();
	double zMax = params.heightfield.Max();
	Box box = params.heightfield.GetBox().ToBox(zMin - 100.0, zMax + 50.0);
	box.Poisson(samples, params.poissonRadius, 500000);

	std::cout << "Sample count: " << samples.size() << std::endl;
}


/*
\brief Initialize the volumetric cost graph from a set of key points.
In our system, the graph of the domain is a nearest neighbour graph constructed from a Poisson sphere distribution in space.
\param keyPts the key points
\param hf the terrain
*/
void VolumetricGraph::ComputeCostGraph(const std::vector<KeyPoint>& keyPts, const GeologicalParameters& geologicalParams)
{
	params = geologicalParams;

	// Key points are added as samples
	for (int i = 0; i < keyPts.size(); i++)
		samples.push_back(keyPts[i].p);

	// Adaptive sampling of space
	SampleSpace();	

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
	double R = 200.0 * 200.0;
	int N = 32;
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

/*
\brief Computes a karstic skeleton from a set of key points. 
This function computes the complete graph between all key points, then prune the edges with a 3D gamma-skeleton approach.
Note that ComputeCostGraph() must be called before this function.
\param pts key points
*/
KarsticSkeleton VolumetricGraph::ComputeKarsticSkeleton(const std::vector<KeyPoint>& pts) const
{
	// Internal representation of key points, storing their index.
	std::vector<InternalKeyPoint> keyPts;
	for (int i = 0; i < pts.size(); i++)
		keyPts.push_back({ NodeIndex(pts[i].p), pts[i].p, pts[i].type });
	
	// Allocates temporary all arrays
	std::vector<std::vector<double>> all_distances;
	std::vector<std::vector<std::vector<int>>> all_paths;
	all_paths.resize(keyPts.size());
	all_distances.resize(keyPts.size());
	for (int i = 0; i < keyPts.size(); i++)
	{
		all_distances[i].resize(keyPts.size(), 1e6);
		all_paths[i].resize(keyPts.size());
	}

	// Compute complete graph between key points
	for (int i = 0; i < keyPts.size(); i++)
	{
		// We don't compute path starting from springs, which is actually not geomorphologically correct as
		// two springs can be linked together by a path sometimes. But, the resulting networks look better this way :-)
		if (keyPts[i].type == KeyPointType::Spring)
			continue;
		// We don't compute path starting from deadends, which is geomorphologically correct.
		if (keyPts[i].type == KeyPointType::Deadend)
			continue;

		int s = keyPts[i].index;
		std::vector<double> distances;
		std::vector<int> previous;
		DijkstraComputePaths(s, distances, previous);

		for (int j = 0; j < keyPts.size(); j++)
		{
			// Ignore same node
			if (i == j) continue;
			// Don't compute paths between sinks
			if (keyPts[i].type == KeyPointType::Sink && keyPts[j].type == KeyPointType::Sink) continue;

			int t = keyPts[j].index;
			double pathSize;
			std::vector<int> path = DijkstraGetShortestPathTo(t, previous, distances, pathSize);
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
			if (all_paths[i][j].size() <= 1)
				continue;
			if (all_paths[j][i].size() <= 1)
				continue;

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
			double dij = Math::Pow(all_distances[i][j], gamma);
			bool keep = true;
			for (int k = 0; k < all_paths[i].size(); k++)
			{
				if (i == k || j == k)			 continue;
				if (all_paths[i][k].size() <= 1) continue;
				if (all_paths[k][j].size() <= 1) continue;

				double dik = Math::Pow(all_distances[i][k], gamma);
				double dkj = Math::Pow(all_distances[k][j], gamma);
				if (dik + dkj < dij)
				{
					keep = false;
					break;
				}
			}
			if (keep)
				pathsFinal.push_back(all_paths[i][j]);
		}
	}
	std::cout << "Nb Paths: " << pathsFinal.size() << std::endl;

	// Build karstic skeleton structure
	return KarsticSkeleton(this, pathsFinal);
}

/*!
\brief Add new samples to the nearest neighbour graph structure. Existing samples are not removed, but their neighbours are modified.
\param samples new sample passed as key points
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
	double R = 50.0 * 50.0;
	int N = 32;
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

			// Forward cost (from key point to sample)
			SetEdge(index, candidates[j].i, ComputeEdgeCost(p, d));

			// Compute the reverse cost as well (from sample to key point)
			SetEdge(candidates[j].i, index, ComputeEdgeCost(pn, -d));
		}

		ret.push_back({ index, p, newSamples[i].type });
	}
	return ret;
}
