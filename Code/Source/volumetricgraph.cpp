#include "graph.h"
#include <algorithm> // for std::sort

/*
\brief
*/
VolumetricGraph::VolumetricGraph() : CostGraph(0)
{
}

/*
\brief
*/
double VolumetricGraph::ComputeEdgeCost(const Vector3& p, const Vector3& pn) const
{
	return 0.0;
}

/*
\brief
*/
void VolumetricGraph::ComputeCostGraph(const std::vector<KeyPoint>& keyPts, const ScalarField2D& hf)
{
	// Key points are added as samples
	for (int i = 0; i < keyPts.size(); i++)
		samples.push_back(keyPts[i].p);

	// TODO(Axel): sampling of the geological features, if any.

	Box box = hf.GetBox().ToBox(hf.Min(), hf.Max());
	box.Poisson(samples, 15.0, 1000000);

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
	double R = 50.0 * 50.0;
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
\brief
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

/*
\brief
*/
void VolumetricGraph::ComputeGammaSkeleton(const std::vector<KeyPoint>& pts) const
{
	// Internal representation of key points, storing their index.
	std::vector<InternalKeyPoint> keyPts;
	for (int i = 0; i < pts.size(); i++)
		keyPts.push_back({ NodeIndex(pts[i].p), pts[i].p, pts[i].type });
	
	// Allocates all arrays
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
		int s = keyPts[i].index;
		std::vector<double> distances;
		std::vector<int> previous;
		DijkstraComputePaths(s, distances, previous);

		for (int j = 0; j < keyPts.size(); j++)
		{
			// TODO(Axel): ignore useless path
			if (i == j) continue;

			int t = keyPts[j].index;
			double pathSize;
			std::vector<int> path = DijkstraGetShortestPathTo(t, previous, distances, pathSize);
			if (path.size() <= 1)
				continue;
			all_paths[i][j] = path;
			all_distances[i][j] = pathSize;
		}
	}

	// Path cleaning
	// TODO:
	//	-Remove duplicates

	// Path pruning based on an anisotropic empty region criterion
	std::vector<std::vector<int>> pathsFinal;
	const double gamma = 2.0;
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
}
