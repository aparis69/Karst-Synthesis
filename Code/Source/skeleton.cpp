#include "graph.h"
#include <fstream>		// for saving the skeleton into a text file
#include <map>			// std::map

/*!
\brief Constructor from a set of path, and the volumetric nearest neighbour graph.
\param graph nearest neighbour graph
\param paths all paths in the skeleton
*/
KarsticSkeleton::KarsticSkeleton(const VolumetricGraph* graph, const std::vector<std::vector<int>>& paths)
{
	AppendPaths(graph, paths);
}

/*!
\brief Amplify an existing network from a new set of key points.
Calling this method with an empty parameter will procedurally place ten key points in the scene, and amplify the skeleton in the same way.
\param keyPts new key points.
*/
void KarsticSkeleton::Amplify(VolumetricGraph* graph, const std::vector<KeyPoint>& baseKeyPts, std::vector<KeyPoint>& newKeyPts)
{
	// if keyPts is empty, generate some procedurally
	if (newKeyPts.empty())
		newKeyPts = GenerateKeyPoints();

	// First, we must add the new key points to the graph and compute their neighbours
	graph->AddNewSamples(newKeyPts);

	// Then, we link the new samples to the existing network
	auto newPaths = graph->AmplifyKarsticSkeleton(nodes, newKeyPts);

	// Update the internal karstic skeleton struture
	AppendPaths(graph, newPaths);
}

/*
\brief Generates a set of key points inside karstic skeleton domain.
Key points can be interior waypoints, or deadends
*/
std::vector<KeyPoint> KarsticSkeleton::GenerateKeyPoints() const
{
	// Compute bounding box of the skeleton
	Box box = GetBox();

	// Compute random points inside the box
	std::vector<KeyPoint> keyPts;
	for (int i = 0; i < 10; i++)
	{
		Vector3 p = box.RandomInside();
		KeyPointType type = Random::Uniform() < 0.5 ? KeyPointType::Deadend : KeyPointType::Waypoint;
		keyPts.push_back(KeyPoint(p, type));
	}
	return keyPts;
}

/*!
\brief Computes the bounding box of the skeleton
*/
Box KarsticSkeleton::GetBox() const
{
	std::vector<Vector3> p;
	p.reserve(nodes.size());
	for (int i = 0; i < nodes.size(); i++)
		p.push_back(nodes[i].p);
	return Box(p);
}

/*!
\brief Utility function for finding the index of a graph node in the karstic skeleton array.
\param nodeIndex index in the graph (index of the sample)
*/
int KarsticSkeleton::GetInternalIndex(int nodeIndex) const
{
	for (int i = 0; i < nodes.size(); i++)
	{
		if (nodes[i].index == nodeIndex)
			return i;
	}
	return -1;
}

/*!
\brief
*/
void KarsticSkeleton::AppendPaths(const VolumetricGraph* graph, const std::vector<std::vector<int>>& paths)
{
	for (int i = 0; i < paths.size(); i++)	// Nodes
	{
		for (int j = 0; j < paths[i].size(); j++)
		{
			int nodeIndex = paths[i][j];
			auto it = std::find(nodes.begin(), nodes.end(), nodeIndex);
			if (it == nodes.end())
			{
				Vector3 p = graph->GetSample(nodeIndex);
				nodes.push_back({ nodeIndex, p });
			}
		}
	}
	for (int i = 0; i < nodes.size(); i++)		// Edges
	{
		int nodeIndex = nodes[i].index;
		std::vector<int> edges;
		for (int j = 0; j < paths.size(); j++)
		{
			for (int k = 0; k < paths[j].size(); k++)
			{
				int n = paths[j][k];
				if (nodeIndex != n)
					continue;
				if (k < paths[j].size() - 1)
				{
					int nAfter = paths[j][k + 1];
					int neiIndex = GetInternalIndex(nAfter);
					if (std::find(edges.begin(), edges.end(), neiIndex) == edges.end())
						edges.push_back(neiIndex);
				}
			}
		}
		for (int j = 0; j < edges.size(); j++)
		{
			auto newSection = KarsticSection({ edges[j], TunnelType::Tube, 0.0 });
			if (std::find(nodes[i].connections.begin(), nodes[i].connections.end(), edges[j]) == nodes[i].connections.end())
				nodes[i].connections.push_back(newSection);
		}
	}
}

/*!
\brief Export the karstic skeleton using a simple format.
\param file filename
*/
void KarsticSkeleton::Save(const std::string& file) const
{
	std::ofstream out;
	out.open(file + "_nodes.dat");
	if (out.is_open() == false)
	{
		std::cout << "Cannot save skeleton to file (nodes): " << file << std::endl;
		return;
	}
	std::map<int, int> nodeIndexMapping;
	for (int i = 0; i < nodes.size(); i++)
	{
		Vector3 nodePos = nodes[i].p;
		nodeIndexMapping.insert(std::make_pair(nodes[i].index, i + 1));
		out << nodePos[0] << " " << nodePos[1] << " " << nodePos[2] << "\n";
	}
	out.flush();
	out.close();

	out.open(file + "_links.dat");
	if (out.is_open() == false)
	{
		std::cout << "Cannot save skeleton to file (links): " << file << std::endl;
		return;
	}
	for (int i = 0; i < nodes.size(); i++)
	{
		int indexA = i + 1;
		for (int j = 0; j < nodes[i].connections.size(); j++)
		{
			int k = nodes[i].connections[j].destIndex;
			out << indexA << " " << (k + 1) << "\n";
		}
	}
	out.flush();
	out.close();
}
