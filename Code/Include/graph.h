#pragma once

#include "basics.h"
#include "geology.h"

class KarsticSection
{
public:
	int destIndex;
	TunnelType type;
	double radius;

	inline bool operator==(int i) const
	{
		return (destIndex == i);
	}
};

class KarsticNode
{
public:
	int index;
	Vector3 p;
	std::vector<KarsticSection> connections;

	inline bool operator==(int i) const
	{
		return (index == i);
	}
};

class VolumetricGraph;
class KarsticSkeleton
{
private:
	std::vector<KarsticNode> nodes;

public:
	inline explicit KarsticSkeleton() { }
	KarsticSkeleton(const VolumetricGraph* graph, const std::vector<std::vector<int>>& paths);

	void Amplify(VolumetricGraph* graph, const std::vector<KeyPoint>& baseKeyPts, std::vector<KeyPoint>& keyPts);
	Box GetBox() const;
	void Save(const std::string& file) const;

private:
	int GetInternalIndex(int index) const;
	std::vector<KeyPoint> GenerateKeyPoints() const;
	void AppendPaths(const VolumetricGraph* graph, const std::vector<std::vector<int>>& paths);
};


class CostGraph
{
protected:
	class GraphEdge
	{
	public:
		int target;			//!< Target node index.
		double weight;		//!< Weight of the edge.
	public:
		inline explicit GraphEdge(int t, double w) : target(t), weight(w)
		{
		}
	};

protected:
	std::vector<std::vector<GraphEdge>> adj; //!< Adjacency graph.
public:
	explicit CostGraph(int n);

protected:
	void SetEdge(int s, int t, double w);
	void DijkstraComputePaths(int, std::vector<double>&, std::vector<int>&) const;
	std::vector<int> DijkstraGetShortestPathTo(int, const std::vector<int>&, const std::vector<double>&, double&) const;
};

class VolumetricGraph : public CostGraph
{
protected:
	struct InternalKeyPoint
	{
		int index;
		Vector3 p;
		KeyPointType type;
	};
	int NodeIndex(const Vector3& p) const;

protected:
	std::vector<Vector3> samples;	//!< Graph nodes.
	GeologicalParameters params;	//!< All geological parameters.

public:
	explicit VolumetricGraph();

	double ComputeEdgeCost(const Vector3& p, const Vector3& pn) const;
	void InitializeCostGraph(const std::vector<KeyPoint>& keyPts, const GeologicalParameters& params);
	KarsticSkeleton ComputeKarsticSkeleton(const std::vector<KeyPoint>& keyPts) const;
	std::vector<std::vector<int>> AmplifyKarsticSkeleton(const std::vector<KarsticNode>& baseKeyPts, const std::vector<KeyPoint>& newKeyPts);
	std::vector<InternalKeyPoint> AddNewSamples(const std::vector<KeyPoint>& samples);

	inline Vector3 GetSample(int i) const { return samples[i]; }
	void SaveSamples(const std::string& path) const;

protected:
	void SampleSpace();
	void BuildNearestNeighbourGraph();
};
