#pragma once

#include "basics.h"

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

class CostGraph
{
protected:
	std::vector<std::vector<GraphEdge>> adj; //!< Adjacency graph.
public:
	explicit CostGraph(int n);
	void SetEdge(int s, int t, double w);
	void DijkstraComputePaths(int, std::vector<double>&, std::vector<int>&) const;
	std::vector<int> DijkstraGetShortestPathTo(int, const std::vector<int>&) const;
};

enum class KeyPointType
{
	Sink = 0,
	Spring,
	Waypoint,
	Deadend,
	Chamber,
};

class KeyPoint
{
public:
	Vector3 p;
	KeyPointType type;
public:
	inline explicit KeyPoint() { }
	inline explicit KeyPoint(const Vector3& p, KeyPointType t) : p(p), type(t)
	{
	}
};

class VolumetricGraph : public CostGraph
{
protected:
	std::vector<Vector3> samples;	//!< Graph nodes.
public:
	explicit VolumetricGraph();
	double ComputeEdgeCost(const Vector3& p, const Vector3& pn) const;
	std::vector<int> ComputeCostGraph(const std::vector<KeyPoint>& keyPts);
};
