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
	std::vector<int> DijkstraGetShortestPathTo(int, const std::vector<int>&, const std::vector<double>&, double&) const;
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

class CostTerm
{
public:
	bool used;
	double weight;
public:
	inline explicit CostTerm() { }
	inline explicit CostTerm(bool u, double w) : used(u), weight(w) { }
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
	CostTerm distanceCost;
	CostTerm fractureCost;
	CostTerm horizonCost;
	CostTerm permeabilityCost;

public:
	explicit VolumetricGraph();

	double ComputeEdgeCost(const Vector3& p, const Vector3& pn) const;
	void ComputeCostGraph(const std::vector<KeyPoint>& keyPts, const ScalarField2D& hf);
	void ComputeGammaSkeleton(const std::vector<KeyPoint>& keyPts) const;

	inline void SetDistanceCost(const CostTerm& t) { distanceCost = t; }
	inline void SetFractureCost(const CostTerm& t) { fractureCost = t; }
	inline void SetHorizonCost(const CostTerm& t) { horizonCost = t; }
	inline void SetPermeabilityCost(const CostTerm& t) { horizonCost = t; }
};
