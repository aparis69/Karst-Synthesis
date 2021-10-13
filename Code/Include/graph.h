#pragma once

#include "basics.h"
#include "geology.h"

enum class KeyPointType
{
	Sink,		// Entrance
	Spring,		// Exists
	Waypoint,	// Waypoints are located inside the bedrock.
	Deadend,	
	Chamber,
};

enum class TunnelType
{
	// Phreatic
	Tube,
	Bed,

	// Vadose
	Epikarst,
	Passage,
	Keyhole,
	Canyon,
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

class CostTerm
{
public:
	bool used;
	double weight;
public:
	inline explicit CostTerm() : used(false), weight(1.0) { }
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
	GeologicalParameters params;	//!< All geological parameters.

	CostTerm distanceCost;
	CostTerm fractureCost;
	CostTerm horizonCost;
	CostTerm permeabilityCost;
	double gamma;

public:
	explicit VolumetricGraph();

	double ComputeEdgeCost(const Vector3& p, const Vector3& pn) const;
	void InitializeCostGraph(const std::vector<KeyPoint>& keyPts, const GeologicalParameters& params);
	KarsticSkeleton ComputeKarsticSkeleton(const std::vector<KeyPoint>& keyPts) const;
	std::vector<std::vector<int>> AmplifyKarsticSkeleton(const std::vector<KeyPoint>& baseKeyPts, const std::vector<KeyPoint>& newKeyPts);
	std::vector<InternalKeyPoint> AddNewSamples(const std::vector<KeyPoint>& samples);

	inline void SetDistanceCost(const CostTerm& t) { distanceCost = t; }
	inline void SetFractureCost(const CostTerm& t) { fractureCost = t; }
	inline void SetHorizonCost(const CostTerm& t) { horizonCost = t; }
	inline void SetPermeabilityCost(const CostTerm& t) { horizonCost = t; }
	inline void SetGamma(double g) { gamma = g; }
	inline Vector3 GetSample(int i) const { return samples[i]; }

	void SaveSamples(const std::string& path) const;

protected:
	void SampleSpace();
	void BuildNearestNeighbourGraph();
};
