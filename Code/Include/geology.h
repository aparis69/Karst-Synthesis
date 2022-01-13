#pragma once

#include "basics.h"

enum class KeyPointType
{
	Sink,		// Entrance
	Spring,		// Exit
	Waypoint,	// Waypoints are located inside the bedrock.
	Deadend,
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
	inline explicit CostTerm() : used(false), weight(1.0) { }
	inline explicit CostTerm(bool u, double w) : used(u), weight(w) { }
};

class PermeabilitySphere : public Sphere
{
public:
	double e;

public:
	inline PermeabilitySphere(const Vector3& c, double r, double e) : Sphere(c, r), e(e)
	{		
	}

	inline double Intensity(const Vector3& p) const
	{
		return e * Math::CubicSmoothCompact(SquaredMagnitude(p - center), radius * radius);
	}
};

class FractureOrientation
{
public:
	Vector3 d;
	double w;

public:
	inline FractureOrientation(const Vector3& dd, double ww)
	{
		d = dd;
		w = ww;
	}

	inline double Cost(const Vector3& dir) const
	{
		double dot = Dot(d, dir);
		return (1.0 - Math::CubicSmoothCompact(dot * dot, 1.0)) * 2.0 - 1.0;
	}
};

class GeologicalParameters
{
public:
	std::string sceneName;

	double graphPoissonRadius;
	double graphNeighbourRadius;
	int graphNeighbourCount;

	std::vector<double> horizons;
	std::vector<PermeabilitySphere> permeabilityVols;
	std::vector<FractureOrientation> fractures;

	ScalarField2D heightfield;
	double elevationOffsetMin;
	double elevationOffsetMax;

	CostTerm distanceCost;
	CostTerm fractureCost;
	CostTerm horizonCost;
	CostTerm permeabilityCost;
	double gamma;
	
	std::vector<KeyPoint> additionalKeyPts;
};
