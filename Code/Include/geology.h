#pragma once

#include "basics.h"

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

class GeologicalParameters
{
public:
	std::string sceneName;

	double poissonRadius;

	std::vector<double> horizons;
	std::vector<PermeabilitySphere> permeabilityVols;

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
