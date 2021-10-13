#pragma once

#include "basics.h"

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
	double poissonRadius;

	std::vector<double> horizons;
	std::vector<PermeabilitySphere> permeabilityVols;
	
	ScalarField2D heightfield;
	double elevationOffsetMin;
	double elevationOffsetMax;
};
