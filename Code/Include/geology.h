#pragma once

#include "basics.h"

class PermeabilitySphere
{
public:
	Vector3 c;
	double r;
	double e;

public:
	double Intensity(const Vector3& p) const
	{
		return e * Math::CubicSmoothCompact(SquaredMagnitude(p - c), r * r);
	}
};

class GeologicalParameters
{
public:
	double poissonRadius;

	std::vector<double> horizons;
	std::vector<PermeabilitySphere> permeability;
	ScalarField2D heightfield;
};
