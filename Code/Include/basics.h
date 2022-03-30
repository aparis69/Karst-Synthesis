#pragma once
#include "vec.h"
#include <time.h>

#include <vector>

// Random (Dirty, C-style)
class Random
{
public:
	/*!
	\brief Constructor.
	*/
	Random()
	{
		// Empty
	}

	/*!
	\brief Compute a random number in a given range.
	\param a min
	\param b max
	*/
	static inline double Uniform(double a, double b)
	{
		return a + (b - a) * Uniform();
	}

	/*!
	\brief Compute a uniform random number in [0, 1]
	*/
	static inline double Uniform()
	{
		return double(rand()) / RAND_MAX;
	}

	/*!
	\brief Compute a random positive integer.
	*/
	static inline int Integer()
	{
		return rand();
	}
};

// AABB 3D
class Box
{
protected:
	Vector3 a;
	Vector3 b;

public:
	inline Box() { }
	explicit Box(const Vector3& A, const Vector3& B);
	explicit Box(const Vector3& C, double R);
	explicit Box(const Box& b1, const Box& b2);
	explicit Box(const std::vector<Vector3>& pts);

	bool Contains(const Vector3&) const;
	Vector3 RandomInside() const;
	Vector3 Vertex(int) const;
	Vector3& operator[](int i);
	Vector3 operator[](int i) const;
	void Poisson(std::vector<Vector3>& samples, double r, int n) const;
};

/*
\brief Constructor
\param A lower left vertex in world coordinates
\param B upper right vertex in world coordinates
*/
inline Box::Box(const Vector3& A, const Vector3& B) : a(A), b(B)
{
}

/*
\brief Constructor
\param C box center
\param R radius
*/
inline Box::Box(const Vector3& C, double R)
{
	Vector3 RR = Vector3(R);
	a = C - RR;
	b = C + RR;
}

/*!
\brief Constructor from 2 boxes
\param b1 first box
\param b2 second box
*/
inline Box::Box(const Box& b1, const Box& b2)
{
	a = Vector3::Min(b1.a, b2.a);
	b = Vector3::Max(b1.b, b2.b);
}

/*!
\brief Constructor from a point set.
\param pts set of point
*/
inline Box::Box(const std::vector<Vector3>& pts)
{
	for (int j = 0; j < 3; j++)
	{
		a[j] = pts.at(0)[j];
		b[j] = pts.at(0)[j];
		for (int i = 1; i < pts.size(); i++)
		{
			if (pts.at(i)[j] < a[j])
			{
				a[j] = pts.at(i)[j];
			}
			if (pts.at(i)[j] > b[j])
			{
				b[j] = pts.at(i)[j];
			}
		}
	}
}

/*
\brief Returns true if p is inside the box, false otherwise.
\param p world point
*/
inline bool Box::Contains(const Vector3& p) const
{
	return (p > a && p < b);
}

/*!
\brief Compute a random point inside a box. Note that this
is not a uniform sampling if the box is not a regular box (width = height = length).

In practice, a uniform sampling doesn't give different results and is way slower, so
we avoid this.

\return a random point inside the box.
*/
inline Vector3 Box::RandomInside() const
{
	Vector3 s = b - a;
	double randw = Random::Uniform(-1.0f * s[0] / 2.0f, s[0] / 2.0f);
	double randh = Random::Uniform(-1.0f * s[1] / 2.0f, s[1] / 2.0f);
	double randl = Random::Uniform(-1.0f * s[2] / 2.0f, s[2] / 2.0f);
	return (a + b) / 2.0f + Vector3(randw, randh, randl);
}

/*
\brief Get one of the vertex of the box.
*/
inline Vector3 Box::Vertex(int i) const
{
	if (i == 0)
		return a;
	return b;
}

/*
\brief Access box vertex by reference
*/
inline Vector3& Box::operator[](int i)
{
	if (i == 0)
		return a;
	return b;
}

/*
\brief Access box vertex by const value
*/
inline Vector3 Box::operator[](int i) const
{
	if (i == 0)
		return a;
	return b;
}

/*!
\brief Compute a Poisson sphere distribution inside a box.
This function uses a simple O(n<SUP>3</SUP>) dart throwing algorithm.
\param array of existing samples (possibly empty)
\param r Radius of the sphere.
\param n Number of candidate points.
*/
inline void Box::Poisson(std::vector<Vector3>& p, double r, int n) const
{
	double c = 4.0 * r * r;
	for (int i = 0; i < n; i++)
	{
		Vector3 t = RandomInside();
		bool hit = false;
		for (int j = 0; j < p.size(); j++)
		{
			if (SquaredMagnitude(t - p.at(j)) < c)
			{
				hit = true;
				break;
			}
		}
		if (hit == false)
			p.push_back(t);
	}
}


// AABB 2D
class Box2D
{
protected:
	Vector2 a;
	Vector2 b;

public:
	explicit Box2D();
	explicit Box2D(const Vector2& A, const Vector2& B);
	explicit Box2D(const Vector2& C, double R);
	explicit Box2D(const Box& b);

	bool Contains(const Vector2&) const;
	void Poisson(std::vector<Vector2>& p, double r, int n) const;
	Vector2 RandomInside() const;

	Vector2 Vertex(int i) const;
	Box ToBox(double zMin, double zMax) const;
	Vector2& operator[](int i);
	Vector2 operator[](int i) const;
};

/*
\brief Default Constructor
*/
inline Box2D::Box2D()
{
	a = Vector2(0);
	b = Vector2(0);
}

/*
\brief Constructor
\param A lower left vertex in world coordinates
\param B upper right vertex in world coordinates
*/
inline Box2D::Box2D(const Vector2& A, const Vector2& B) : a(A), b(B)
{
}

/*
\brief Constructor
\param C box center
\param R radius
*/
inline Box2D::Box2D(const Vector2& C, double R)
{
	Vector2 RR = Vector2(R);
	a = C - RR;
	b = C + RR;
}

/*!
\brief Constructor from a 3D box.
\param box the box
*/
inline Box2D::Box2D(const Box& box)
{
	a = Vector2(box.Vertex(0));
	b = Vector2(box.Vertex(1));
}

/*
\brief Returns true if p is inside the box, false otherwise.
\param p world point
*/
inline bool Box2D::Contains(const Vector2& p) const
{
	return (p > a && p < b);
}

/*
\brief Get one of the vertex of the box.
*/
inline Vector2 Box2D::Vertex(int i) const
{
	if (i == 0)
		return a;
	return b;
}

/*
\brief Transform a Box2 in a Box.
\param yMin altitude of the first vertex for the new Box
\param yMax altitude of the second vertex for the new Box
*/
inline Box Box2D::ToBox(double yMin, double yMax) const
{
	return Box(a.ToVector3(yMin), b.ToVector3(yMax));
}

/*
\brief Access box vertex by reference
*/
inline Vector2& Box2D::operator[](int i)
{
	if (i == 0)
		return a;
	return b;
}

/*
\brief Access box vertex by const value
*/
inline Vector2 Box2D::operator[](int i) const
{
	if (i == 0)
		return a;
	return b;
}

/*!
\brief Compute a random point inside a box. Note that this
is not a uniform sampling if the box is not a regular box (width = height = length).

In practice, a uniform sampling doesn't give different results and is way slower, so
we avoid this.

\return a random point inside the box.
*/
inline Vector2 Box2D::RandomInside() const
{
	Vector2 s = b - a;
	double randw = Random::Uniform(-1.0f * s[0] / 2.0f, s[0] / 2.0f);
	double randh = Random::Uniform(-1.0f * s[1] / 2.0f, s[1] / 2.0f);
	return (a + b) / 2.0f + Vector2(randw, randh);
}

/*!
\brief Compute a Poisson sphere distribution inside a box.
This function uses a simple O(n<SUP>3</SUP>) dart throwing algorithm.
\param array of existing samples (possibly empty)
\param r Radius of the sphere.
\param n Number of candidate points.
*/
inline void Box2D::Poisson(std::vector<Vector2>& p, double r, int n) const
{
	double c = 4.0 * r * r;
	for (int i = 0; i < n; i++)
	{
		Vector2 t = RandomInside();
		bool hit = false;
		for (int j = 0; j < p.size(); j++)
		{
			if (SquaredMagnitude(t - p.at(j)) < c)
			{
				hit = true;
				break;
			}
		}
		if (hit == false)
			p.push_back(t);
	}
}


// Sphere. Spherical geometric element.
class Sphere
{
protected:
	Vector3 center;
	double radius;

public:
	Sphere(const Vector3& c, double r);

	Vector3 RandomInside() const;
	void Poisson(std::vector<Vector3>& p, double r, int n) const;
};

/*!
\brief Constructor.
\param c center
\param r radius
*/
inline Sphere::Sphere(const Vector3& c, double r) : center(c), radius(r)
{

}

/*!
\brief Compute a random point inside a sphere, uniformly.
*/
inline Vector3 Sphere::RandomInside() const
{
	return center + Vector3(Random::Uniform(-radius, radius), Random::Uniform(-radius, radius), Random::Uniform(-radius, radius));
}

/*!
\brief Compute a Poisson sphere distribution inside a box.
This function uses a simple O(n<SUP>3</SUP>) dart throwing algorithm.
\param array of existing samples (possibly empty)
\param r Radius of the sphere.
\param n Number of candidate points.
*/
inline void Sphere::Poisson(std::vector<Vector3>& p, double r, int n) const
{
	double c = 4.0 * r * r;
	for (int i = 0; i < n; i++)
	{
		Vector3 t = RandomInside();
		bool hit = false;
		for (int j = 0; j < p.size(); j++)
		{
			if (SquaredMagnitude(t - p.at(j)) < c)
			{
				hit = true;
				break;
			}
		}
		if (hit == false)
			p.push_back(t);
	}
}


// ScalarField2D. A 2D field (nx * ny) of scalar values bounded in world space.
class ScalarField2D
{
protected:
	Box2D box;
	int nx, ny;
	std::vector<double> values;

public:
	/*
	\brief Default Constructor
	*/
	inline ScalarField2D() : nx(0), ny(0)
	{
		// Empty
	}

	/*
	\brief Constructor
	\param nx size in x axis
	\param ny size in z axis
	\param bbox bounding box of the domain in world coordinates
	*/
	inline ScalarField2D(int nx, int ny, const Box2D& bbox) : box(bbox), nx(nx), ny(ny)
	{
		values.resize(size_t(nx * ny));
	}

	/*
	\brief Constructor
	\param nx size in x axis
	\param ny size in y axis
	\param bbox bounding box of the domain
	\param value default value of the field
	*/
	inline ScalarField2D(int nx, int ny, const Box2D& bbox, double value) : box(bbox), nx(nx), ny(ny)
	{
		values.resize(nx * ny);
		Fill(value);
	}

	/*!
	\brief Constructor
	\param nx size in x axis
	\param ny size in y axis
	\param bbox bounding box of the domain
	\param value all values for the field
	*/
	inline ScalarField2D(int nx, int ny, const Box2D& bbox, const std::vector<double>& vals) : ScalarField2D(nx, ny, bbox)
	{
		for (int i = 0; i < vals.size(); i++)
			values[i] = vals[i];
	}

	/*
	\brief copy constructor
	\param field Scalarfield2D to copy
	*/
	inline ScalarField2D(const ScalarField2D& field) : ScalarField2D(field.nx, field.ny, field.box)
	{
		for (unsigned int i = 0; i < values.size(); i++)
			values[i] = field.values[i];
	}

	/*
	\brief Destructor
	*/
	inline ~ScalarField2D()
	{
	}

	/*
	\brief Compute the gradient for the vertex (i, j)
	*/
	inline Vector2 Gradient(int i, int j) const
	{
		Vector2 ret;
		double cellSizeX = (box.Vertex(1).x - box.Vertex(0).x) / (nx - 1);
		double cellSizeY = (box.Vertex(1).y - box.Vertex(0).y) / (ny - 1);

		// X Gradient
		if (i == 0)
			ret.x = (Get(i + 1, j) - Get(i, j)) / cellSizeX;
		else if (i == ny - 1)
			ret.x = (Get(i, j) - Get(i - 1, j)) / cellSizeX;
		else
			ret.x = (Get(i + 1, j) - Get(i - 1, j)) / (2.0f * cellSizeX);

		// Y Gradient
		if (j == 0)
			ret.y = (Get(i, j + 1) - Get(i, j)) / cellSizeY;
		else if (j == nx - 1)
			ret.y = (Get(i, j) - Get(i, j - 1)) / cellSizeY;
		else
			ret.y = (Get(i, j + 1) - Get(i, j - 1)) / (2.0f * cellSizeY);

		return ret;
	}

	/*
	\brief Compute a vertex world position including his height.
	*/
	inline Vector3 Vertex(int i, int j) const
	{
		double x = box.Vertex(0).x + i * (box.Vertex(1).x - box.Vertex(0).x) / (nx - 1);
		double y = Get(i, j);
		double z = box.Vertex(0).y + j * (box.Vertex(1).y - box.Vertex(0).y) / (ny - 1);
		return Vector3(z, y, x);
	}

	/*
	\brief Get Vertex world position by performing bilinear interpolation.
	\param v world position in 2D
	*/
	inline Vector3 Vertex(const Vector2& v) const
	{
		return Vector3(v.x, GetValueBilinear(v), v.y);
	}

	/*!
	\brief Check if a point lies inside the bounding box of the field.
	*/
	inline bool Inside(const Vector2& p) const
	{
		Vector2 q = p - box.Vertex(0);
		Vector2 d = box.Vertex(1) - box.Vertex(0);

		double u = q[0] / d[0];
		double v = q[1] / d[1];

		int j = int(u * (nx - 1));
		int i = int(v * (ny - 1));

		return Inside(i, j);
	}

	/*!
	\brief Check if a point lies inside the bounding box of the field.
	*/
	inline bool Inside(int i, int j) const
	{
		if (i < 0 || i >= nx || j < 0 || j >= ny)
			return false;
		return true;
	}

	/*!
	\brief Utility.
	*/
	inline void ToIndex2D(int index, int& i, int& j) const
	{
		i = index / nx;
		j = index % nx;
	}

	/*!
	\brief Utility.
	*/
	inline int ToIndex1D(int i, int j) const
	{
		return i * nx + j;
	}

	/*!
	\brief Returns the value of the field at a given coordinate.
	*/
	inline double Get(int row, int column) const
	{
		int index = ToIndex1D(row, column);
		return values[index];
	}

	/*!
	\brief Returns the value of the field at a given coordinate.
	*/
	inline double Get(int index) const
	{
		return values[index];
	}

	/*!
	\brief Compute the bilinear interpolation at a given world point.
	\param p world point.
	*/
	inline double GetValueBilinear(const Vector2& p) const
	{
		Vector2 q = p - box.Vertex(0);
		Vector2 d = box.Vertex(1) - box.Vertex(0);

		double texelX = 1.0f / double(nx - 1);
		double texelY = 1.0f / double(ny - 1);

		double u = q[0] / d[0];
		double v = q[1] / d[1];

		int i = int(v * (ny - 1));
		int j = int(u * (nx - 1));

		if (!Inside(i, j) || !Inside(i + 1, j + 1))
			return -1.0f;

		double anchorU = j * texelX;
		double anchorV = i * texelY;

		double localU = (u - anchorU) / texelX;
		double localV = (v - anchorV) / texelY;

		double v1 = Get(i, j);
		double v2 = Get(i + 1, j);
		double v3 = Get(i + 1, j + 1);
		double v4 = Get(i, j + 1);

		return (1 - localU) * (1 - localV) * v1
			+ (1 - localU) * localV * v2
			+ localU * (1 - localV) * v4
			+ localU * localV * v3;
	}

	/*!
	\brief Fill all the field with a given value.
	*/
	inline void Fill(double v)
	{
		std::fill(values.begin(), values.end(), v);
	}

	/*!
	\brief Set a given value at a given coordinate.
	*/
	inline void Set(int row, int column, double v)
	{
		values[ToIndex1D(row, column)] = v;
	}

	/*!
	\brief Set a given value at a given coordinate.
	*/
	inline void Set(int index, double v)
	{
		values[index] = v;
	}

	/*!
	\brief Compute the maximum of the field.
	*/
	inline double Max() const
	{
		if (values.size() == 0)
			return 0.0f;
		double max = values[0];
		for (int i = 1; i < values.size(); i++)
		{
			if (values[i] > max)
				max = values[i];
		}
		return max;
	}

	/*!
	\brief Compute the minimum of the field.
	*/
	inline double Min() const
	{
		if (values.size() == 0)
			return 0.0f;
		double min = values[0];
		for (int i = 1; i < values.size(); i++)
		{
			if (values[i] < min)
				min = values[i];
		}
		return min;
	}

	/*!
	\brief Returns the size of x-axis of the array.
	*/
	inline int SizeX() const
	{
		return nx;
	}

	/*!
	\brief Returns the size of y-axis of the array.
	*/
	inline int SizeY() const
	{
		return ny;
	}

	/*!
	\brief Returns the bounding box of the field.
	*/
	inline Box2D GetBox() const
	{
		return box;
	}

	/*!
	\brief Compute the memory used by the field.
	*/
	inline int Memory() const
	{
		return sizeof(ScalarField2D) + sizeof(double) * int(values.size());
	}
};
