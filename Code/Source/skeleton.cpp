#include "graph.h"
#include <fstream>		// for saving the skeleton into a text file
#include <map>			// std::map
#include <algorithm>	// std::find
#include <string>		// MSVC2017 requires this

/*!
\brief Constructor from the volumetric nearest neighbour graph and a set of path.
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
void KarsticSkeleton::Amplify(VolumetricGraph* graph, std::vector<KeyPoint>& newKeyPts)
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
\brief Computes the bounding box of the skeleton.
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
\brief Appends a set of paths to the existing karstic skeleton.
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
	for (int i = 0; i < nodes.size(); i++)	// Edges
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
			auto newSection = KarsticSection({ edges[j] });
			if (std::find(nodes[i].connections.begin(), nodes[i].connections.end(), edges[j]) == nodes[i].connections.end())
				nodes[i].connections.push_back(newSection);
		}
	}
}

/*!
\brief Export the karstic skeleton graph as a pair of ASCII file (one for nodes, one for links)
\param file filename
*/
void KarsticSkeleton::SaveDAT(const std::string& file) const
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

/*!
\brief Export the karstic skeleton as an .obj file, using sphere mesh for key points, and cylinder meshes for edges.
\param file filename
*/
void KarsticSkeleton::SaveObj(const std::string& file) const
{
	// Build geometry
	const int n = 8;
	const float r = 1.5f;
	struct Mesh {
		public:
			std::vector<Vector3> vertices, normals;
			std::vector<int> indices;

		public:
			void MergeMesh(const Mesh& mesh) {
				// Set size
				vertices.resize(vertices.size() + mesh.vertices.size());
				normals.resize(normals.size() + mesh.normals.size());
				indices.resize(indices.size() + mesh.indices.size());

				// Merge vertices, normals
				for (int i = 0; i < mesh.vertices.size(); i++)
					vertices[vertices.size() - mesh.vertices.size() + i] = mesh.vertices[i];
				for (int i = 0; i < mesh.normals.size(); i++)
					normals[normals.size() - mesh.normals.size() + i] = mesh.normals[i];

				// Indexes should be shifted
				for (int i = 0; i < mesh.indices.size(); i++)
					indices[indices.size() - mesh.indices.size() + i] = mesh.indices[i] + int(vertices.size() - mesh.vertices.size());
			}
			static Mesh CreateSphere(const Vector3& c, float r) {
				Mesh ret;

				int p = 2 * n;
				int s = (2 * n) * (n - 1) + 2;

				ret.vertices.resize(s);
				ret.normals.resize(s);

				// Create set of vertices
				float dt = Math::Pi / n;
				float df = Math::Pi / n;

				int k = 0;
				float f = -Math::HalfPi;
				for (int j = 1; j < n; j++)
				{
					f += df;
					float t = 0.0;
					for (int i = 0; i < 2 * n; i++)
					{
						Vector3 u = Vector3(cos(t) * cos(f), sin(t) * cos(f), sin(f));
						ret.normals[k] = u;
						ret.vertices[k] = c + r * u;
						k++;
						t += dt;
					}
				}
				// North pole
				ret.normals[s - 2] = Vector3(0.0, 0.0, 1.0);
				ret.vertices[s - 2] = c + Vector3(0.0, 0.0, r);

				// South
				ret.normals[s - 1] = Vector3(0.0, 0.0, -1.0);
				ret.vertices[s - 1] = c - Vector3(0.0, 0.0, r);

				// Reserve space for the smooth triangle array
				ret.indices.reserve(4 * n * (n - 1) * 3);

				// South cap
				for (int i = 0; i < 2 * n; i++)
					ret.AddSmoothTriangle(s - 1, (i + 1) % p, i);

				// North cap
				for (int i = 0; i < 2 * n; i++)
					ret.AddSmoothTriangle(s - 2, 2 * n * (n - 2) + i, 2 * n * (n - 2) + (i + 1) % p);

				// Sphere
				for (int j = 1; j < n - 1; j++)
				{
					for (int i = 0; i < 2 * n; i++)
						ret.AddSmoothQuadrangle((j - 1) * p + i, (j - 1) * p + (i + 1) % p, j * p + (i + 1) % p, j * p + i);
				}

				return ret;
			}
			static Mesh CreateCylinder(const Vector3& a, const Vector3& b, float r) {
				Mesh ret;

				Vector3 z = Normalize(b - a);
				Vector3 x, y;
				z.Orthonormal(x, y);

				// Set dimension of array of vertices and normals
				ret.vertices.resize(2 * n + 2);
				ret.normals.resize(n + 2);

				// Vertices and normals
				for (int i = 0; i < n; i++)
				{
					float t = Math::Angle(i, n);
					Vector3 u = x * cos(t) + y * sin(t);

					ret.vertices[i] = a + r * u;
					ret.vertices[n + i] = b + r * u;
					ret.normals[i] = u;
				}

				// Indexes for apex vertices and normals
				int iav = 2 * n;
				int ibv = 2 * n;
				int ian = n;
				int ibn = n;

				ret.vertices[iav] = a;
				ret.normals[ian] = -z;

				ibv = iav + 1;
				ibn = ian + 1;
				ret.vertices[ibv] = b;
				ret.normals[ibn] = z;

				// Reserve space for the triangle and smooth triangle arrays
				ret.indices.reserve(4 * n * 3);

				// Caps
				for (int i = 0; i < n; i++)
					ret.AddTriangle(iav, (i + 1) % n, i);
				for (int i = 0; i < n; i++)
					ret.AddTriangle(ibv, n + i, n + (i + 1) % n);

				// Side
				for (int i = 0; i < n; i++)
				{
					ret.AddSmoothTriangle(i, (i + 1) % n, n + i);
					ret.AddSmoothTriangle((i + 1) % n, n + (i + 1) % n, n + i);
				}

				return ret;
			}

		private:
			void AddSmoothTriangle(int a, int b, int c) {
				indices.push_back(a);
				indices.push_back(b);
				indices.push_back(c);
			}
			void AddSmoothQuadrangle(int a, int b, int c, int d) {
				AddSmoothTriangle(a, b, c);
				AddSmoothTriangle(a, c, d);
			}
			void AddTriangle(int a, int b, int c) {
				indices.push_back(a);
				indices.push_back(b);
				indices.push_back(c);
			}
	};

	Mesh mesh;
	for (int i = 0; i < nodes.size(); i++)
	{
		mesh.MergeMesh(Mesh::CreateSphere(nodes[i].p, r * 2.0f));
		for (int j = 0; j < nodes[i].connections.size(); j++)
			mesh.MergeMesh(Mesh::CreateCylinder(nodes[i].p, nodes[nodes[i].connections[j].destIndex].p, r));
	}

	// Write to file
	std::ofstream out;
	out.open(file + ".obj");
	if (out.is_open() == false)
		return;
	out << "g " << "Obj" << std::endl;
	for (int i = 0; i < mesh.vertices.size(); i++)
		out << "v " << mesh.vertices.at(i).x << " " << mesh.vertices.at(i).y << " " << mesh.vertices.at(i).z << '\n';
	for (int i = 0; i < mesh.normals.size(); i++)
		out << "vn " << mesh.normals.at(i).x << " " << mesh.normals.at(i).y << " " << mesh.normals.at(i).z << '\n';
	for (int i = 0; i < mesh.indices.size(); i += 3)
	{
		out << "f " << mesh.indices.at(i) + 1 << "//" << mesh.indices.at(i) + 1
			<< " " << mesh.indices.at(i + 1) + 1 << "//" << mesh.indices.at(i + 1) + 1
			<< " " << mesh.indices.at(i + 2) + 1 << "//" << mesh.indices.at(i + 2) + 1
			<< '\n';
	}
	out.close();
}
