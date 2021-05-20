#include "graph.h"

#include <limits>       // for numeric_limits
#include <set>          // for set
#include <utility>      // for pair

/*
\brief
*/
CostGraph::CostGraph(int n)
{
	adj.resize(n);
}

/*
\brief
*/
void CostGraph::SetEdge(int s, int t, double w)
{
	adj[s].push_back(GraphEdge(t, w));
}

/*
\brief
*/
void CostGraph::DijkstraComputePaths(int source, std::vector<double>& distance, std::vector<int>& previous) const
{
    constexpr double max_weight = std::numeric_limits<double>::infinity();
    size_t n = adj.size();
    distance.clear();
    distance.resize(n, max_weight);
    distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
    std::set<std::pair<double, int>> vertex_queue;
    vertex_queue.insert(std::make_pair(distance[source], source));
    while (!vertex_queue.empty())
    {
        int u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());

        // Visit each edge exiting u
        const std::vector<GraphEdge>& neighbors = adj[u];
        for (std::vector<GraphEdge>::const_iterator neighbor_iter = neighbors.begin(); neighbor_iter != neighbors.end(); neighbor_iter++)
        {
            int v = neighbor_iter->target;
            double weight = neighbor_iter->weight;
            double distance_through_u = /*dist +*/ weight;
            if (distance_through_u < distance[v])
            {
                vertex_queue.erase(std::make_pair(distance[v], v));

                distance[v] = distance_through_u;
                previous[v] = u;
                vertex_queue.insert(std::make_pair(distance[v], v));
            }
        }
    }
}

/*
\brief
*/
std::vector<int> CostGraph::DijkstraGetShortestPathTo(int target, const std::vector<int>& previous) const
{
    std::vector<int> path;
    int n = target;
    for (; n != -1; n = previous[n])
        path.insert(path.begin(), n);
    return path;
}
