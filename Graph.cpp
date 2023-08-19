#include <algorithm>
#include <iterator>
#include <numeric>
#include <queue>
#include "Graph.h"

/**
 * Print vertices and list of edges
 *
 * @param os output stream
 * @param graph graph
 * @return output stream
 */
std::ostream &operator<<(std::ostream& os, const Graph& graph) {
    os << "Vertices: ";
    std::ranges::copy(graph.vertices, std::ostream_iterator<int>(os, " "));
    os << std::endl;

    os << "Edges: " << std::endl;
    for (auto const& i : graph.vertices) {
        auto it = graph.adjList.find(i);
        if (it != graph.adjList.end()) {
            for (auto const& j : it->second) {
                if (i < j) {
                    os << i << " -> " << j << std::endl;
                }
            }
        }
    }

    return os;
}

/**
 * Construct empty graph with n vertices
 *
 * @param n number of vertices
 */
Graph::Graph(int n) {
    std::vector<int> vertices(n);
    std::iota(vertices.begin(), vertices.end(), 1);

    this->vertices = std::set<int>(vertices.begin(), vertices.end());
}

/**
 * Construct empty graph with vertex vector
 *
 * @param vertices vector of vertex ids
 */
Graph::Graph(std::vector<int> vertices) {
    this->vertices = std::set<int>(vertices.begin(), vertices.end());
}

/**
 * Construct graph by edge vector
 *
 * @param vertices vector of vertex ids
 */
Graph::Graph(const std::vector<std::pair<int, int>>& edges) {
    for (auto const &e : edges) {  // add all edges such that e.dst < e.src
        this->edges.insert(edge(e.first, e.second));
    }

    for (auto const &e : this->edges) {
        this->vertices.insert(e.src);
        this->vertices.insert(e.dst);

        this->adjList[e.src].insert(e.dst);
        this->adjList[e.dst].insert(e.src);
    }
}

/**
 * Verifies that vertex exists
 *
 * @param v vertex id
 */
bool Graph::existVertex(const int v) {
    return this->vertices.contains(v);
}

/**
 * Adds a new vertex
 *
 * @param v vertex id
 * @return if successful
 */
bool Graph::addVertex(const int v) {
    if (!existVertex(v)) {
        this->vertices.insert(v);
        return true;
    }
    return false;
}

/**
 * Removes a vertex and all incident edges
 *
 * @param v vertex id
 * @return if successful
 */
bool Graph::removeVertex(const int v) {
    if (existVertex(v)) {
        this->vertices.erase(v);  // remove vertex

        for (auto const& w : this->adjList.at(v)) {  // remove all incident edges
            this->edges.erase(edge(v, w));
            this->adjList[w].erase(v);
        }
        this->adjList.erase(v);
        return true;
    }
    return false;
}

/**
 * Verify that edge (v, w) exists
 *
 * @param v first vertex
 * @param w second vertex
 */
bool Graph::existEdge(const int v, const int w) {
    return existVertex(v) && existVertex(w) && this->adjList[v].contains(w);
}

/**
 * Verify that edge e exists
 *
 * @param e edge
 */
bool Graph::existEdge(const edge e) {
    return this->edges.contains(e);
}

/**
 * Add edge (v, w)
 *
 * @param v first vertex
 * @param w second vertex
 * @return if successful
 */
bool Graph::addEdge(const int v, const int w) {
    if (!existEdge(v, w)) {
        this->edges.insert(edge(v, w));
        this->adjList[v].insert(w);
        this->adjList[w].insert(v);
        return true;
    }
    return false;
}

/**
 * Add edge e
 *
 * @param e edge
 * @return if successful
 */
bool Graph::addEdge(const edge e) {
    if (!existEdge(e)) {
        this->edges.insert(e);
        this->adjList[e.src].insert(e.dst);
        this->adjList[e.dst].insert(e.src);
        return true;
    }
    return false;
}

/**
 * Remove edge (v, w)
 *
 * @param v first vertex
 * @param w second vertex
 * @return if successful
 */
bool Graph::removeEdge(const int v, const int w) {
    if (existEdge(v, w)) {
        this->edges.erase(edge(v, w));
        this->adjList[v].erase(w);
        this->adjList[w].erase(v);
        return true;
    }
    return false;
}

/**
 * Remove edge e
 *
 * @param e edge
 * @return if successful
 */
bool Graph::removeEdge(const edge e) {
    if (existEdge(e)) {
        this->edges.erase(e);
        this->adjList[e.src].erase(e.dst);
        this->adjList[e.dst].erase(e.src);
        return true;
    }
    return false;
}

/**
 * Find shortest path between source and destination using the Dijkstra Algorithm
 *
 * @param src source vertex
 * @param dst destination vertex
 * @return vector of path vertices, including src and dst
 */
std::vector<int> Graph::path(const int src, const int dst) {
    std::queue<int> queue;
    std::map<int, int> distances;
    for (auto const& v : this->vertices) {
        distances[v] =  std::numeric_limits<int>::max();
    }
    std::map<int, int> predecessor;
    for (auto const &v : this->vertices) {
        predecessor[v] = -1;
    }
    queue.push(src);
    distances[src] = 0;

    while (!queue.empty()) {
        int current = queue.front();
        queue.pop();

        if (current == dst) {
            std::vector<int> path;
            if (predecessor[current] == -1 && current != src) { return std::vector<int>(); }

            while (current != -1) {
                path.push_back(current);
                current = predecessor[current];
            }

            std::reverse(path.begin(), path.end());
            return path;
        }

        for (auto const &i: this->adjList[current]) {
            if (distances[i] > distances[current] + 1) {
                distances[i] = distances[current] + 1;
                queue.push(i);
                predecessor[i] = current;
            }
        }
    }

    return std::vector<int>();
}

/**
 * Find length of shortest path using Graph::path()
 *
 * @param src source vertex
 * @param dst destination vertex
 * @return length of path, including src and dst
 */
int Graph::dist(const int src, const int dst) {
    return path(src, dst).size() - 1;
}

/**
 * Contract vertex w into vertex w
 *
 * @param v remaining vertex
 * @param w absorbed vertex
 * @return if successful
 */
bool Graph::contract(const int v, const int w) {
    if (existEdge(v, w)) {
        this->removeEdge(v, w);

        for (auto const& u : this->adjList[w]) {  // neighbours of w
            this->addEdge(v, u);
        }

        this->removeVertex(w);
        return true;
    }

    return false;
}

/**
 * Print edges of matching
 *
 * @param os output stream
 * @param matching machting
 * @return output stream
 */
std::ostream &operator<<(std::ostream& os, const matching& matching) {
    os << "Edges: " << std::endl;
    for (auto const& e : matching.edges) {
        os << e.src << " -> " << e.dst << std::endl;
    }

    return os;
}

/**
 * Determines cardinality of matching
 *
 * @return size
 */
int matching::size() {
    return this->edges.size();
}

/**
 * Find all vertices incident to edges in the matching
 *
 * @return vector of vertex ids
 */
std::set<int> matching::vertices() {
    std::set<int> vertices;
    for (auto const& e : this->edges) {
        vertices.insert(e.dst);
        vertices.insert(e.src);
    }

    return vertices;
}

/**
 * Verifies that vertex exists
 *
 * @param v vertex id
 */
bool matching::existsVertex(const int v) {
    return this->vertices().contains(v);
}

/**
 * Remove edge incident to vertex v
 *
 * @param v vertex id
 * @return if successful
 */
bool matching::removeIncidentEdge(const int v) {
    for (auto const& e : this->edges) {
        if (e.src == v || e.dst == v) {
            this->edges.erase(e);
            return false;
        }
    }

    return false;
}

/**
 * Find vertex matched to vertex v in the matching
 *
 * @param v vertex id
 * @return if successful
 *
 * @throws std::invalid_argument Thrown if matching does not contain v
 */
 int matching::matchedVertex(int v) {
     if (this->existsVertex(v)) {
         for (auto const& e : this->edges) {
             if (e.src == v) {
                 return e.dst;
             } else if (e.dst == v) {
                 return e.src;
             }
         }
     }

     throw std::invalid_argument("Vertex not found in matching");
 }

/**
 * Check if matching has no incident edges
 */
bool matching::isValid() {
    return 2 * this->edges.size() == this->vertices().size();
}

/**
 * Augmenting matching via augmenting path
 * @param path augmenting path
 * @return if path is actually augmenting
 */
bool matching::augment(std::vector<edge> path) {
    for (int i = 0; i < path.size(); ++i) {
        if (i % 2 == 0 && !this->edges.contains(path[i])) {  // even edge not in matching
            this->edges.insert(path[i]);
            continue;
        }
        if (i % 2 == 1 && this->edges.contains(path[i])) {  // odd edge in matching
            this->edges.erase(path[i]);
            continue;
        }

        return false;
    }

    return true;
}