#ifndef BLOSSOM_GRAPH_H
#define BLOSSOM_GRAPH_H

#include <vector>
#include <set>
#include <list>
#include <ostream>
#include <map>

/**
 * Edge struct
 *
 * - Custom operator for ordered sets and maps
 */
struct edge {
    edge();
    edge(int src, int dst) : src(std::min(src, dst)), dst(std::max(src, dst)) {};  // for uniqueness always e.src < e.dst

    bool operator<(const edge& rhs) const {
        if (src == rhs.src) {
            return dst < rhs.dst;
        }
        return src < rhs.src;
    }

    int src;
    int dst;
};

/**
 * Matching struct
 *
 * - Gives access to matched vertices and validation
 */
struct matching {
    friend std::ostream &operator<<(std::ostream& os, const matching& matching);
public:
    matching() = default;
    explicit matching(std::set<edge> edges) : edges(edges) {};
    explicit matching(std::vector<edge> edges) : edges(std::set<edge>(edges.begin(), edges.end())) {};

    int size();
    std::set<int> vertices();
    bool existsVertex(int v);
    bool removeIncidentEdge(int v);
    int matchedVertex(int v);
    bool isValid();
    bool augment(std::vector<edge> path);

    std::set<edge> edges;
};

/**
 * Graph class for the blossom algorithm
 *
 * - Vertices have arbitrary numbering instead of being labeled from 0 to n-1
 * - Graph is assumed to be undirected and having no loops or double edges
 */
class Graph {
    friend std::ostream &operator<<(std::ostream& os, const Graph& graph);
public:
    explicit Graph(int n);
    explicit Graph(std::vector<int> vertices);
    explicit Graph(const std::vector<std::pair<int, int>>& edges);
    Graph() = default;

    bool existVertex(int v);  // no input verification for performance reasons
    bool addVertex(int v);
    bool removeVertex(int v);
    bool existEdge(int v,  int w);
    bool existEdge(edge e);
    bool addEdge(int v, int w);
    bool addEdge(edge e);
    bool removeEdge(int v, int w);
    bool removeEdge(edge e);

    std::vector<int> path(int src, int dst);
    int dist(int src, int dst);
    bool contract(int v, int w);

    std::map<int, std::set<int>> adjList;
    std::set<int> vertices;
    std::set<edge> edges;
};
std::ostream &operator<<(std::ostream& os, const Graph& graph);

#endif //BLOSSOM_GRAPH_H
