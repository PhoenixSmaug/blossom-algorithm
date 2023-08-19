#ifndef BLOSSOM_BLOSSOM_H
#define BLOSSOM_BLOSSOM_H

#include <stdexcept>
#include <stack>
#include <ranges>
#include "Graph.h"

/**
 * Find all exposed vertices, meaning they aren't incident to any edge in the machting
 *
 * @param g graph
 * @param m matching
 * @return vector of exposed vertices
 */
std::vector<int> exposedVertices(Graph g, matching m) {
    std::vector<int> exposed;
    for (auto const& v : g.vertices) {
        if (!m.existsVertex(v)) {
            exposed.push_back(v);
        }
    }

    return exposed;
}

/**
 * Find index of tree in forest of which v is a part
 *
 * @param forest forest
 * @param v vertex
 * @return index
 *
 * @throws std::invalid_argument Thrown if forest does not contain v
 */
int findIdx(std::vector<Graph> forest, int v) {
    for (int i = 0; i < forest.size(); ++i) {
        for (auto const& w : forest[i].vertices) {
            if (w == v) {
                return i;
            }
        }
    }

    throw std::invalid_argument("Vertex not found in forest");
}

/**
 * Find a unmarked vertex with even distance to root
 *
 * If none exist return i as -1
 *
 * @param forest forest
 * @param fRoots roots of forest trees
 * @param vMark marked vertices
 * @param fNodes nodes in forest for order
 * @return v candidate vertex
 * @return i tree index of candidate vertex
 */
std::pair<int, int> candidateVertex(std::vector<Graph> forest, std::vector<int> fRoots, std::set<int> vMark, std::vector<int> fNodes) {
    for (auto const& v : fNodes) {
        if (!vMark.contains(v)) {
            return {v, findIdx(forest, v)};
        }
    }

    return {0, -1};
}

/**
 * Find an unmarked edge incident to v
 *
 * @param g graph
 * @param v vertex
 * @param eMark marked edges
 * @return w vertex such that (v, w) is a candidate edge
 * @return state if successful
 */
std::pair<int, bool> candidateEdge(Graph g, int v, matching eMark) {
    for (auto const &w : g.adjList[v]) {  // neighbours of v
        if (!eMark.edges.contains(edge(v, w))) {
            return {w, true};
        }
    }
    return {0, false};
}

/**
 * Checks if vertex v is part of tree
 *
 * @param forest forest
 * @param v vertex
 */
bool existInTree(std::vector<Graph> forest, int v) {
    for (int i = 0; i < forest.size(); ++i) {
        for (auto const& w : forest[i].vertices) {
            if (w == v) {
                return true;
            }
        }
    }
    return false;
}

/**
 * Convert path noted as sequence of vertices to a sequence of edges
 *
 * @param vertexPath vertex sequence
 * @return edge sequence
 */
std::vector<edge> pathConversion(std::vector<int> vertexPath) {
    std::vector<edge> edgePath;
    for (int i = 0; i < vertexPath.size() - 1; ++i) {
        edgePath.push_back(edge(vertexPath[i], vertexPath[i+1]));
    }
    return edgePath;
}

/**
 * Find base vertex of blossom
 *
 * @param m matching
 * @param blossom blossom
 * @return v vertex
 * @return i index of vertex
 *
 * @throws std::invalid_argument Thrown if blossom has no base
 */
std::pair<int, int> findBase(matching m, std::vector<int> blossom) {
    blossom.push_back(blossom[0]);  // for circular for loop
    blossom.push_back(blossom[1]);

    for (int i = 0; i < blossom.size() - 2; ++i) {
        bool a = !(m.edges.contains(edge(blossom[i], blossom[i+1])));
        bool b = !(m.edges.contains(edge(blossom[i+1], blossom[i+2])));
        if (!(m.edges.contains(edge(blossom[i], blossom[i+1]))) && !(m.edges.contains(edge(blossom[i+1], blossom[i+2])))) {  // two successive edges not in matching
            return {blossom[i+1], i+1};
        }
    }

    throw std::invalid_argument("Blossom has no base");
}

/**
 * Lift blossom with left stem
 *
 * @param g Graph
 * @param blossom blossom
 * @param lStem left stem
 * @return lifted blossom
 *
 * @throws std::invalid_argument Thrown if left stem is not connected
 */
std::vector<int> liftLeft(Graph g, std::vector<int> blossom, std::vector<int> lStem) {
    for (int i = 0; i < blossom.size(); ++i) {
        if (g.existEdge(blossom[i], lStem.back())) {
            if (i % 2 == 0) {
                std::vector<int> result(blossom.begin(), blossom.begin() + i + 1);
                std::reverse(result.begin(), result.end());
                return result;
            } else {
                return std::vector<int>(blossom.begin() + i, blossom.end());
            }
        }
    }

    throw std::invalid_argument("Left stem is not connected");
}

/**
 * Lift blossom with right stem
 *
 * @param g Graph
 * @param blossom blossom
 * @param rStem right stem
 * @return lifted blossom
 *
 * @throws std::invalid_argument Thrown if right stem is not connected
 */
std::vector<int> liftRight(Graph g, std::vector<int> blossom, std::vector<int> rStem) {
    for (int i = 0; i < blossom.size(); ++i) {
        if (g.existEdge(blossom[i], rStem[0])) {
            if (i % 2 == 0) {
                return std::vector<int>(blossom.begin(), blossom.begin() + i + 1);
            } else {
                std::vector<int> result(blossom.begin() + i, blossom.end());
                std::reverse(result.begin(), result.end());
                return result;
            }
        }
    }

    throw std::invalid_argument("Right stem is not connected");
}

/**
 * Find an augmenting path for m in g if one exists
 *
 * Pseudo code for the augmenting path can be found at https://en.m.wikipedia.org/wiki/Blossom_algorithm, with a
 * detailed description of the lifting in https://github.com/amyshoe/CME323-Project/blob/master/seq_blossom.py
 *
 * @param g undirected graph
 * @param m matching
 * @return vector of augmenting path edges
 *
 * @throws std::out_of_range Thrown if blossom stack is empty
 */
std::vector<int> augmentingPath(Graph g, matching m, std::stack<int> stack = std::stack<int>()) {
    std::vector<Graph> forest;
    std::vector<int> fRoots;  // roots of forest trees
    std::vector<int> fNodes;  // all vertices in the forest

    std::set<int> vMark;  // marked vertices
    matching eMark = m;  // marked edges, not necessarily a valid matching

    for (auto const& v : exposedVertices(g, m)) {
        Graph tempGraph;  // add singleton {v} to forest
        tempGraph.addVertex(v);
        forest.push_back(tempGraph);

        fRoots.push_back(v);
        fNodes.push_back(v);
    }

    while (true) {
        auto [v, vIdx] = candidateVertex(forest, fRoots, vMark, fNodes);
        if (vIdx == -1) {  // no candidate vertex exists
            break;
        }

        while (true) {
            auto [w, wState] = candidateEdge(g, v, eMark);
            if (!wState) {
                break;
            }

            if (!existInTree(forest, w)) {  // w is matched, add edges to f
                int u = m.matchedVertex(w);

                forest[vIdx].addVertex(w);
                forest[vIdx].addVertex(u);
                fNodes.push_back(u);

                forest[vIdx].addEdge(v, w);
                forest[vIdx].addEdge(w, u);
            } else {
                int wIdx = findIdx(forest, w);
                if (forest[wIdx].dist(w, fRoots[wIdx]) % 2 == 0) {
                    if (vIdx != wIdx) {  // no blossom found
                        std::vector<int> pathV = forest[vIdx].path(fRoots[vIdx], v);
                        std::vector<int> pathW = forest[wIdx].path(w, fRoots[wIdx]);

                        pathV.insert(pathV.end(), pathW.begin(), pathW.end());

                        return pathV;
                    } else {
                        std::vector<int> blossom = forest[vIdx].path(v, w);
                        Graph gCont = g;
                        matching mCont = m;

                        for (auto const& u : blossom) {
                            if (u != w) {
                                gCont.contract(w, u);
                                mCont.removeIncidentEdge(u);
                            }
                        }

                        blossom.push_back(v);  // for lifting afterwards append v also at the end
                        stack.push(w);  // remember blossom

                        std::vector<int> augPath = augmentingPath(gCont, mCont, stack);  // recursive call on contracted graph

                        if (stack.empty()) {
                            throw std::out_of_range("Blossom stack is empty");
                        }

                        int b = stack.top();  // blossom vertex

                        if (std::ranges::find(augPath, b) != augPath.end()) {  // blossom part of augmenting path
                            auto it = std::find(augPath.begin(), augPath.end(), b);  // index of b
                            int index = std::distance(augPath.begin(), it);

                            std::vector<int> lStem(augPath.begin(), augPath.begin() + index);  // split into the stems
                            std::vector<int> rStem(augPath.begin() + index, augPath.end());

                            auto [base, baseIdx] = findBase(m, blossom);
                            std::rotate(blossom.begin(), blossom.begin() + baseIdx - 1, blossom.end());

                            if (lStem.empty() || rStem.empty()) {  // blossom contains start or end point
                                if (!lStem.empty()) {
                                    if (g.existEdge(base, lStem.back())) {  // lStem is directly connected to base
                                        lStem.push_back(base);
                                        return lStem;
                                    } else {
                                        std::vector<int> liftedBlossom = liftLeft(g, blossom, lStem);
                                        lStem.insert(lStem.end(), liftedBlossom.begin(), liftedBlossom.end());
                                        return lStem;
                                    }
                                } else {
                                    if (g.existEdge(base, rStem[0])) {  // rStem is directly connected to base
                                        std::vector<int> path;
                                        path.push_back(base);
                                        path.insert(path.end(), rStem.begin(), rStem.end());
                                    } else {
                                        std::vector<int> liftedBlossom = liftRight(g, blossom, rStem);
                                        liftedBlossom.insert(liftedBlossom.end(), rStem.begin(), rStem.end());
                                        return liftedBlossom;
                                    }
                                }
                            } else {  // blossom is in the middle of the path
                                if (m.edges.contains(edge(base, lStem.back()))) {  // if lStem attaches to base
                                    if (g.existEdge(base, rStem[0])) {
                                        lStem.push_back(base);
                                        lStem.insert(lStem.end(), rStem.begin(), rStem.end());
                                        return lStem;
                                    } else {
                                        std::vector<int> liftedBlossom = liftRight(g, blossom, rStem);
                                        lStem.insert(lStem.end(), liftedBlossom.begin(), liftedBlossom.end());
                                        lStem.insert(lStem.end(), rStem.begin(), rStem.end());
                                        return lStem;
                                    }
                                } else {
                                    if (g.existEdge(base, lStem.back())) {
                                        lStem.push_back(base);
                                        lStem.insert(lStem.end(), rStem.begin(), rStem.end());
                                        return lStem;
                                    } else {
                                        std::vector<int> liftedBlossom = liftLeft(g, blossom, lStem);
                                        lStem.insert(lStem.end(), liftedBlossom.begin(), liftedBlossom.end());
                                        lStem.insert(lStem.end(), rStem.begin(), rStem.end());
                                        return lStem;
                                    }
                                }
                            }
                        } else {
                            return augPath;
                        }
                    }
                }
            }
            eMark.edges.insert(edge(v, w));
        }
        vMark.insert(v);
    }

    return std::vector<int>();
}

/**
 * Find a maximum matching of the graph g
 *
 * @param g undirected graph
 * @param m starting matching
 * @return vector of matching edges
 */
matching maximumMatching(Graph g, matching m) {
    std::vector<int> vertexPath = augmentingPath(g, m);
    if (!vertexPath.empty()) {
        std::vector<edge> path = pathConversion(vertexPath);
        m.augment(path);
        return maximumMatching(g, m);
    } else {
        return m;
    }
}

/**
 * Find a maximum matching of the graph g
 *
 * Select first edge of grpah as starting guess for the matching
 *
 * @param g undirected graph
 * @return vector of matching edges
 */
matching maximumMatching(Graph g) {
    matching m(std::vector<edge>({*g.edges.begin()}));
    return maximumMatching(g, m);
}

#endif //BLOSSOM_BLOSSOM_H
