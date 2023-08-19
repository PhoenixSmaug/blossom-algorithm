#include <iostream>
#include <set>
#include <algorithm>
#include <iterator>
#include "blossom.h"

int main() {
    Graph g(std::vector<std::pair<int, int>>({{1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 8}, {7, 9}, {3, 7}}));
    matching m(std::vector<edge>({edge(2, 3), edge(4, 5), edge(6, 7)}));

    std::ranges::copy(augmentingPath(g, m), std::ostream_iterator<int>(std::cout, " "));
    std::cout << std::endl;

    matching mg = maximumMatching(g);
    for (auto const& e : mg.edges) {
        std::cout << "(" << e.src << ", " << e.dst << ") ";
    }
    std::cout << std::endl;

    Graph h(std::vector<std::pair<int, int>>({{1, 2},{2, 3},{3, 4},{4, 5},{3, 5}, {5, 6},{6, 7}}));
    matching mh = maximumMatching(h);
    for (auto const& e : mh.edges) {
        std::cout << "(" << e.src << ", " << e.dst << ") ";
    }

    return 0;
}