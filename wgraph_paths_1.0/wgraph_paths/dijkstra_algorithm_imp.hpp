/**
 * @file dijkstra_algorithm_imp.hpp
 *
 * CopyRight F. J. Madrid-Cuevas <fjmadrid@uco.es>
 *
 * Sólo se permite el uso de este código en la docencia de las asignaturas sobre
 * Estructuras de Datos de la Universidad de Córdoba.
 *
 * Está prohibido su uso para cualquier otro objetivo.
 */
#pragma once
#include <cassert>
#include <tuple>
#include <queue>
#include <limits>
#include <functional>

#include "dijkstra_algorithm.hpp"


template<class T>
void dijkstra_algorithm(typename WGraph<T>::Ref g,
                        std::vector<size_t>& predecessors,
                        std::vector<float>& distances)
{
    assert(g->has_current_node());

    //TODO
    //Hint: initialize distances with inf.
    //Hint: Initialize the vector of predecessors so that each vertex is a
    //      predecessor of itself.
    //Hint: use the the template tuple para represent edges. Set the fields so
    //      the tuples will be sorted properly.
    //      @see: https://en.cppreference.com/w/cpp/utility/tuple
    // Hint: you can use the keyword "using" to create an alias "Tuple"
    // Initialize distances with infinity
        const float inf = std::numeric_limits<float>::infinity();
        distances.assign(g->size(), inf);

        // Initialize the vector of predecessors so that each vertex is a predecessor of itself
        predecessors.assign(g->size(), g->current_node()->label());

        // Create an alias for the edge tuple using the template parameter T
        using Tuple = std::tuple<float, size_t>;

        // Create a priority queue to store the vertices with their distances
        std::priority_queue<Tuple, std::vector<Tuple>, std::greater<Tuple>> pq;

        // Set the distance of the current vertex to 0 and add it to the priority queue
        size_t currentVertex = g->current_node()->label();
        distances[currentVertex] = 0.0;
        pq.emplace(0.0, currentVertex);

        while (!pq.empty())
        {
            // Get the vertex with the smallest distance from the priority queue
            auto top = pq.top();
            float dist = std::get<0>(top);
            size_t u = std::get<1>(top);
            pq.pop();

            // Skip if the vertex has already been visited
            if (dist > distances[u])
            {
                continue;
            }

            // Explore all adjacent vertices of u
            g->goto_node(g->node(u));
            g->goto_first_edge();
            while (g->has_current_edge())
            {
                size_t v = g->current_edge()->second()->label();
                float weight = g->current_weight();

                // Calculate the new distance from the start vertex to v
                float newDist = distances[u] + weight;

                // If the new distance is smaller than the current distance, update it
                if (newDist < distances[v])
                {
                    distances[v] = newDist;
                    predecessors[v] = u; // Update the predecessor of vertex v
                    pq.emplace(newDist, v);
                }

                g->goto_next_edge();
            }
        }
    //
}


inline std::list<size_t>
dijkstra_path(size_t src, size_t dst,
              std::vector<size_t>const& predecessors)
{
    assert(src<predecessors.size());
    assert(dst<predecessors.size());
    assert(predecessors[src]==src);
    std::list<size_t> path;
    //TODO
    size_t current = dst;

        while (current != src) {
            path.push_front(current);
            current = predecessors[current];
            assert(current < predecessors.size());  // Ensure valid predecessor index
        }

        path.push_front(src);
    //
    return path;
}

