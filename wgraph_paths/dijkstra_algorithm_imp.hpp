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

    //
    return path;
}

