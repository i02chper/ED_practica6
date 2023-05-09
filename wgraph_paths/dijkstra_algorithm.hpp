/**
 * @file dijkstra_algorithm.hpp
 *
 * CopyRight F. J. Madrid-Cuevas <fjmadrid@uco.es>
 *
 * Sólo se permite el uso de este código en la docencia de las asignaturas sobre
 * Estructuras de Datos de la Universidad de Córdoba.
 *
 * Está prohibido su uso para cualquier otro objetivo.
 */
#pragma once

#include <vector>
#include <list>

#include "wgraph.hpp"

/** @brief The Dijkstra's algorithm.
 * Using current vertex as start vertex, compute all the minimum paths to the
 * rest of vertices.
 * @arg[in] g is the graph.
 * @arg[out] predecessors saved which is the predecessor vertex to archive a
 *           vertex n.
 * @arg[out] distances saved the distances of each vertex to the source vertex.
 * @pre g->has(source)
 * @warning if predecessor[n]=n means that vertex or is the start vertex,
 *          (distance[n]==0), or the vertex is not connected to the start
 *          vertex (distance[n]=inf).
 */
template<class T>
void dijkstra_algorithm(typename WGraph<T>::Ref g,
                        std::vector<size_t>& predecessors,
                        std::vector<float>& distances);

/**
 * @brief Retrieve a path from Dijkstra's predecessor vector.
 * @param src is the start vertex's label.
 * @param dst is the destination vertex's label.
 * @param predecessors vector returned by the Dijkstra algorithm.
 * @return a list with the path of vertices from src to dst.
 * @pre src<predecessors.size()
 * @pre dst<predecessors.size()
 * @pre predecessor[src]==src
 */
inline std::list<size_t>
dijkstra_path(size_t src, size_t dst,
              std::vector<size_t>const& predecessors);

#include "dijkstra_algorithm_imp.hpp"