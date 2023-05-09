/**
 * @file floyd_algorithm.hpp
 *
 * CopyRight F. J. Madrid-Cuevas <fjmadrid@uco.es>
 *
 * Sólo se permite el uso de este código en la docencia de las asignaturas sobre
 * Estructuras de Datos de la Universidad de Córdoba.
 *
 * Está prohibido su uso para cualquier otro objetivo.
 */
#pragma once

#include <list>
#include "wgraph.hpp"



/**
 * @brief Search for the minium path between vertices using the Floyd's algorithm.
 * @arg[in] g is a weighted graph.
 * @arg[out] D is the Distances matrix.
 * @arg[out] I is the Intermediate matrix.
 */
template<class T>
void floyd_algorithm(const typename WGraph<T>::Ref g, FMatrix::Ref& D,
                     IMatrix::Ref& I);


/**
 * @brief Given the Floyd's Intermidiate matrix, compute the path.
 * @arg[in] src is the label of the start vertex.
 * @arg[in] dst is the label of the end vertex.
 * @arg[in] I is the Floyd's Intermidiate matrix.
 * @return the path as list with the sequence of vertex's labels for
 *           the minimum path linking u with v.
 * @pre u and v are connected (D[src,dst]<inf).
 */
std::list<size_t> floyd_path(size_t src, size_t dst, IMatrix::Ref I);

#include "floyd_algorithm_imp.hpp"
