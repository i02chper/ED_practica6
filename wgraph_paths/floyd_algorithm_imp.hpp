/**
 * @file floyd_algorithm_imp.hpp
 *
 * CopyRight F. J. Madrid-Cuevas <fjmadrid@uco.es>
 *
 * Sólo se permite el uso de este código en la docencia de las asignaturas sobre
 * Estructuras de Datos de la Universidad de Córdoba.
 *
 * Está prohibido su uso para cualquier otro objetivo.
 */
#pragma once

#include <utility>
#include <stack>
#include "floyd_algorithm.hpp"

#ifdef __VERBOSE__
#include <iostream>
#include <iomanip>
extern int Verbose_level;
#endif

template<class T>
void floyd_algorithm(const typename WGraph<T>::Ref g, FMatrix::Ref& D,
                     IMatrix::Ref& I)
{
    D = g->weight_matrix();
    I = IMatrix::create(g->size(), g->size(), -1);

    //TODO: Codify the Floyd algorithm.
    
    
    //
}

std::list<size_t> floyd_path(size_t src, size_t dst, IMatrix::Ref I)
{
    //Prec: distance (u,v) < inf
    std::list<size_t> path;

    //TODO: Find the path.
    //Hint: Think first. Is it necessary to build a binary tree? or it
    //is enough to do an first-depth search using an iterative approach with
    //a stack of pairs (u,v).

    
    
    //
    return path;
}




