/**
 * @file test_floyd.cpp
 *
 * CopyRight F. J. Madrid-Cuevas <fjmadrid@uco.es>
 *
 * Sólo se permite el uso de este código en la docencia de las asignaturas sobre
 * Estructuras de Datos de la Universidad de Córdoba.
 *
 * Está prohibido su uso para cualquier otro objetivo.
 */
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <exception>
#include <vector>
#include <valarray>

#include "item.hpp"
#include "city.hpp"
#include "wgraph.hpp"
#include "floyd_algorithm.hpp"


int Verbose_level=0;

template <class T>
void print_I_matrix(typename WGraph<T>::Ref g, const IMatrix::Ref I)
{
    std::cout << "  ";
    for (size_t i=0;i<g->size();++i)
        std::cout << g->node(i)->item().key() << ' ';
    std::cout << std::endl;
    for (size_t r=0;r<g->size();++r)
    {
        std::cout << g->node(r)->item().key() << ' ';
        for (size_t c=0;c<g->size();++c)
        {
            if (I->get(r,c)==-1)
                std::cout << "- ";
            else
                std::cout << g->node(I->get(r,c))->item().key() << ' ';
        }
        std::cout << std::endl;
    }
}

template <class T>
void print_D_matrix(typename WGraph<T>::Ref g, const FMatrix::Ref D)
{
    std::cout << "  ";
    for (size_t i=0;i<g->size();++i)
        std::cout << ' ' << g->node(i)->item().key() << ' ';
    std::cout << std::endl;

    for (size_t r=0;r<g->size();++r)
    {
        std::cout << g->node(r)->item().key() << ' ';
        for (size_t c=0; c<g->size(); ++c)
        {
            if (D->get(r, c)==std::numeric_limits<float>::infinity())
                std::cout << " - ";
            else
                std::cout << std::setw(2) << D->get(r,c) << ' ';
        }
        std::cout << std::endl;
    }
}

template<class T>
int
do_the_work(std::istream& input_file)
{
    auto g = create_wgraph<T>(input_file);
    if (!g)
    {
        std::cerr << "Error: could not load a valid wgraph." << std::endl;
        return EXIT_FAILURE;
    }
    typename T::key_t src_key;
    typename T::key_t dst_key;
    if (!(input_file >> src_key >> dst_key))
    {
        std::cerr << "Error: could not read start/end vertices." << std::endl;
        return EXIT_FAILURE;
    }
    if (!g->find_node(dst_key))
    {
        std::cerr << "Error: could not find dst node '"
                  << dst_key << "'." << std::endl;
        return EXIT_FAILURE;
    }
    size_t dst  = g->current_node()->label();
    if (!g->find_node(src_key))
    {
        std::cerr << "Error: could not find src node '"
                  << src_key << "'." << std::endl;
        return EXIT_FAILURE;
    }
    size_t src  = g->current_node()->label();
    FMatrix::Ref D;
    IMatrix::Ref I;
    floyd_algorithm<T> (g, D, I);
    std::cout << "D matrix: " << std::endl;
    print_D_matrix<T>(g, D);
    std::cout << "I matrix:" << std::endl;
    print_I_matrix<T>(g, I);

    std::list<size_t> path;
    if (D->get(src, dst)<std::numeric_limits<float>::infinity())
        path = floyd_path(src, dst, I);
    else
        path.push_back(dst);

    std::cout << "Path from: '" << src_key << "' to '" << dst_key << "': ";
    for (auto v: path)
    {
        std::cout << ' ' << g->node(v)->item().key();
    }
    std::cout << " distance: " << D->get(src, dst) << std::endl;

    return EXIT_SUCCESS;
}

int
main(int argc, const char* argv[])
{
    int exit_code = EXIT_SUCCESS;
    try
    {
        std::cout << std::unitbuf;

        if (argc < 2 || argc > 3)
        {
            std::cerr << "Usage: " << argv[0] << " [-v] filename" << std::endl;
            return EXIT_FAILURE;
        }
        std::ifstream input_file;
        if (std::string(argv[1])==std::string("-v"))
        {
            Verbose_level=1;
            input_file.open(argv[2]);
        }
        else
            input_file.open(argv[1]);
        if (!input_file)
        {
            std::cerr << "Error: could not open input filename '" << argv[1] << "'." << std::endl;
            return EXIT_FAILURE;
        }
        std::string type;
        input_file >> type;
        input_file.ignore();
        if (type=="INT")
            exit_code = do_the_work<Int>(input_file);
        else if (type=="STRING")
            exit_code = do_the_work<String>(input_file);
        else if(type=="CITY")
            exit_code = do_the_work<City>(input_file);
        else
        {
            std::cerr << "Error: unknown graph type." << std::endl;
            return EXIT_FAILURE;
        }
    }
    catch(std::runtime_error &e)
    {
        std::cerr << "Run time exception: " << e.what() << std::endl;
        exit_code = EXIT_FAILURE;
    }
    catch(...)
    {
        std::cerr << "Catched unknown exception!." << std::endl;
        exit_code = EXIT_FAILURE;
    }
    return exit_code;
}

