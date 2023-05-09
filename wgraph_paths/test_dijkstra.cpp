/**
 * @file test_dijkstra.cpp
 *
 * CopyRight F. J. Madrid-Cuevas <fjmadrid@uco.es>
 *
 * Sólo se permite el uso de este código en la docencia de las asignaturas sobre
 * Estructuras de Datos de la Universidad de Córdoba.
 *
 * Está prohibido su uso para cualquier otro objetivo.
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <exception>
#include <vector>

#include "item.hpp"
#include "city.hpp"
#include "wgraph.hpp"
#include "dijkstra_algorithm.hpp"


int Verbose_level=0;

template<class T>
std::ostream&
operator<<(std::ostream& out, const std::vector<T>& v)
{
    out << '[';
    for(size_t i=0;i<v.size();++i)
        out << ' ' << v[i];
    out << " ]";
    return out;
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

    std::vector<size_t> predecessors;
    std::vector<float> distances;
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
    dijkstra_algorithm<T> (g, predecessors, distances);
    std::cout << "Predecessors: " << predecessors << std::endl;
    std::cout << "Distances   : " << distances << std::endl;
    auto path = dijkstra_path(src, dst, predecessors);
    std::cout << "Path from: '" << src_key << "' to '" << dst_key << "': ";
    for (auto v: path)
    {
        std::cout << ' ' << g->node(v)->item().key();
    }
    std::cout << " distance: " << distances[dst] << std::endl;
    return EXIT_SUCCESS;
}


int
main(int argc, const char* argv[])
{
    int exit_code = EXIT_SUCCESS;
    try
    {
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

