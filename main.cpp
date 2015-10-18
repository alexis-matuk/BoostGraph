//
//  main.cpp
//  BoostGraph
//
//  Created by alexis matuk on 10/11/15.
//  Copyright (c) 2015 alexis matuk. All rights reserved.
//

#include <iostream>
#include <deque>
#include <iterator>
#include <vector>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/exterior_property.hpp>

using namespace boost;


struct node{
    float id;
};

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;

typedef adjacency_list<listS,vecS,directedS,node,EdgeWeightProperty> Graph;

typedef boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap;

typedef graph_traits<Graph>::vertex_descriptor vertex_descriptor;

typedef boost::graph_traits < Graph >::edge_descriptor edge_descriptor;

typedef std::pair<int, int> Edge;

typedef boost::exterior_vertex_property<Graph, float> DistanceProperty;

typedef DistanceProperty::matrix_type DistanceMatrix;

typedef DistanceProperty::matrix_map_type DistanceMatrixMap;

class custom_dfs_visitor : public boost::default_dfs_visitor
{
public:
    
    template < typename Vertex, typename Graph >
    void discover_vertex(Vertex u, const Graph & g) const
    {
        std::cout << u << std::endl;
    }
};

class custom_bfs_visitor : public boost::default_bfs_visitor
{
public:
    
    template < typename Vertex, typename Graph >
    void discover_vertex(Vertex u, const Graph & g) const
    {
        std::cout << u << std::endl;
    }
};

bool vertexExists(Graph & g, int n, int & t)
{
    int pos = 0;
    Graph::vertex_iterator start,end;
    for(tie(start,end) = vertices(g); start!=end; start++)
    {
        if(g[*start].id == n)
        {
            t = pos;
            return true;
        }
        pos++;
    }
    return false;
}

bool edgeExists(Graph & g, int v1, int v2, float w)
{
    Graph::edge_iterator edgeIt, edgeEnd;
    for (tie(edgeIt, edgeEnd) = edges(g); edgeIt != edgeEnd; ++edgeIt)
    {
        if(source(*edgeIt, g) == v1 && target(*edgeIt, g) == v2 && boost::get(edge_weight, g, *edgeIt) == w)
            return true;
    }
    return false;
}

vertex_descriptor addVertex(Graph & g, int n)
{
    vertex_descriptor v0 = boost::add_vertex(g);
    g[v0].id = n;
    return v0;
}

int main(int argc, const char * argv[]) {
    
    Graph g;
    bool negativeEdge = false;
    int choice;
    bool done = false;
    while(!done)
    {
        std::cout << "===== Boost Library Implementation ===== " << std::endl;
        std::cout << "1) Insertar vértice en el grafo" << std::endl;
        std::cout << "2) Insertar arista en el grafo" << std::endl;
        std::cout << "3) Eliminar vértice del grafo" << std::endl;
        std::cout << "4) Eliminar arista del grafo" << std::endl;
        std::cout << "5) Depth First Traversal" << std::endl;
        std::cout << "6) Breadth First Traversal" << std::endl;
        std::cout << "7) Algoritmo de Prim" << std::endl;
        std::cout << "8) Algoritmo de Kruskal" << std::endl;
        std::cout << "9) Ruta mínima con Dijkstra" << std::endl;
        std::cout << "10) Ruta mínima con Floyd-Warshall" << std::endl;
        std::cout << "11) Imprimir grafo" << std::endl;
        std::cout << "12) Salir" << std::endl;
        std::cout << "Elige una opción: ";
        while(!(std::cin >> choice)){
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Entrada inválida, intenta otra vez: ";
        }
        switch(choice)
        {
            case 1:{
                int v;
                std::cout << "Id del nodo a agregar: ";
                while(!(std::cin >> v)){
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Entrada inválida, intenta otra vez: ";
                }
                int posv;
                if(vertexExists(g, v, posv))
                {
                    std::cout << "El vértice con id: " << v << " ya existe" << std::endl;
                }
                else
                {
                    vertex_descriptor v0 = addVertex(g,v);
                    std::cout << "Vértice " << v0 << " con id: " << g[v0].id << " Insertado correctamente" << std::endl;
                }
                break;}
            case 2:{
                float v1, v2;
                int pos1, pos2;
                std::cout << "Vértice 1: ";
                while(!(std::cin >> v1)){
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Entrada inválida, intenta otra vez: ";
                }
                if(!vertexExists(g, v1, pos1))
                {
                    std::cout << "El vértice con id " << v1 << " no existe, debe crearse antes" << std::endl;
                    break;
                }
                else
                {
                    std::cin.ignore();
                    std::cout << "Vértice 2: ";
                    while(!(std::cin >> v2)){
                        std::cin.clear();
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                        std::cout << "Entrada inválida, intenta otra vez: ";
                    }
                    
                    if(!vertexExists(g, v2, pos2))
                    {
                        std::cout << "El vértice con id " << v2 << " no existe, debe crearse antes " << std::endl;
                        break;
                    }
                    else
                    {
                        float weight;
                        std::cin.ignore();
                        std::cout << "Peso de la arista a insertar: ";
                        while(!(std::cin >> weight)){
                            std::cin.clear();
                            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                            std::cout << "Entrada inválida, intenta otra vez: ";
                        }
                        EdgeWeightProperty w = weight;
                        if(edgeExists(g,pos1,pos2,weight))
                        {
                            std::cout << "La arista de " << pos1 << " a " << pos2 << " con peso " << weight << " ya existe" << std::endl;
                        }
                        else
                        {
                            if(weight < 0)
                            {
                                negativeEdge = true;
                                std::cout << "Por agregar una arista negativa ya nos e podrá ejecutar el algoritmo de Dijkstra" << std::endl;
                            }
                            add_edge(pos1,pos2,w,g);
                            std::cout << "Arista añadida correctamente" << std::endl;
                        }
                    }
                }
                break;}
            case 3:{
                float v;
                int pos;
                std::cout << "Id del vértice a borrar: ";
                while(!(std::cin >> v)){
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Entrada inválida, intenta otra vez: ";
                }
                if(vertexExists(g, v, pos))
                {
                    graph_traits<Graph>::vertex_iterator start,end,next;
                    boost::tie(start, end) = vertices(g);
                    for (next = start; start != end; start = next) {
                        ++next;
                        if(g[*start].id == v)
                        {
                            clear_vertex(*start, g);
                            remove_vertex(*start,g);
                            std::cout << "Vértice borrado exitosamente" << std::endl;
                        }
                    }
                }
                else
                {
                    std::cout << "El vértice con id " << v << " no existe " << std::endl;
                }
                break;}
            case 4:{
                float v1,v2;
                int pos1,pos2;
                std::cout << "Id del vértice 1 conectado por la arista: ";
                while(!(std::cin >> v1)){
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    std::cout << "Entrada inválida, intenta otra vez: ";
                }
                if(vertexExists(g, v1,pos1))
                {
                    std::cin.ignore();
                    std::cout << "Id del vértice 2 conectado por la arista: ";
                    while(!(std::cin >> v2)){
                        std::cin.clear();
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                        std::cout << "Entrada inválida, intenta otra vez: ";
                    }
                    if(vertexExists(g, v2,pos2))
                    {
                        if(edge(pos1,pos2,g).second)
                        {
                            std::pair<Graph::edge_descriptor, bool> retrievedEdge = boost::edge(pos1, pos2, g);
                            remove_edge(retrievedEdge.first,g);
                            std::cout << "Arista borrada exitosamente" << std::endl;
                        }
                        else
                        {
                            std::cout << "La arista de " << v1 << " a " << v2 << " no existe" << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "El vértice con id " << v2 << " no existe " << std::endl;
                    }
                }
                else
                {
                    std::cout << "El vértice con id " << v1 << " no existe " << std::endl;
                }
                
                break;}
            case 5:{
                if(num_edges(g) == 0)
                    std::cout << "No hay aristas por visitar" << std::endl;
                else
                {
                    custom_dfs_visitor vis;
                    depth_first_search(g, visitor(vis));
                }
                break;}
            case 6:{
                if(num_edges(g) == 0)
                    std::cout << "No hay aristas por visitar" << std::endl;
                else
                {
                    custom_bfs_visitor vis;
                    breadth_first_search(g, vertex(0,g),visitor(vis));
                }
                break;}
            case 7:{
                std::vector <vertex_descriptor> p(num_vertices(g));
                prim_minimum_spanning_tree(g, &p[0]);
                for (std::size_t i = 0; i != p.size(); ++i)
                {
                    if (p[i] != i)
                        std::cout << g[p[i]].id << " <--> " << g[i].id << std::endl;
                    else
                        std::cout << "start -> " << g[i].id << std::endl;
                }
                break;}
            case 8:{
                property_map < Graph, edge_weight_t >::type weight = get(edge_weight, g);
                std::vector < edge_descriptor > spanning_tree;
                
                kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree));
                
                for (std::vector < edge_descriptor >::iterator ei = spanning_tree.begin(); ei != spanning_tree.end(); ++ei)
                {
                    std::cout << g[source(*ei, g)].id << " <--> " << g[target(*ei, g)].id << " peso: " << weight[*ei] << std::endl;
                }
                break;}
            case 9:{
                if(!negativeEdge)
                {
                    std::vector<vertex_descriptor> parents(boost::num_vertices(g));
                    std::vector<float> distances(boost::num_vertices(g));
                    std::cout << "Id del vértice origen: ";
                    float v;
                    int posv;
                    while(!(std::cin >> v)){
                        std::cin.clear();
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                        std::cout << "Entrada inválida, intenta otra vez: ";
                    }
                    if(vertexExists(g, v, posv))
                    {
                        boost::dijkstra_shortest_paths(g, posv, boost::predecessor_map(&parents[0]).distance_map(&distances[0]));
                        std::cout << "distances and parents:" << std::endl;
                        boost::graph_traits < Graph >::vertex_iterator vertexIterator, vend;
                        std::cout << "Vértice orígen: " << v << std::endl;
                        for (boost::tie(vertexIterator, vend) = boost::vertices(g); vertexIterator != vend; ++vertexIterator)
                        {
                            std::cout << "distancia a " << *vertexIterator << " = " << distances[*vertexIterator] << ", ";
                            std::cout << "padre de " << *vertexIterator << " = " << parents[*vertexIterator] << std::endl;
                        }
                    }
                    else
                    {
                        std::cout << "El vértice con id " << v << " no existe " << std::endl;
                    }
                }
                else
                {
                    std::cout << "El grafo tiene aristas con peso negativo, no se puede ejecutar el algoritmo de Dijkstra correctamente" << std::endl;
                }
                break;}
            case 10:{
                std::map<vertex_descriptor, std::map<vertex_descriptor, float> > matrix;
                EdgeWeightMap weight_pmap = boost::get(boost::edge_weight, g);
                
                DistanceMatrix distances(num_vertices(g));
                DistanceMatrixMap dm(distances, g);
                
                bool noNegCycles = floyd_warshall_all_pairs_shortest_paths (g, dm, weight_map(weight_pmap));
                if(noNegCycles)
                {
                    std::cout << "Distance matrix: " << std::endl;
                    for (std::size_t i = 0; i < num_vertices(g); ++i)
                    {
                        for (std::size_t j = 0; j < num_vertices(g); ++j)
                        {
                            std::cout << "From vertex " << i+1 << " to " << j+1 << " : ";
                            if(distances[i][j] == std::numeric_limits<float>::max())
                                std::cout << "inf" << std::endl;
                            else
                                std::cout << distances[i][j] << std::endl;
                        }
                        std::cout << std::endl;
                    }
                }
                else
                {
                    std::cout << "Se encontraron ciclos negativos" << std::endl;
                }
                break;}
            case 11:{
                Graph::vertex_iterator start,end;
                if(num_vertices(g) == 0)
                    std::cout << "No hay vértices en el grafo" << std::endl;
                else
                {
                    std::cout << "Vértices" << std::endl;
                    int pos = 0;
                    for (tie(start, end) = vertices(g); start != end; ++start)
                    {
                        std::cout << "Vértice " << pos << " con id: " << g[*start].id << std::endl;
                        pos++;
                    }
                }
                
                if(num_edges(g) == 0)
                {
                    std::cout << "No hay aristas en el grafo" << std::endl;
                }
                else
                {
                    std::cout << "Aristas" << std::endl;
                    Graph::edge_iterator edgeIt, edgeEnd;
                    for (tie(edgeIt, edgeEnd) = edges(g); edgeIt != edgeEnd; ++edgeIt)
                    {
                        std::cout << "arista " << g[source(*edgeIt, g)].id << "-->" << g[target(*edgeIt, g)].id << " peso: "<< boost::get(edge_weight, g, *edgeIt) <<  "\n";
                    }
                }
                break;}
            case 12:{
                std::cout << "Adios!" << std::endl;
                done = true;
                break;}
            default:
                std::cout << "Opción inválida, elige otra" << std::endl;
                break;
        }
    }
}
