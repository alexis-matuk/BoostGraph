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
#include <chrono>
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

typedef std::chrono::microseconds micros;

typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;

typedef adjacency_list<listS,vecS,undirectedS,node,EdgeWeightProperty> Graph;

typedef boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap;

typedef graph_traits<Graph>::vertex_descriptor vertex_descriptor;

typedef boost::graph_traits < Graph >::edge_descriptor edge_descriptor;

typedef std::pair<int, int> Edge;

typedef boost::exterior_vertex_property<Graph, float> DistanceProperty;

typedef DistanceProperty::matrix_type DistanceMatrix;

typedef DistanceProperty::matrix_map_type DistanceMatrixMap;

vertex_descriptor addVertex(Graph & g, int n)
{
    vertex_descriptor v0 = boost::add_vertex(g);
    g[v0].id = n;
    return v0;
}

void addEdge(int vertex1, int vertex2, float weight, Graph & g)
{
    add_edge(vertex1-1, vertex2-1, weight, g);
}

void printAlgorithm(std::string alg, micros dur)
{
    std::cout << alg << ": " << dur.count()<< " micros" << std::endl;
    std::cout << std::endl;
}

class custom_dfs_visitor : public boost::default_dfs_visitor
{
public:
    
    template < typename Vertex, typename Graph >
    void discover_vertex(Vertex u, const Graph & g) const
    {
        std::cout << g[u].id << std::endl;
    }
};

class custom_bfs_visitor : public boost::default_bfs_visitor
{
public:
    
    template < typename Vertex, typename Graph >
    void discover_vertex(Vertex u, const Graph & g) const
    {
        std::cout << g[u].id << std::endl;
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



int main(int argc, const char * argv[]) {
    
    Graph g;
    std::vector<vertex_descriptor> verts;
//    for(int i=1; i<15; i++)
//    {
//        verts.push_back(addVertex(g, i));
//    }
//    
//    
//    addEdge(1,4,8,g);
//    addEdge(1,3,8,g);
//    addEdge(2,5,7,g);
//    addEdge(3,10,4,g);
//    addEdge(3,2,7,g);
//    addEdge(3,5,8,g);
//    addEdge(4,7,3,g);
//    addEdge(4,5,1,g);
//    addEdge(4,8,2,g);
//    addEdge(5,6,9,g);
//    addEdge(6,13,4,g);
//    addEdge(7,4,6,g);
//    addEdge(8,9,3,g);
//    addEdge(8,7,3,g);
//    addEdge(9,10,2,g);
//    addEdge(9,12,4,g);
//    addEdge(10,3,10,g);
//    addEdge(10,6,6,g);
//    addEdge(11,12,6,g);
//    addEdge(12,11,8,g);
//    addEdge(12,9,2,g);
//    addEdge(12,14,9,g);
//    addEdge(13,14,6,g);
//    addEdge(14,13,2,g);
    
    for(int i=1; i<6; i++)
    {
        verts.push_back(addVertex(g, i));
    }
    
    addEdge(1,2,5,g);
    addEdge(2,1,5,g);
    addEdge(1,4,3,g);
    addEdge(4,1,3,g);
    addEdge(2,3,6,g);
    addEdge(3,2,6,g);
    
    //--------------------- DFS ---------------------//
    std::cout << "==============DFS===============" << std::endl;
    auto begin = std::chrono::high_resolution_clock::now();
    
    default_dfs_visitor vis;
    depth_first_search(g, visitor(vis));
    
    auto end = std::chrono::high_resolution_clock::now();
    auto alg = std::chrono::duration_cast<micros>(end-begin);
    printAlgorithm("DFS", alg);
    
    //--------------------- BFS ---------------------//
    std::cout << "==============BFS===============" << std::endl;
    begin = std::chrono::high_resolution_clock::now();
    
    default_bfs_visitor vis2;
    breadth_first_search(g, vertex(0,g),visitor(vis2));
    
    end = std::chrono::high_resolution_clock::now();
    alg = std::chrono::duration_cast<micros>(end-begin);
    printAlgorithm("BFS", alg);
    
    //--------------------- Prim ---------------------//
    std::cout << "==============PRIM===============" << std::endl;
    begin = std::chrono::high_resolution_clock::now();
    
    std::vector <vertex_descriptor> p(num_vertices(g));
    prim_minimum_spanning_tree(g, &p[0]);
    
    end = std::chrono::high_resolution_clock::now();
    alg = std::chrono::duration_cast<micros>(end-begin);
    printAlgorithm("Prim", alg);
    
    /*
    for (std::size_t i = 0; i != p.size(); ++i)
    {
        if (p[i] != i)
            std::cout << g[p[i]].id << " --> " << g[i].id << std::endl;
        else
            std::cout << "start -> " << g[i].id << std::endl;
    }
    */
    
    
    //--------------------- Kruskal ---------------------//
    std::cout << "==============Kruskal===============" << std::endl;
    begin = std::chrono::high_resolution_clock::now();
    
    property_map < Graph, edge_weight_t >::type weight = get(edge_weight, g);
    std::vector < edge_descriptor > spanning_tree;
    kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree));
    
    end = std::chrono::high_resolution_clock::now();
    alg = std::chrono::duration_cast<micros>(end-begin);
    printAlgorithm("Kruskal", alg);
    
    /*
    for (std::vector < edge_descriptor >::iterator ei = spanning_tree.begin(); ei != spanning_tree.end(); ++ei)
    {
        std::cout << g[source(*ei, g)].id << " --> " << g[target(*ei, g)].id << " peso: " << weight[*ei] << std::endl;
    }
    */
     
    
    
    //--------------------- Dijkstra ---------------------//
    
    std::cout << "==============Dijkstra===============" << std::endl;
    begin = std::chrono::high_resolution_clock::now();
    
    std::vector<vertex_descriptor> parents(boost::num_vertices(g));
    std::vector<float> distances(boost::num_vertices(g));
    boost::dijkstra_shortest_paths(g, verts[0], boost::predecessor_map(&parents[0]).distance_map(&distances[0]));
    
    end = std::chrono::high_resolution_clock::now();
    alg = std::chrono::duration_cast<micros>(end-begin);
    printAlgorithm("Dijkstra", alg);
    
    
    std::cout << "distances and parents:" << std::endl;
    boost::graph_traits < Graph >::vertex_iterator vertexIterator, vend;
    std::cout << "Vértice orígen: " << g[verts[0]].id << std::endl;
    /*
    for (boost::tie(vertexIterator, vend) = boost::vertices(g); vertexIterator != vend; ++vertexIterator)
    {
        std::cout << "distancia a " << *vertexIterator+1 << " = " << distances[*vertexIterator] << ", ";
        std::cout << "padre de " << *vertexIterator+1 << " = " << parents[*vertexIterator]+1 << std::endl;
    }
*/
    
    
    //--------------------- Floyd Warshall ---------------------//
    
    std::cout << "==============Floyd Warshall===============" << std::endl;
    begin = std::chrono::high_resolution_clock::now();
    
    std::map<vertex_descriptor, std::map<vertex_descriptor, float> > matrix;
    EdgeWeightMap weight_pmap = boost::get(boost::edge_weight, g);
    
    DistanceMatrix dist(num_vertices(g));
    DistanceMatrixMap dm(dist, g);
    
    bool noNegCycles = floyd_warshall_all_pairs_shortest_paths (g, dm, weight_map(weight_pmap));
    
    end = std::chrono::high_resolution_clock::now();
    alg = std::chrono::duration_cast<micros>(end-begin);
    printAlgorithm("Floyd Warshall", alg);
    
    /*
    std::cout << "Distance matrix: " << std::endl;
    for (std::size_t i = 0; i < num_vertices(g); ++i)
    {
        for (std::size_t j = 0; j < num_vertices(g); ++j)
        {
            std::cout << "From vertex " << i+1 << " to " << j+1 << " : ";
            if(dist[i][j] == std::numeric_limits<float>::max())
                std::cout << "inf" << std::endl;
            else
                std::cout << dist[i][j] << std::endl;
        }
        std::cout << std::endl;
    }
    */
    
}
