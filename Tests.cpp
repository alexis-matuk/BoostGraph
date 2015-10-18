//
//  Tests.cpp
//  BoostGraph
//
//  Created by alexis matuk on 10/17/15.
//  Copyright (c) 2015 alexis matuk. All rights reserved.
//

#include <stdio.h>
/*


 //============= Insertar vértice en el grafo =================
 Graph g;
 add_vertex(g);
 GVertex v0 = getGVertex(g, 0);
 g[v0].id = 10;
 
 std::pair<adjacency_list<>::vertex_iterator,adjacency_list<>::vertex_iterator> vs = boost::vertices(g);
 
 std::copy(vs.first, vs.second, std::ostream_iterator<adjacency_list<>::vertex_descriptor>{
 std::cout});
 
 //============= Recorrer grafo y tomar el id de los nodos ==============
 
 graph_traits < Graph >::vertex_iterator vi, vend;
 for (boost::tie(vi, vend) = vertices(g); vi != vend; ++vi) {
 std::cout << "(" << g[*vi].id << ")";
 }
 
 
 
 
 //    typedef adjacency_list < listS, vecS, directedS,no_property, property < edge_weight_t, int > > graph_t;
 //    typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
 //    typedef std::pair<int, int> Edge;
 
 //    typedef boost::adjacency_list<boost::setS, boost::vecS,
 //    boost::undirectedS> graph;
 //    graph g;
 //
 //    enum { topLeft, topRight, bottomRight, bottomLeft };
 //
 //    boost::add_edge(topLeft, topRight, g);
 //    boost::add_edge(topRight, bottomRight, g);
 //    boost::add_edge(bottomRight, bottomLeft, g);
 //    boost::add_edge(bottomLeft, topLeft, g);
 //
 //    graph::edge_iterator it, end;
 //    std::tie(it, end) = boost::edges(g);
 //    std::copy(it, end,
 //              std::ostream_iterator<graph::edge_descriptor>{std::cout, "\n"});

 
 
 Graph::edge_iterator start, end;
 for (tie(start, end) = edges(g); start != end; ++start)
 {
 std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(source(*start,g), target(*start,g), g);
 Graph::edge_descriptor edge = edgePair.first;
 std::cout << "Weight: " << edgeWeightMap[edge]<<std::endl;
 }
 
 //property_map<Graph, vertex_distance_t>::type distance = get(vertex_distance, g);
 //property_map<Graph, vertex_index_t>::type indexmap = get(vertex_index, g);
 //typedef std::pair < int, int >E;
 //int num_nodes = num_vertices(g);
 //typedef boost::property_map<Graph, boost::edge_weight_t>::type EdgeWeightMap;
 //EdgeWeightMap edgeWeightMap = get(boost::edge_weight, g);

*/