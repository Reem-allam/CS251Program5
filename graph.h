/*graph.h*/

//
// Reem Allam
//
// Basic graph class using adjacency list representation.
//
// original author: Prof. Scott Reckinger
// University of Illinois Chicago
// CS 251: Summer 2023
//

#pragma once

#include <iostream>
#include <list>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

template <typename VertexT, typename WeightT> class graph {
private:
  struct EdgeData {
    bool EdgeExists;
    WeightT Weight;

    EdgeData() {
      EdgeExists = false; // initially no edge, and no weight
    }
  };
  unordered_map<VertexT, unordered_map<VertexT, EdgeData>> AdjList;
  //
  // We are using adjacency matrix implementation, where rows
  // are the starting vertex and cols are the ending vertex.
  // We keep track of the vertices in the Vertices vector,
  // where the vertex's position in the vector --- 0, 1, 2,
  // 3, 4, 5, ... --- denotes the row in the adjacency matrix
  // where their edges are found.  Example: if vertex "ORD" is
  // in position 1 of the Vertices vector, then row 1 of
  // AdjMatrix are the edges that start at "ORD" and lead to
  // other vertices.
  //

  //
  // _LookupVertex
  //
  // Finds the vertex in the Vertices vector and returns it's
  // index position if found, otherwise returns -1.
  //

public:
  //
  // constructor:
  //
  // Constructs an empty graph where n is the max # of vertices
  // you expect the graph to contain.
  //
  // NOTE: the graph is implemented using an adjacency matrix.
  // If n exceeds the dimensions of this matrix, an exception
  // will be thrown to let you know that this implementation
  // will not suffice.
  //
  graph() {}
  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const { return static_cast<int>(AdjList.size()); }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
    int count = 0;

    //
    // loop through the adjacency list and count how many
    // edges currently exist:
    //

    for (auto &fromNode : AdjList) {

      count += fromNode.second.size();
    }
    return count;
  }

  //
  // addVertex
  //
  // Adds the vertex v to the graph if there's room, and if so
  // returns true.  If the graph is full, or the vertex already
  // exists in the graph, then false is returned.
  //

  bool addVertex(VertexT v) {

    // is the vertex already in the graph?  If so, we do not
    // insert again otherwise Vertices may fill with duplicates:
    //
    //
    // if we get here, vertex does not exist so insert.  Where
    // we insert becomes the rows and col position for this
    // vertex in the adjacency matrix.
    //

    if (AdjList.find(v) == AdjList.end()) {
      AdjList[v] = unordered_map<VertexT, EdgeData>();
      return true;
    }

    // return AdjList.insert({v,{}}).second;
    return false;
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist or for some reason the
  // graph is full, false is returned.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    auto fromIt = AdjList.find(from);
    auto toIt = AdjList.find(to);

    if (fromIt == AdjList.end() || toIt == AdjList.end()) {
      return false;
    }

    auto &neighbors = fromIt->second;
    neighbors[to].EdgeExists = true;
    neighbors[to].Weight = weight;

    return true;
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT &weight) const {
    auto fromIt = AdjList.find(from);

    if (fromIt == AdjList.end()) {
      return false;
    }

    const auto &neighbors = fromIt->second;
    auto toIt = neighbors.find(to);
    if (toIt == neighbors.end() || !toIt->second.EdgeExists) {
      return false;
    }

    weight = toIt->second.Weight;
    return true;
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;
    auto it = AdjList.find(v);

    if (it != AdjList.end()) {
      const auto &neighbors = it->second;
      for (const auto &neighbor : neighbors) {
        if (neighbor.second.EdgeExists) {
          S.insert(neighbor.first);
        }
      }
    }

    return S;
  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
    vector<VertexT> vertices;
    for (auto &entry : AdjList) {
      vertices.push_back(entry.first);
    }
    return vertices;
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream &output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;

    for (const auto &vertex : getVertices()) {
      output << " " << vertex << endl;
    }

    output << endl;
    output << "**Edges:" << endl;
    for (const auto &fromNode : AdjList) {
      output << " " << fromNode.first << ": ";
      const auto &neighbors = fromNode.second;
      for (const auto &neighbor : neighbors) {
        if (neighbor.second.EdgeExists) {
          output << "(" << fromNode.first << "," << neighbor.first << ","
                 << neighbor.second.Weight << ") ";
        }
      }
      output << endl;
    }
    output << "**************************************************" << endl;
  }
};
