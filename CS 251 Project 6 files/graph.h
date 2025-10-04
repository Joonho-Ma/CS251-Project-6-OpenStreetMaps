#pragma once

#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

using namespace std;

/// @brief Simple directed graph using an adjacency list.
/// @tparam VertexT vertex type
/// @tparam WeightT edge weight type
template <typename VertexT, typename WeightT>
class graph {
 private:
  // TODO_STUDENT

  // Define unordered_map
  // ex) A->B with weight 1 : adj_list.add(A,B,1)
  // Covers the case A has 2+ branches --> adj_list.add(A,D,2) 
  unordered_map<VertexT, unordered_map<VertexT, WeightT>> adj_list;
  size_t num_Edges;

  // **compile failed when using numEdges
  
 public:
  /// Default constructor
  graph() {
    // TODO_STUDENT
    adj_list.clear();
    num_Edges = 0;
    // **compile failed when using numEdges
  }

  /// @brief Add the vertex `v` to the graph, must typically be O(1).    - >> using 
  /// @param v
  /// @return true if successfully added; false if it existed already
  bool addVertex(VertexT v) {
    // TODO_STUDENT
    // Functions: insert(), find(), at(), begin(), end(), erase(), 
    //  clear(), contains(), empty(), size()

    // In case 'v' is non-repeated vertex
    if (!adj_list.contains(v)) {
      adj_list.insert({v,unordered_map<VertexT, WeightT>()});
      return true;
    }
    else {
      return false;
    }

    // Otherwise, return false
    return false;
  }

  /// @brief Add or overwrite directed edge in the graph, must typically be
  /// O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight edge weight / label
  /// @return true if successfully added or overwritten;
  ///         false if either vertices isn't in graph
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    // TODO_STUDENT
    
    // -1. check if 'from' and 'to' exists in adj_list
    // 0. check from's validity (non-repeated vertex)
    // Adding the format, unordered_map<from, unordered_map<to, weight>>
    // 1. check if the vertice (from->to) exists
    // if so, return false;
    // if not, insert();
    if (!adj_list.contains(from)) {
      return false;
    }
    else if (!adj_list.contains(to)) {
      return false;
    }

    //if not, check if the vertice is 'new' (non- repeated)
    if (!adj_list[from].contains(to)) {
      num_Edges++;
      adj_list[from][to] = weight;
    }

    // if 'from' and 'to' are both valid, add weight or simply update the weight.
    else {
      adj_list[from][to] = weight;
    }
    return true;
  }

  /// @brief Maybe get the weight associated with a given edge, must typically
  /// be O(1).
  /// @param from starting vertex
  /// @param to ending vertex
  /// @param weight output parameter
  /// @return true if the edge exists, and `weight` is set;
  ///         false if the edge does not exist
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    // TODO_STUDENT
    // 0. check if the edge exist
    if (!adj_list.contains(from)) {
      return false;
    }

    // if (!adj_list[from].contains(to)) {
    //   return false;
    // }

    //access the inner unordered_map
    auto const& temp = adj_list.at(from);
    
    if (!temp.contains(to)) { // if 'to' does not exist, return false
      return false;
    }

    // now check if the weight exists?
    // simply get the weight?
    weight = temp.at(to);

    // weight = adj_list[from][to];
    return true;
    }

  /// @brief Get the out-neighbors of `v`. Must run in at most O(|V|).
  /// @param v
  /// @return vertices that v has an edge to
  set<VertexT> neighbors(VertexT v) const {
    set<VertexT> S;
    // TODO_STUDENT

    // for loop {}
    // check if contains();
    // if so, add to S?

    auto const& temp = adj_list.find(v);

    if (temp != adj_list.end()) {
      for (auto const& [to, weight] : temp->second) {
        S.insert(to);
      }
    }
    return S;
  }

  /// @brief Return a vector containing all vertices in the graph
  vector<VertexT> getVertices() const {
    // TODO_STUDENT
    vector<VertexT> all_Vertex;

    for (auto const& temp: adj_list) {
      all_Vertex.push_back(temp.first);
    }
    return all_Vertex;
  }

  /// @brief Get the number of vertices in the graph. Runs in O(1).
  size_t numVertices() const {
    // TODO_STUDENT
    return adj_list.size();
  }

  /// @brief Get the number of directed edges in the graph. Runs in at most
  /// O(|V|), but should be O(1).
  size_t numEdges() const {
    // TODO_STUDENT
    return num_Edges;
  }
};
