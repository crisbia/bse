#ifndef _BSE_GRAPH_H_INCLUDED
#define _BSE_GRAPH_H_INCLUDED

#include <vector>
#include <list>
#include <algorithm>

// external dependancies
#include "bseTypes.h"
#include "bseBinaryHeap.h"
#include "bseBucket.h"

namespace bse
{
namespace ai
{

class AIScene;

class GraphConnection;
class GraphNode;
class GraphNodeData;
class Graph;

typedef std::vector<GraphNode*> GraphNodesList;
typedef std::vector<const GraphNode*> GraphNodesListConst;
typedef std::vector<GraphNode*>::iterator GraphNodesListIter;
typedef std::vector<GraphConnection*> GraphConnectionsList;
typedef std::vector<const GraphConnection*> GraphConnectionsListConst;

typedef std::list<const GraphConnection*> GraphPath;

//---------------------------------------------------------------------------------------------------------------------
// Graph connection
class GraphConnection
{
public:
  GraphConnection(GraphNode* source, GraphNode* dest, bse::Real cost);
  virtual ~GraphConnection();

  GraphNode* getSource() const { return m_source; }
  GraphNode* getDest()   const { return m_dest; }
  const bse::Real getCost() const { return m_cost; }

private:
  GraphNode* m_source;
  GraphNode* m_dest;
  bse::Real m_cost;
  const Graph* m_graph;

  // usually helpful
  bse::UserData m_userData;
};

//---------------------------------------------------------------------------------------------------------------------
// Graph node
class GraphNode
{
public:
  GraphNode(Graph* graph, GraphNodeData* data);

  const bse::UInt getID() const { return m_id; }
  const bse::UInt getNumberOfConnections() const { return m_connections.size(); }
  const GraphConnectionsListConst& getConnections() const { return m_connections; }
  const GraphConnection* getConnection(bse::UInt iConn) const { return m_connections[iConn]; }
  GraphNode* getDestNode(bse::UInt iConn) const;

  void addConnection(GraphConnection* connection);
  void removeConnection(GraphConnection* connection);

  const Graph* getGraph() const { return m_graph; }

  void reset()
  {
    setCostToDest(0);
  }

  void setCostToDest(bse::Real costToDest=0) { m_costToDest = costToDest; }

  GraphNodeData* getNodeData() const { return m_nodeData; }
  void setNodeData(GraphNodeData* data) { m_nodeData = data; }
  bool hasConnectionWithDest(const GraphNode* node) const;
private:
  const Graph* m_graph;
  GraphNodeData* m_nodeData;

  bse::Real m_costToDest; // estimated cost

  GraphConnectionsListConst m_connections;

  // usually helpful
  bse::UserData m_userData;

  bse::UInt m_id;
};


// interface. application must implement the heuristic
class GraphHeuristic
{
public:
  virtual const bse::Real estimatePathCost(const GraphNode* source, const GraphNode* dest) const = 0;
};

class GraphNodeData
{
public:
  GraphNodeData(void* data=0) :
      m_data(data)
  {
  }

  void setData(void* data) { m_data = data; }
  void* getData() const { return m_data; }

private:
  void* m_data;
};

// class used to store nodes data during the pathfinder execution
// this allows to perform concurrent executions on the same graph
class GraphNodeGhost
{
public:
  bool operator<(const GraphNodeGhost& node)
  {
    if (getTotalCost() > node.getTotalCost()) return true;
    return false;
  }

  GraphNodeGhost(GraphNode* n=0) :
      node(n),
      costSoFar(0), costToDest(0),parentInPath(0),connToWalk(0),
      indexOnHeap(0)
  {
  }

  const GraphNode* node;
  bse::Real costSoFar;
  bse::Real costToDest; // estimated cost
  GraphNodeGhost* parentInPath;
  const GraphConnection* connToWalk;
  size_t  indexOnHeap;

  float getTotalCost() const { return costSoFar + costToDest; }
};

struct nodeBetterThan : public std::function<bool(const GraphNodeGhost*, const GraphNodeGhost*)>
{
public:
  bool operator() (const GraphNodeGhost* a, const GraphNodeGhost* b) const
  {
    return a->getTotalCost() < b->getTotalCost();
  }
};

struct nodeSwap : public std::function<void(GraphNodeGhost* , GraphNodeGhost*)>
{
public:
  void operator() (GraphNodeGhost* a, GraphNodeGhost* b)
  {
    size_t tempIndex = a->indexOnHeap;
    a->indexOnHeap = b->indexOnHeap;
    b->indexOnHeap = tempIndex;
  }
};

struct nodeIsEqual : public std::function<bool(GraphNodeGhost*, GraphNodeGhost*)>
{
public:
  bool operator() (GraphNodeGhost* a, GraphNodeGhost* b) const
  {
    return a->node == b->node;
  }
};

struct nodeElementValue : public std::function<float(GraphNodeGhost*)>
{
public:
  float operator() (GraphNodeGhost* a) const
  {
    return a->costSoFar;
  }
};

struct nodeID : public std::function<int(GraphNodeGhost*)>
{
public:
  int operator() (GraphNodeGhost* n) const
  {
    return n->node->getID();
  }
};

class SearchStats
{
public:
  SearchStats() :
      success(false),
      visitedNodes(0),
      pathCost(0),
      searchTime(0.0f)
  {

  }
  bool    success;
  bse::UInt visitedNodes;
  float   pathCost;
  float   searchTime;
};

typedef BinaryHeap<GraphNodeGhost*, nodeBetterThan, nodeSwap> OpenListHeap;
typedef Bucket<GraphNodeGhost*, nodeIsEqual, nodeID>          OpenListBucket;

class SearchParameters
{
public:
    typedef enum
    {
        INVALID_SEARCH_TYPE = -1,
        SEARCH_TYPE_COMPLETE,               // non interruptable search. explore as much as possible, without time limits.
        SEARCH_TYPE_TIME_LIMITED,           // interruptable search. returns the best found node when the time elapses.
        SEARCH_TYPE_VISITED_NODES_LIMITED,  // interruptable search. returns the best found node when a given number of nodes has been visited.
        NUM_VALID_SEARCH_TYPES
    } SearchType;

    typedef enum
    {
        INVALID_RESULT_TYPE = -1,
        RESULT_TYPE_PATH_TO_BEST_NODE_ONFAIL, // returns the path to the best node in the open list (this is sometimes unstable)
        RESULT_TYPE_EMPTY_PATH_ONFAIL,        // if the search fails, the path structure given is reset before returning
        RESULT_TYPE_NOACTION_ONFAIL,          // if the search fails, the path structure is not changed.
        NUM_VALID_RESULT_TYPES
    } ResultType;

    SearchParameters() :
      searchType(SEARCH_TYPE_COMPLETE),
      resultType(RESULT_TYPE_EMPTY_PATH_ONFAIL)
    {

    }

    SearchType searchType;
    ResultType resultType;

    union
    {
        bse::Real timeLimit;
        bse::UInt visitedNodesLimit;
    } params;
};

// Graph
class Graph
{
  friend class AIScene;
private:
  Graph(AIScene* scene, const GraphHeuristic* heuristic);
  virtual ~Graph();

public:
  // allows creation with null node data
  GraphNode* createNode(GraphNodeData* data=0);
  void removeNode(GraphNode* node);

  GraphConnection* createConnection(GraphNode* source, GraphNode* dest, bse::Real connectionCost);
  void removeConnection(const GraphConnection* connection);

////////////////////////////////
//// debug accessors
  const int getNumberOfNodes() const { return (int)m_nodes.size(); }
  const int getNumberOfConnections() const { return (int)m_connections.size(); }

  inline GraphNode* getNode(int iNode) const { return m_nodes[iNode]; }
  inline GraphConnection* getConnection(int iConn) const { return m_connections[iConn]; }

  GraphNode* getLastSourceNode() const { return m_lastSourceNode; }
  GraphNode* getLastDestNode() const { return m_lastDestNode; }
/////////////////////////////////////////


  // use maps to access nodes
  const GraphNode* getNodeFromData(const GraphNodeData* data) const;
  const GraphNodeData* getDataFromNode(const GraphNode* node) const;

  /**
   * \brief Performs a path finding
   */
  bool computeShortestPath(
	  const GraphNode* start,			// start search node
	  const GraphNode* end,			// goal search node
	  GraphPath& path,			// returned path
	  const SearchParameters& searchParams = SearchParameters(),
	  SearchStats* stats=0		// optional statistics (slow)
	  );


  /**
   * \brief Returns true if a direct connection between node1 and node2 exists
   */
  bool nodesAreConnected(const GraphNode* node1, const GraphNode* node2) const;

private:
  bse::UserData m_userData;

  GraphConnectionsList m_connections;
  GraphNodesList       m_nodes;

  const GraphHeuristic* m_heuristic; // mandatory!!!

//// debug
  GraphNode* m_lastSourceNode;
  GraphNode* m_lastDestNode;

  unsigned int m_lastGeneratedId;
public:
  unsigned int generateNewId() { return m_lastGeneratedId++; }

protected:
  AIScene* m_scene;
};

}
}

#endif // _BSE_GRAPH_H_INCLUDED
