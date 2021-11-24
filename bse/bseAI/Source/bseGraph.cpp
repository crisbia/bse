#include "bseCommons.h"
#include "bseAIContext.h"
#include "bseGraph.h"
#include "bseAIScene.h"
#include "bseProfilingTools.h"
#include "bseBitField.h"

namespace bse
{
namespace ai
{

typedef FrameObjectsPool<GraphNodeGhost, 1024> GraphNodesPool;

//---------------------------------------------------------------------------------------------------------------------
// Graph connection
//---------------------------------------------------------------------------------------------------------------------
GraphConnection::GraphConnection(GraphNode* source, GraphNode* dest, bse::Real cost) :
  m_source(source),
  m_dest(dest),
  m_cost(cost),
  m_userData(0)
{
  BSE_ASSERT( source && dest );
  BSE_ASSERT( source->getGraph() == dest->getGraph() );

  // remember connection in the source
  m_source->addConnection(this);
}

//---------------------------------------------------------------------------------------------------------------------
GraphConnection::~GraphConnection()
{
  m_source->removeConnection(this);
}

//---------------------------------------------------------------------------------------------------------------------
// Graph node
//---------------------------------------------------------------------------------------------------------------------
GraphNode::GraphNode(Graph* graph, GraphNodeData* data) :
  m_graph(graph),
  m_nodeData(data),
  m_userData(0),
  m_id(0)
{
  reset();
  m_id = graph->generateNewId();
}

//---------------------------------------------------------------------------------------------------------------------
void GraphNode::addConnection(GraphConnection* connection)
{
  m_connections.push_back(connection);
}

//---------------------------------------------------------------------------------------------------------------------
void GraphNode::removeConnection(GraphConnection* connection)
{
  size_t numConns = m_connections.size();
  size_t posConn = numConns;
  for (size_t iConn=0; iConn<numConns; ++iConn)
  {
    if (m_connections.at(iConn) == connection)
    {
      posConn = iConn;
      break;
    }
  }

  if (posConn != numConns)
  {
    if (posConn != numConns-1)
    {
      m_connections.at(posConn) = m_connections.at(numConns-1);
    }

    m_connections.pop_back();
  }
}

//---------------------------------------------------------------------------------------------------------------------
GraphNode* GraphNode::getDestNode(bse::UInt iConn) const
{
  const GraphConnection* const conn = getConnection(iConn);
  BSE_ASSERT(conn);
  BSE_ASSERT(conn->getSource() == this); // this should be always valid...
  return conn->getDest();
}

//---------------------------------------------------------------------------------------------------------------------
bool GraphNode::hasConnectionWithDest(const GraphNode* node) const
{
  for (GraphConnectionsListConst::const_iterator iter=m_connections.begin(); iter != m_connections.end(); ++iter)
  {
    if ((*iter)->getDest() == node)
    {
      return true;
    }
  }

  return false;
}

//---------------------------------------------------------------------------------------------------------------------
// Graph
//---------------------------------------------------------------------------------------------------------------------
Graph::Graph(AIScene* scene, const GraphHeuristic* heuristic) :
  m_heuristic(heuristic),
  m_lastSourceNode(0),
  m_lastDestNode(0),
  m_lastGeneratedId(0),
  m_scene(scene)
{
  BSE_ASSERT(m_heuristic);
}

//---------------------------------------------------------------------------------------------------------------------
Graph::~Graph()
{
  while (!m_connections.empty())
  {
    delete m_connections.back();
    m_connections.pop_back();
  }

  while (!m_nodes.empty())
  {
    delete m_nodes.back();
    m_nodes.pop_back();
  }
}

//---------------------------------------------------------------------------------------------------------------------
GraphNode* Graph::createNode(GraphNodeData* data)
{
  GraphNode* node = new GraphNode(this, data);
  m_nodes.push_back(node);
  return node;
}

//---------------------------------------------------------------------------------------------------------------------
void Graph::removeNode(GraphNode* node)
{
  // TODO: this needs to be implemented
  (void)node;
}

//---------------------------------------------------------------------------------------------------------------------
GraphConnection* Graph::createConnection(GraphNode* source, GraphNode* dest, bse::Real connectionCost)
{
  // create connection
  GraphConnection* connection = new GraphConnection(source, dest, connectionCost);

  // store the connection
  m_connections.push_back(connection);

  return connection;
}

//---------------------------------------------------------------------------------------------------------------------
void Graph::removeConnection(const GraphConnection* connection)
{
  // TODO: use a map is probably the best solution
  // TODO: remove the connection from the nodes
  (void)connection;
}

//---------------------------------------------------------------------------------------------------------------------
bool Graph::computeShortestPath(const GraphNode* start, const GraphNode* end, GraphPath& path, const SearchParameters& searchParams, SearchStats* stats)
{
  bse::GenericTimer searchTimer; // keep a timer instance here, so I can still have multiple instances of the search algorithm
  searchTimer.startTimer();

  AIContext* context = m_scene->getContextManager()->acquireContext();

  // mark if the end node has been reached
  GraphNodeGhost* endGhostNode = 0;

  {

#ifdef BSE_ENABLE_PROFILER
  bse::Profiler* profiler = context->pathFinderContext->getProfiler();
  bse::ProfileTaskWrapper computePath(profiler, "BSE_TOTAL_PATHFINDER_COST");
#endif

  OpenListHeap& openList = context->pathFinderContext->openList;

  BitField*         closedBits = context->pathFinderContext->closedBits;
  OpenListBucket&   openBucket = context->pathFinderContext->openBucket;

  GraphNodesPool& nodesPool = context->pathFinderContext->graphNodesPool;

  bse::UInt visitedNodes = 1; // the start node

  float estCost = 0;
  {
#ifdef BSE_ENABLE_PROFILER
    bse::ProfileTaskWrapper estimCost(profiler, "BSE_ESTIMATE_COST");
#endif
    estCost = m_heuristic->estimatePathCost(start, end);
  }

  GraphNodeGhost* startNodeGhost = new (nodesPool.getObject()) GraphNodeGhost();

  {
#ifdef BSE_ENABLE_PROFILER
    bse::ProfileTaskWrapper openListInsert(profiler, "BSE_OPENLIST_INSERT");
#endif
    openList.insert(startNodeGhost);

    startNodeGhost->node = start;
    startNodeGhost->costSoFar = 0;
    startNodeGhost->costToDest = estCost;
    startNodeGhost->indexOnHeap = 0;  // first node, it goes on the top of the heap.
    startNodeGhost->connToWalk = 0;

    openBucket.insert(startNodeGhost);
  }

  while (!openList.empty())
  {
    if (searchParams.searchType == SearchParameters::SEARCH_TYPE_TIME_LIMITED &&
        searchTimer.getElapsedTime() > searchParams.params.timeLimit)
    {
        break; // time limit elapsed.
    }

    // get the minimum and move it to the closed list
    GraphNodeGhost* minCostNodeGhost;
    {
#ifdef BSE_ENABLE_PROFILER
      bse::ProfileTaskWrapper openListRemove(profiler, "BSE_OPENLIST_REMOVE");
#endif
      minCostNodeGhost = openList.removeBest();
    }

    // remove it from the bucket
    {
#ifdef BSE_ENABLE_PROFILER
      bse::ProfileTaskWrapper openBuckRemove(profiler, "BSE_OPENBUCKET_REMOVE");
#endif
      openBucket.remove(minCostNodeGhost);
    }

    {
#ifdef BSE_ENABLE_PROFILER
      bse::ProfileTaskWrapper closedInsert(profiler, "BSE_CLOSED_INSERT");
#endif
      closedBits->setBit(minCostNodeGhost->node->getID());
    }

    const GraphNode* minCostNode = minCostNodeGhost->node;

    if (minCostNode == end)
    {
      // exit condition, goal node added to the closed list
      endGhostNode = minCostNodeGhost;
      break;
    }

    const GraphConnectionsListConst& connections = minCostNode->getConnections();
    const GraphConnectionsListConst::const_iterator connEnd = connections.end();
    for (GraphConnectionsListConst::const_iterator iter = connections.begin();
         iter != connEnd;
         ++iter)
    {
      const GraphConnection* connection;
      {
#ifdef BSE_ENABLE_PROFILER
        bse::ProfileTaskWrapper getConn(profiler, "BSE_GET_CONNECTION");
#endif
        connection = (*iter);
      }
      const GraphNode* destNode = connection->getDest();

      GraphNodeGhost dummy;
      dummy.node = destNode;
      GraphNodeGhost* ghostDest;
      size_t outIBuck, outIElem;
      bool found = false;
      {
#ifdef BSE_ENABLE_PROFILER
        bse::ProfileTaskWrapper openListSearch(profiler, "BSE_OPENBUCKET_SEARCH");
#endif
        found = openBucket.findElement(&dummy, ghostDest, outIBuck, outIElem);
      }

      if (found)
      {
        // the node is already in the open list, I need to check the cost
        // NB: costToDest should be already ok, because the node is in open list, so already computed
        bse::Real newCost = minCostNodeGhost->costSoFar + connection->getCost();

        if (newCost < ghostDest->costSoFar)
        {
          // the node remains in the list
          // this node is updated directly on the list.
          ghostDest->costSoFar = newCost;
          ghostDest->parentInPath = minCostNodeGhost;
          ghostDest->connToWalk = connection;
          {
#ifdef BSE_ENABLE_PROFILER
            bse::ProfileTaskWrapper openListUpdate(profiler, "BSE_OPENLIST_UPDATE");
#endif
            openList.onPriorityIncreased(ghostDest->indexOnHeap);
          }
        }
      }
      else
      {
        bool foundClosed;
        {
#ifdef BSE_ENABLE_PROFILER
          bse::ProfileTaskWrapper closedSearch(profiler, "BSE_CLOSED_SEARCH");
#endif
          foundClosed = closedBits->hasBit(destNode->getID());
        }

        if (!foundClosed)
        {
          ++visitedNodes;
          // the node is unprocessed
          GraphNodeGhost* newNode = new (nodesPool.getObject()) GraphNodeGhost();
          {
#ifdef BSE_ENABLE_PROFILER
            bse::ProfileTaskWrapper estimCost(profiler, "BSE_ESTIMATE_COST");
#endif
            newNode->costToDest = m_heuristic->estimatePathCost(destNode, end);
          }
          newNode->node = destNode;
          newNode->costSoFar = minCostNodeGhost->costSoFar + connection->getCost();
          newNode->parentInPath = minCostNodeGhost;
          newNode->connToWalk = connection;
          newNode->indexOnHeap = openList.size();

          {
#ifdef BSE_ENABLE_PROFILER
            bse::ProfileTaskWrapper openListInsert(profiler, "BSE_OPENLIST_INSERT");
#endif
            openList.insert(newNode);
            openBucket.insert(newNode);
          }
        }
      }
    }
  }


  // check if end has been reached
  {
    if (stats)
    {
      stats->visitedNodes = visitedNodes;
    }

    GraphNodeGhost* ghostNode = 0;
    float pathCost = 0;

    if (endGhostNode ||
        searchParams.resultType == SearchParameters::RESULT_TYPE_PATH_TO_BEST_NODE_ONFAIL ||
        searchParams.resultType == SearchParameters::RESULT_TYPE_EMPTY_PATH_ONFAIL)
    {
      path.clear(); // prepare the path empty.
    }

    if (endGhostNode)
    {
      ghostNode = endGhostNode;
    }
    else if (searchParams.resultType == SearchParameters::RESULT_TYPE_PATH_TO_BEST_NODE_ONFAIL)
    {
      // the search failed (time limit or node limit elapsed or no solution existed)
      // but the user wants the best solution anyway
      // start the path computation from the most promising node (if one still exists in the queue)
      if (!openList.empty())
      {
        ghostNode = openList.getBest();
      }
    }

    if (ghostNode)
    {
      {
#ifdef BSE_ENABLE_PROFILER
        bse::ProfileTaskWrapper pathConstruction(profiler, "BSE_PATH_CONSTRUCTION");
#endif

        // walk the path... path contains the connections list in the reverse order
        while (ghostNode->parentInPath != 0)
        {
          pathCost += ghostNode->connToWalk->getCost();
          path.push_front(ghostNode->connToWalk);
          ghostNode = ghostNode->parentInPath;
        }
      }

      // output stats
      if (stats)
      {
        stats->success = true;
        stats->pathCost = pathCost;
      }
    }
  }

  }

  // release the search context, so that it can be used by another instance of the path finder.
  m_scene->getContextManager()->releaseContext(context);

  bse::Real totalElapsedTime = searchTimer.stopTimer();
  if (stats)
  {
    stats->searchTime = totalElapsedTime;
  }

  return (endGhostNode != 0);
}

//---------------------------------------------------------------------------------------------------------------------
bool Graph::nodesAreConnected(const GraphNode* node1, const GraphNode* node2) const
{
  // scan node1 connections
  if (node1->hasConnectionWithDest(node2))
  {
    return true;
  }

  // scan node2 connections
  if (node2->hasConnectionWithDest(node1))
  {
    return true;
  }

  return false;
}

}
}
