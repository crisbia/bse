#include "bseIsland.h"
#include "bseBody.h"

/*
Islands generation scheme:

for each body do
  // skip non dynamic bodies
  if !body.isDynamic continue 

  connectionFound = false
  if body.islandID == -1 then
    for each connection in body.connections do

      // skip connection to non dynamic bodies
      if !connection.otherBody.isDynamic then continue

      if connection.otherBody.islandID != -1 then
        islands[conn.otherBody.islandID].add(body)
        connectionFound = true
        break
      end
    end
  end

  // no island found, we need to create a new one
  if !connectionFound then
    islands.push_back(new Island())
    islands.back().add(body)
  end

end
*/

namespace bse
{
namespace phx
{
// TODO reuse islands in the islands generator, because
// they can contain hundreds of contacts and bodies, so the allocation and deallocation
// is going to be quite expensive.
// I just need to keep them until the generator is destroyed.
// TODO change the name of the generator in "manager" :)

//---------------------------------------------------------------------------------------------------------------------
Island::Island(size_t initialSize)
{
  m_bodies.reserve(initialSize);
  m_contacts.reserve(2*initialSize);
}

//---------------------------------------------------------------------------------------------------------------------
void Island::clear()
{
  m_bodies.clear();
  m_contacts.clear();
}

//---------------------------------------------------------------------------------------------------------------------
void Island::addBody(Body* body)
{
  if (body!=0 && body->getCurrentIsland()!=this)
  {
    m_bodies.push_back(body);
    body->setCurrentIsland(this);
  }
}

//---------------------------------------------------------------------------------------------------------------------
int Island::computeContacts()
{
  m_contacts.clear();

  for (size_t iBody=0; iBody<m_bodies.size(); ++iBody)
  {
    Body* body = m_bodies[iBody];
    const BodyInfluencesList influences = body->getBodyInfluences();
    for (BodyInfluencesList::const_iterator influence=influences.begin(); influence != influences.end(); ++influence)
    {
      const BodyInfluence& current = (*influence);
      if (current.influenceType == BSE_INFLUENCE_PRIMARY || current.other->isStatic())
      {
        m_contacts.push_back((*influence).contact);
      }
    }
  }

  return (int)m_contacts.size();
}

//---------------------------------------------------------------------------------------------------------------------
const ContactsList& Island::getContacts() const
{
  return m_contacts;
}

//---------------------------------------------------------------------------------------------------------------------
IslandsManager::IslandsManager(size_t initialNumIslands, size_t initialIslandSize) :
  m_firstAvailable(0)
{
  if (initialNumIslands == 0)
    initialNumIslands = 1;
  if (initialIslandSize == 0)
    initialIslandSize = 1;

  m_islands.resize(initialNumIslands);
  for (size_t i=0; i<m_islands.size(); ++i)
  {
    m_islands[i] = new Island(initialIslandSize);
  }
}

//---------------------------------------------------------------------------------------------------------------------
IslandsManager::~IslandsManager()
{
  clear();
  for (size_t i=0; i<m_islands.size(); ++i)
  {
    delete m_islands[i];
  }
}

//---------------------------------------------------------------------------------------------------------------------
void IslandsManager::clear()
{
  for (size_t i=0; i<m_islands.size(); ++i)
  {
    m_islands[i]->clear();
  }

  m_firstAvailable = 0;
}

//---------------------------------------------------------------------------------------------------------------------
const IslandsList& IslandsManager::getIslands() const
{
  return m_islands;
}

//---------------------------------------------------------------------------------------------------------------------
int IslandsManager::computeIslands(const BodiesList& bodies)
{
  clear();
  for (BodiesList::const_iterator iter=bodies.begin(); iter != bodies.end(); ++iter)
  {
    Body* body = (*iter);
    // skip non dynamic bodies
    // TODO consider kinematic ones...
    if (body->isStatic()) continue; 

    if (body->getCurrentIsland()==0)
    {
      bool influenceFound = false;
      const BodyInfluencesList influences = body->getBodyInfluences();
      for (BodyInfluencesList::const_iterator influence=influences.begin(); influence != influences.end(); ++influence)
      {
        Body* other = (*influence).other;
        if (other)
        {
          if (other->isStatic()) 
          {
            continue;
          }
          
          if (other->getCurrentIsland()!=0)
          {
            other->getCurrentIsland()->addBody(body);
            influenceFound = true;
            break;
          }
        }
      }

      // no island found, we need to create a new one
      if (!influenceFound)
      {
        if (m_firstAvailable == m_islands.size())
          m_islands.push_back(new Island());
        Island* isl = m_islands[m_firstAvailable];
        ++m_firstAvailable;
        isl->addBody(body);

        // fix immediately all the connections which are not already in an island
        const BodyInfluencesList influences = body->getBodyInfluences();
        for (BodyInfluencesList::const_iterator influence=influences.begin(); influence != influences.end(); ++influence)
        {
          Body* other = (*influence).other;
          if (other && !other->isStatic() && other->getCurrentIsland()==0)
          {
            isl->addBody(other);
          }
        }
      }
    }
  }

  for (size_t i = 0; i < m_islands.size(); ++i)
  {
    Island* island = m_islands[i];
    island->computeContacts();
  }

  return (int)m_islands.size();
}

} // namespace phx
} // namespace bse