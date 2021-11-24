#ifndef _BSE_ISLAND_INCLUDED
#define _BSE_ISLAND_INCLUDED

#include "bseTypes.h"
#include "bseCollision.h"

#include <vector>

namespace bse
{
namespace phx
{
//---------------------------------------------------------------------------------------------------------------------
enum BodyInfluenceType
{
  BSE_INFLUENCE_PRIMARY,
  BSE_INFLUENCE_SECONDARY
};

//---------------------------------------------------------------------------------------------------------------------
class BodyInfluence
{
public:
  BodyInfluence() :
    other(0), contact(0), influenceType(BSE_INFLUENCE_PRIMARY)
  {
  }

  // TODO refactor this to deal with other types of influences (eg: joints)
  BodyInfluence(Body* b, Contact* c, BodyInfluenceType infType=BSE_INFLUENCE_PRIMARY) :
    other(b), contact(c), influenceType(infType)
  {
  }

  Body* otherBody()
  {
    return other;
  }

  Body* other;
  Contact* contact;
  BodyInfluenceType influenceType;
};

//---------------------------------------------------------------------------------------------------------------------
class BodyInfluencesList : public std::vector<BodyInfluence>
{
public:
  bool insert(const BodyInfluence& influence)
  {
    // TODO improve this, avoiding duplicates
    push_back(influence);
    return true;
  }
};

//---------------------------------------------------------------------------------------------------------------------
class Island
{
public:
  Island(size_t initialSize=16);
  void clear();
  void addBody(Body* body);
  int computeContacts();
  const ContactsList &getContacts() const;
private:
  BodiesList m_bodies;
  ContactsList m_contacts;
};

typedef std::vector<Island*> IslandsList;


//---------------------------------------------------------------------------------------------------------------------
class IslandsManager
{
public:
  IslandsManager(size_t initialNumIslands=4, size_t initialIslandSize=16);
  ~IslandsManager();
  void clear();
  int computeIslands(const BodiesList &bodies);
  const IslandsList& getIslands() const;
protected:
  IslandsList m_islands;
  size_t m_firstAvailable;
};

} // namespace phx
} // namespace bse

#endif // _BSE_ISLAND_INCLUDED
