#ifndef _BSE_PHYSICSTYPES_H_INCLUDED
#define _BSE_PHYSICSTYPES_H_INCLUDED

#include <vector>

namespace bse
{
namespace phx
{

class Contact;
class Shape;
class ShapeDesc;
class Body;
class Ray;
class Material;

template<typename T>
class Vector : public std::vector<T>
{
public:
  bool push_backUnique(const T& elem)
  {
    auto iter = std::find(this->begin(), this->end(), elem);
    if (iter != this->end())
    {
      return false;      
    }

    push_back(elem);
    return true;
  }
};

typedef Vector<Shape*>       ShapesList;
typedef std::vector<ShapeDesc*>   ShapeDescsList;
typedef std::vector<Contact*>     ContactsList;
typedef std::vector<Body*>        BodiesList;
typedef std::vector<Ray*>         RaysList;
typedef std::vector<Material*> MaterialsList;

//---------------------------------------------------------------------------------------------------------------------
// the user should derive this class to implement it's own contact feedback
class ContactFeedback
{
public:
  virtual void onContactReport(const Contact* contact)=0;
};


//---------------------------------------------------------------------------------------------------------------------
// the user should derive this class to implement it's own contact filtering
class ContactFilter
{
public:
  virtual bool applyFilterOnContact(Contact* contact)=0;
};

}
}

#endif // _BSE_PHYSICSTYPES_H_INCLUDED
