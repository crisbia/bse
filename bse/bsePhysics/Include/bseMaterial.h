#ifndef _BSE_MATERIAL_INCLUDED
#define _BSE_MATERIAL_INCLUDED

#include "bseMath.h"

namespace bse
{
namespace phx
{

//---------------------------------------------------------------------------------------------------------------------
typedef enum FrictionMode_enum
{
  BSE_FRICTIONMODE_MULTSQRT,
  BSE_FRICTIONMODE_MULT,
  BSE_FRICTIONMODE_ADD,
  NUM_BSE_FRICTIONMODE_TYPES
} FrictionMode;

//---------------------------------------------------------------------------------------------------------------------
typedef enum RestitutionMode_enum
{
  BSE_RESTITUTIONMODE_MULTSQRT,
  BSE_RESTITUTIONMODE_MULT,
  BSE_RESTITUTIONMODE_ADD,
  NUM_RESTITUTIONMODE_TYPES
} RestitutionMode;

//---------------------------------------------------------------------------------------------------------------------
class MaterialDesc
{
public:
  MaterialDesc() :
      friction(0),
      frictionMode(BSE_FRICTIONMODE_MULT),
      restitution(0),
      restitutionMode(BSE_RESTITUTIONMODE_MULT)
  {
  }
public:
  bse::Real friction;
  FrictionMode frictionMode;
  bse::Real restitution;
  RestitutionMode restitutionMode;
};

//---------------------------------------------------------------------------------------------------------------------
class Material
{
  friend class Scene;
public:
  bse::Real getFriction() const { return m_materialDesc.friction; }
  bse::Real getRestitution() const { return m_materialDesc.restitution; }
  void setFriction(bse::Real friction) { m_materialDesc.friction = friction; }
  void setRestitution(bse::Real restitution) { m_materialDesc.restitution = restitution; }

  FrictionMode getFrictionMode() const { return m_materialDesc.frictionMode; }
  RestitutionMode getRestitutionMode() const { return m_materialDesc.restitutionMode; }
  void setFrictionMode(FrictionMode frictionMode) { m_materialDesc.frictionMode = frictionMode; }
  void setRestitutionMode(RestitutionMode restitutionMode) { m_materialDesc.restitutionMode = restitutionMode; }

  static bse::Real combineFriction(const Material* material1, const Material* material2);
  static bse::Real combineRestitution(const Material* material1, const Material* material2);

protected:
  Material(MaterialDesc* materialDesc) : m_materialDesc(*materialDesc)
  {
  }

  virtual ~Material()
  {
  }

  MaterialDesc m_materialDesc;
};

}
}

#endif // _BSE_MATERIAL_INCLUDED
