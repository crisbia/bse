#ifndef _PARAMSTWEAKER_H_INCLUDED
#define _PARAMSTWEAKER_H_INCLUDED

namespace BSEDemo
{
/*
 * Generic helper class to create
 * a params tweaker
 * The idea is to provide a flexible
 * way to setup and edit params
 * load/save from/to xml
 * maybe cook data for easy rendering or log
 *
 *
 *
 *
 *
 *
 */

// the client app should derive from this class
// and maybe declare an enum type like the following
//typedef enum {
//  INVALID_AAA_PARAM = -1,
//  AAA_PARAM_1,
//  AAA_PARAM_2,
//  .....,
//  NUM_AAA_PARAMS
//} AAAParamName;
//

// defines the possible types for a param
typedef enum {
  PARAM_BOOL,
  PARAM_FLOAT,
  PARAM_INT,
  PARAM_VEC3,
  NUM_PARAM_TYPES
} ParamType;

typedef int ParamName;

struct Param
{
public:
  typedef union
  {
    struct
    {
       float x, y, z;
    } v;
    float f;
    int i;
    bool b;
  } ParamValue;
public:
  ParamName name;
  ParamType type;
  ParamValue value;
  ParamValue defaultValue;
  bool show; // used to hide/show
  char visualize[64];
};

#define INIT_PARAM_BOOL(x) {{x}}
#define INIT_PARAM_VEC3(x,y,z) {{x,y,z}}

class ParamDescription
{
public:
  ParamName name;
  const char* description;
};

class ParamsTweaker
{
public:
  ParamsTweaker() : m_params(0), m_numParams(0) { }

  bool getParamValue(const ParamName name) const;
  bool retrieveParamValue(const ParamName name, float& param) const;
  bool retrieveParamValue(const ParamName name, bool& param) const;
  bool retrieveParamValue(const ParamName name, int& param) const;
  Param retrieveParam(const ParamName name) const;

public:
  const int getNumParams() const { return m_numParams; }
protected:
  void setupParamsData(const Param* params, const int numParams) { m_params = params; m_numParams = numParams; }
  const Param* m_params;
  int m_numParams;
};

}

#endif // _PARAMSTWEAKER_H_INCLUDED
