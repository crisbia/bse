
#include "ParamsTweaker.h"

#include <assert.h>

namespace BSEDemo
{

//
// the client app should declare a params structure like the following:
// nb: the first element of every line is redundant, but helps readabiity
/*
Param gAAAParams[NUM_AAA_PARAMS] = 
{
  { AAA_PARAM1, BSEPARAM_BOOL, false, false, "Disable Simulation", true},
  { AAA_PARAM1, BSEPARAM_BOOL, false, false, "Disable Simulation", true},
  .....
}
*/



bool ParamsTweaker::getParamValue(const ParamName name) const
{
  assert(m_params);
  assert(name >= 0 && name < getNumParams());
 
  assert( m_params[name].type == PARAM_BOOL );

  return m_params[name].value.b;
}

bool ParamsTweaker::retrieveParamValue(const ParamName name, float& param) const
{
  assert(m_params);
  assert(name >= 0 && name < getNumParams());
  
  assert( m_params[name].type == PARAM_FLOAT );

  param = m_params[name].value.f;
  return true;
}

bool ParamsTweaker::retrieveParamValue(const ParamName name, bool& param) const
{
  assert(m_params);
  assert(name >= 0 && name < getNumParams());

  assert( m_params[name].type == PARAM_BOOL );

  param = m_params[name].value.b;
  return true;
}

bool ParamsTweaker::retrieveParamValue(const ParamName name, int& param) const
{
  assert(m_params);
  assert(name >= 0 && name < getNumParams());

  assert( m_params[name].type == PARAM_INT );

  param = m_params[name].value.i;
  return true;
}


Param ParamsTweaker::retrieveParam(const ParamName name) const
{
  assert(m_params);
  assert(name >= 0 && name < getNumParams());

  return m_params[name];
}

}