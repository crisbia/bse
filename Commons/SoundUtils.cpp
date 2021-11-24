#include "SoundUtils.h"

#ifdef WIN32
#include <windows.h>
#endif

// TODO: this is just the initial skeleton for a nice wrap around the sound system library.

SoundManager::SoundSample::SoundSample(SoundManager* manager) : m_manager(manager)
{

}

SoundManager::SoundSample::~SoundSample()
{
}

void SoundManager::SoundSample::play()
{
}

void SoundManager::SoundSample::stop()
{

}

SoundManager::SoundManager()
{

}

SoundManager::~SoundManager()
{

}

SampleID SoundManager::loadSoundSample(const char* filename)
{

  return INVALID_SAMPLE_ID;
}

bool SoundManager::unloadSoundSample(SampleID sample)
{
  return true;
}

bool SoundManager::playSoundSample(SampleID sample)
{
  return true;
}

