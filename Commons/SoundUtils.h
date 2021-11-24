#ifndef _SOUNDUTILS_H_INCLUDED
#define _SOUNDUTILS_H_INCLUDED

#include <map>

typedef unsigned int SampleID;
#define INVALID_SAMPLE_ID 0xffffff


class SoundManager
{
private:
  class SoundSample
  {
  public:
    SoundSample(SoundManager* manager);
    ~SoundSample();
    void play();
    void stop();
  public:
    SampleID sample;
  private:
    SoundManager* m_manager;
  };

public:
  SoundManager();
  ~SoundManager();
  SampleID loadSoundSample(const char* filename);
  bool unloadSoundSample(SampleID sample);
  bool playSoundSample(SampleID sample);

private:
  typedef std::map<SampleID, SoundSample*> SampleIDToSoundSampleMap;
  SampleIDToSoundSampleMap m_samplesMap;
  SampleID m_nextID;  // next ID to be assigned
};

#endif // _SOUNDUTILS_H_INCLUDED
