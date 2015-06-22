#include "VectorClock.h"

#include <iostream>
#include <sstream>

#include "../../include/klee/Internal/Module/KInstruction.h"

namespace klee
{
    VectorClock::VectorClock(uint32_t *vc, uint32_t size)
    {
        for (uint32_t i = 0; i < size; i++)
        {
            _clockMap[i] = vc[i];
        }

 //       _accessInstruction = 0;
  //      _accessThreadID = 0;
        //std::cerr << "created vc" << std::endl;
    }

    VectorClock::VectorClock()
    {

    }

    void VectorClock::import(const VectorClock &other)
    {
        //std::cerr << "called vc->import" << std::endl;
        std::map<uint64_t, uint32_t>::const_iterator importIterator = other._clockMap.begin();
        while (importIterator != other._clockMap.end())
        {
            if (_clockMap[importIterator->first] < importIterator->second)
                _clockMap[importIterator->first] = importIterator->second;

            importIterator++;
        }
    }

    void VectorClock::import(uint32_t *vc, uint32_t size)
    {
        for (uint32_t i = 0; i < size; i++)
        {
            if (_clockMap[i] < vc[i])
                _clockMap[i] = vc[i];
        }


    }

    std::string VectorClock::toString() const
    {
        std::string returnValue;
        std::map<uint64_t, uint32_t>::const_iterator iter = _clockMap.begin();
        while (iter != _clockMap.end())
        {
            std::stringstream ss;
            ss << iter->first;
            returnValue += ss.str();
            returnValue += " ";
            ss.clear();
            ss.str("");
            ss << iter->second;
            returnValue += ss.str();
            returnValue += ", ";
            iter++;
        }

        return returnValue;
    }

    bool VectorClock::happensBefore(const VectorClock &other) const
    {
        bool greaterExists = false;
        bool allLessOrEqual = true;

        std::map<uint64_t, uint32_t>::const_iterator iterA = _clockMap.begin();
        std::map<uint64_t, uint32_t>::const_iterator iterB = other._clockMap.begin();
        while (iterA != _clockMap.end() || iterB != other._clockMap.end())
        {
            if (iterA == _clockMap.end())
            {
                greaterExists = true;
                iterB++;
            }
            else if (iterB == other._clockMap.end())
            {
                allLessOrEqual = false;
                iterA++;
            }
            else
            {
                if (iterA->first == iterB->first)
                {
                    //value included in both vcs

                    if (iterA->second < iterB->second)
                        greaterExists = true;

                    if (iterA->second > iterB->second)
                        allLessOrEqual = false;

                    iterA++;
                    iterB++;
                }
                else if (iterA->first > iterB->first)
                {
                    //vc B got an entry more
                    greaterExists = true;
                    iterB++;
                }
                else
                {
                    allLessOrEqual = false;
                    iterA++;
                }
            }
        }
        return (greaterExists && allLessOrEqual);
    }
/*
    void VectorClock::setOperation(uint32_t threadID, KInstruction *kInst)
    {
        _accessThreadID = threadID;
        _accessInstruction = kInst;
    }
    */
}
