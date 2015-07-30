#include "VectorClock.h"

#include <iostream>
#include <sstream>

#include "../../include/klee/Internal/Module/KInstruction.h"

#include "Common.h"

namespace klee
{
    //std::map<uint64_t, VectorClock> VectorClock::globalVectorClocks;

    uint64_t VectorClock::createVectorClock(std::map<uint64_t, VectorClock> &container)
    {
        //static uint64_t id = 1;
        //container[id];
        //klee_message("creating vc. container size: %lu", container.size());
        //if (container.size() != 1)
            container[container.size() + 1];
        //globalVectorClocks[id];
        //klee::klee_message("created VectorClock %llu", globalVectorClocks.size());
        //return id++;
        return container.size();
    }



    VectorClock::VectorClock(uint32_t *vc, uint32_t size)
    {
        for (uint32_t i = 0; i < size; i++)
        {
            if (vc[i] > 0)
                _clockMap[i] = vc[i];
        }
    }

    VectorClock::VectorClock()
    {

    }

    void VectorClock::import(const VectorClock &other)
    {
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
            if (vc[i] > 0)
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

    bool VectorClock::operator<(const VectorClock &other) const
    {
        std::map<uint64_t, uint32_t>::const_iterator iterA = _clockMap.begin();
        std::map<uint64_t, uint32_t>::const_iterator iterB = other._clockMap.begin();
        while (iterA != _clockMap.end() || iterB != other._clockMap.end())
        {
            if (iterA == _clockMap.end())
            {
                return true;
            }
            else if (iterB == other._clockMap.end())
            {
                return false;
            }
            else
            {
                if (iterA->first == iterB->first && iterA->second == iterB->second)
                {
                    iterA++;
                    iterB++;
                }
                else if (iterA->first < iterB->first)
                    return true;
                else if (iterA->first == iterB->first && iterA->second < iterB->second)
                    return true;
                else
                    return false;
            }
        }
        return false;
    }

    void VectorClock::clear()
    {
        _clockMap.clear();
    }

    uint32_t& VectorClock::operator[](uint64_t index)
    {
        return _clockMap[index];
    }
}
