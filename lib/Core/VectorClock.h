
#ifndef VECTORCLOCK_H
#define VECTORCLOCK_H

#include <inttypes.h>
#include <map>
#include <string>

#include "../../include/klee/Internal/Module/KInstruction.h"

namespace klee
{
    //class KInstruction;

    class VectorClock
    {
    public:
        VectorClock(uint32_t *vc, uint32_t size);
        VectorClock();

        void import(uint32_t *vc, uint32_t size);
        void import(const VectorClock &other);

        bool happensBefore(const VectorClock &other) const;

        std::string toString() const;

        void setOperation(uint32_t threadID, KInstruction *kInst);
    private:
        std::map<uint64_t, uint32_t> _clockMap;
       // uint32_t _accessThreadID;
        //KInstruction *_accessInstruction;
    };
}

#endif
