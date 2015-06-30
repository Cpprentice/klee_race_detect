
#ifndef VECTORCLOCK_H
#define VECTORCLOCK_H

#include <inttypes.h>
#include <map>
#include <string>

#include "../../include/klee/Internal/Module/KInstruction.h"

namespace klee
{
    class VectorClock
    {
    public:

//        static std::map<uint64_t, VectorClock> globalVectorClocks;
        static uint64_t createVectorClock(std::map<uint64_t, VectorClock> &container);

        typedef std::map<uint64_t, uint32_t>::iterator clock_iterator_t;

        VectorClock(uint32_t *vc, uint32_t size);
        VectorClock();

        void import(uint32_t *vc, uint32_t size);
        void import(const VectorClock &other);

        bool happensBefore(const VectorClock &other) const;
        bool operator<(const VectorClock &other) const;

        std::string toString() const;

        void clear();

        uint32_t& operator[](uint64_t index);

    private:
        std::map<uint64_t, uint32_t> _clockMap;
    };

}

#endif
