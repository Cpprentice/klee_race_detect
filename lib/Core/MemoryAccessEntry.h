#ifndef MEMORY_ACCESS_ENTRY_H
#define MEMORY_ACCESS_ENTRY_H

#include "../../include/klee/Internal/Module/KInstruction.h"
#include "Threading.h"

namespace klee
{
    //class RaceReport;

    class MemoryAccessEntry
    {
        friend class RaceReport;
    public:
        MemoryAccessEntry(Thread::thread_id_t thread, VectorClock vc, std::string varName, std::string location, bool write);
        bool operator<(const MemoryAccessEntry &other) const;
        bool isRace(const MemoryAccessEntry &other) const;
        bool isRuntime() const;
    private:
        Thread::thread_id_t _thread;
        VectorClock _vc;
        std::string _varName;
        std::string _location;
        bool _write;
    };
}

#endif // MEMORY_ACCESS_ENTRY_H
