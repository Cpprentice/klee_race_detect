#ifndef MEMORY_ACCESS_ENTRY_H
#define MEMORY_ACCESS_ENTRY_H

#include "../../include/klee/Internal/Module/KInstruction.h"

namespace klee
{
    class MemoryAccessEntry
    {
    public:
        MemoryAccessEntry(KInstruction *kInst);
        bool operator<(const MemoryAccessEntry &other) const;
    //private:
        KInstruction *_kInst;
        //uint32_t _threadID;
    };
}

#endif // MEMORY_ACCESS_ENTRY_H
