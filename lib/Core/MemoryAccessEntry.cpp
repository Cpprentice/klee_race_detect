#include "MemoryAccessEntry.h"

#include "../../include/klee/Internal/Module/InstructionInfoTable.h"

#include "llvm/IR/Instructions.h"

#include "../../include/klee/ExecutionState.h"

namespace klee
{

    bool MemoryAccessEntry::operator<(const MemoryAccessEntry &other) const
    {
        bool equalThread = _thread == other._thread;
        bool equalVar = _varName == other._varName;
        bool equalLocation = _location == other._location;
        bool equalWrite = _write == other._write;
        bool equalScheduling = _schedulingHistory == other._schedulingHistory;

        if (_thread < other._thread)
            return true;
        if (equalThread && _varName < other._varName)
            return true;
        if (equalThread && equalVar && _location < other._location)
            return true;
        if (equalThread && equalVar && equalLocation && _write < other._write)
            return true;
        if (equalThread && equalVar && equalLocation && equalWrite && _schedulingHistory < other._schedulingHistory)
            return true;
        if (equalThread && equalVar && equalLocation && equalWrite && equalScheduling && _vc < other._vc)
            return true;
        return false;
    }

    MemoryAccessEntry::MemoryAccessEntry(Thread::thread_id_t thread, VectorClock vc, std::string varName, std::string location, bool write, const ExecutionState *state)
    {
        _thread = thread;
        _vc = vc;
        _varName = varName;
        _location = location;
        _write = write;
        _schedulingHistory = state->schedulingHistory;
    }

    bool MemoryAccessEntry::isRace(const MemoryAccessEntry &other) const
    {
        if (_thread == other._thread)
            return false;
        if (_write == false && other._write == false)
            return false;
        if (_varName != other._varName)
            return false;
        //if (!isSameScheduling(other))
         //   return false;
        if (!_vc.happensBefore(other._vc) && !other._vc.happensBefore(_vc))
            return true;
        return false;
    }

    bool MemoryAccessEntry::isRuntime() const
    {
        return _location.find("POSIX") != std::string::npos || _location.find("Intrinsic") != std::string::npos;
    }

    bool MemoryAccessEntry::isSameScheduling(const MemoryAccessEntry &other) const
    {
        size_t size = _schedulingHistory.size();
        size_t oldSize = other._schedulingHistory.size();
        size_t minSize = oldSize;
        if (size < oldSize)
            minSize = size;
        for (size_t i = 0; i < minSize; i++)
        {
            if (_schedulingHistory[i] != other._schedulingHistory[i])
                return false;
        }
        return true;
    }
}
