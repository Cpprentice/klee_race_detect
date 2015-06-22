#ifndef RACE_REPORT_H
#define RACE_REPORT_H

#include "MemoryAccessEntry.h"
#include "VectorClock.h"

#include <string>
#include <inttypes.h>
#include <set>

namespace klee
{
    struct KInstruction;
    class ObjectState;

    class RaceReport
    {
    public:
        RaceReport(uint32_t thread1, uint32_t thread2, std::string file, int line);
        RaceReport(uint32_t thread1, uint32_t thread2, KInstruction *kInst, ObjectState *os);

        RaceReport(uint32_t currentThread, KInstruction *currInst, ObjectState *currState, const MemoryAccessEntry &lastAccess, uint32_t lastThread);
        RaceReport(uint32_t currentThread, KInstruction *currInst, ObjectState *currState, const MemoryAccessEntry &lastAccess, uint32_t lastThread, const VectorClock &curr, const VectorClock &last);

        std::string toString() const;

        bool operator<(const RaceReport& rr) const;


        static std::set<RaceReport> overallReports;


    private:
        uint32_t _thread1;
        uint32_t _thread2;
        std::string _file;
        int _line;


        KInstruction *_kInst;
        ObjectState *_os;
        MemoryAccessEntry _last;
        uint32_t _currThread;
        uint32_t _lastThread;

        VectorClock _currVC;
        VectorClock _lastVC;
    };


}

#endif // RACE_REPORT_H
