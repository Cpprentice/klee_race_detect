#include "RaceReport.h"

#include <sstream>
#include "../../include/klee/Internal/Module/KInstruction.h"
#include "../../include/klee/Internal/Module/InstructionInfoTable.h"
#include "Memory.h"

#include "llvm/Support/raw_ostream.h"

#include "llvm/IR/Instructions.h"

namespace klee
{
    std::set<RaceReport> RaceReport::overallReports;

    RaceReport::RaceReport(const MemoryAccessEntry &last, const MemoryAccessEntry &current) : _last(last), _current(current)
    {
    }
    std::string RaceReport::toString() const
    {
        std::stringstream ss;
        ss  << "\nRaceReport: ________________________\n"
            << "Thread IDs: " << _current._thread << " " << _last._thread << "\n"
            << "Variable: " << _current._varName << "\n"
            << "Location 1: " << _current._location << " " << (_current._write?"store":"load") << "\n"
            << "Location 2: " << _last._location << " " << (_last._write?"store":"load") << "\n"
            << "VC 1: " << _current._vc.toString() << "\n"
            << "VC 2: " << _last._vc.toString() << "\n"
            << "Scheduling 1: ";
        for (size_t i = 0; i < _current._schedulingHistory.size(); i++)
            ss << _current._schedulingHistory[i] << ",";
        ss  << "\nScheduling 2: ";
        for (size_t i = 0; i < _last._schedulingHistory.size(); i++)
            ss << _last._schedulingHistory[i] << ",";
        ss  << "\n________________________________________\n";
        return ss.str();
    }

    bool RaceReport::operator<(const RaceReport &rr) const
    {
        bool equalCurrentThread = _current._thread == rr._current._thread;
        bool equalLastThread = _last._thread == rr._last._thread;
        bool equalCurrentLocation = _current._location == rr._current._location;
        bool equalLastLocation = _last._location == rr._last._location;

        if (_current._thread < rr._current._thread)
            return true;
        if (equalCurrentThread && _last._thread < rr._last._thread)
            return true;
        if (equalCurrentThread && equalLastThread && _current._location < rr._current._location)
            return true;
        if (equalCurrentThread && equalLastThread && equalCurrentLocation && _last._location < rr._last._location)
            return true;

        return false;
    }
}
