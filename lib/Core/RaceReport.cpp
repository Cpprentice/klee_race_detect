#include "RaceReport.h"

#include <sstream>
//#include <llvm.h>
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
/*
    RaceReport::RaceReport(uint32_t thread1, uint32_t thread2, std::string file, int line) : _last(0)
    {
        _line = line;
        _file = file;
        _thread1 = thread1;
        _thread2 = thread2;
        _kInst = 0;
        _os = 0;
    }
*/
 //   RaceReport::RaceReport(uint32_t thread1, uint32_t thread2, KInstruction *kInst, ObjectState *os) : _last(0)
  //  {
  /*      int *operands = kInst->operands;
        unsigned dest = kInst->dest;

        unsigned assemblyLine = kInst->info->assemblyLine;
        unsigned id = kInst->info->id;

        llvm::Instruction *inst = kInst->inst;

        std::string allocInfo;
        os->getObject()->getAllocInfo(allocInfo);
        const llvm::Value *value = os->getObject()->allocSite;

        std::string valueName = value->getName().str();
*/
  /*      _line = kInst->info->line;
        _file = kInst->info->file;
        _thread1 = thread1;
        _thread2 = thread2;
        _kInst = kInst;
        _os = os;
*/
        /*llvm::errs()    << "RaceReport: ________________\n"
                        << "Thread IDs: " << thread1 << " " << thread2 << "\n"
                        << "Source: " << _file << ":" << _line << "\n"
                        << "Assembly: " << inst->getOpcodeName() << " Line: " << assemblyLine << "\n"
                        << "allocSiteName: " << valueName << "\n"
                        << "allocInfo: " << allocInfo << "\n"
                        << "dest: " << dest << "\n"
                        << "address: " << (void*)os->getObject()->address << "\n"
                        << "_____________________________\n";

*/
   // }

/*    RaceReport::RaceReport(uint32_t currentThread, KInstruction *currInst, ObjectState *currState, const MemoryAccessEntry &lastAccess, uint32_t lastThread) : _last(lastAccess)
    {
        _currThread = currentThread;
        _kInst = currInst;
        _os = currState;
        //_last = lastAccess;
        _lastThread = lastThread;
    }

    RaceReport::RaceReport(uint32_t currentThread, KInstruction *currInst, ObjectState *currState, const MemoryAccessEntry &lastAccess, uint32_t lastThread, const VectorClock &curr, const VectorClock &last) : _last(lastAccess)
    {
        _currThread = currentThread;
        _kInst = currInst;
        _os = currState;
        //_last = lastAccess;
        _lastThread = lastThread;
        _currVC.import(last);
        _lastVC.import(curr);
    }
*/
    std::string RaceReport::toString() const
    {
 /*       int *operands = _kInst->operands;
        unsigned dest = _kInst->dest;

        unsigned assemblyLine = _kInst->info->assemblyLine;
        unsigned id = _kInst->info->id;

        llvm::Instruction *inst = _kInst->inst;

        std::string allocInfo;
        _os->getObject()->getAllocInfo(allocInfo);
        const llvm::Value *value = _os->getObject()->allocSite;

        std::string valueName = value->getName().str();
   */      std::stringstream ss;
       //ss << "Race between thread" << _thread1 << " and thread" << _thread2 << " at " << _file << ":" << _line;
 /*       ss              << "\nRaceReport: ________________\n"
                        << "Thread IDs: " << _thread1 << " " << _thread2 << "\n"
                        << "Source: " << _file << ":" << _line << "\n"
                        << "Assembly: " << inst->getOpcodeName() << " Line: " << assemblyLine << "\n"
                        << "allocSiteName: " << valueName << "\n"
                        << "allocInfo: " << allocInfo << "\n"
                        << "dest: " << dest << "\n"
                        << "address: " << (void*)_os->getObject()->address << "\n"
                        << "_____________________________\n";*/

   /*     ss  << "\nRaceReport: ________________________\n"
            << "Thread IDs: " << _currThread << " " << _lastThread << "\n"
            << "Variable: " << _os->getObject()->allocSite->getName().str() << "\n"
            << "Location 1: " << _kInst->info->file << ":" << _kInst->info->line << " " << _kInst->inst->getOpcodeName() << "\n"
            << "Location 2: " << _last._kInst->info->file << ":" << _last._kInst->info->line << " " << _last._kInst->inst->getOpcodeName() << "\n"
            << "VC 1: " << _currVC.toString() << "\n"
            << "VC 2: " << _lastVC.toString() << "\n"
            << "________________________________________\n";*/
        ss  << "\nRaceReport: ________________________\n"
            << "Thread IDs: " << _current._thread << " " << _last._thread << "\n"
            << "Variable: " << _current._varName << "\n"
            << "Location 1: " << _current._location << " " << (_current._write?"store":"load") << "\n"
            << "Location 2: " << _last._location << " " << (_last._write?"store":"load") << "\n"
            << "VC 1: " << _current._vc.toString() << "\n"
            << "VC 2: " << _last._vc.toString() << "\n"
            << "________________________________________\n";
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

 /*       bool inverse = _currThread == rr._lastThread && _lastThread == rr._currThread;
        inverse = inverse && _kInst->info->file == rr._last._kInst->info->file && _last._kInst->info->file == rr._kInst->info->file;
        inverse = inverse && _kInst->info->line == rr._last._kInst->info->line && _last._kInst->info->line == rr._kInst->info->line;
        if (inverse)
            return false;

        if (_currThread < rr._currThread)
            return true;
        if (_currThread == rr._currThread && _lastThread < rr._lastThread)
            return true;
        if (_currThread == rr._currThread && _lastThread == rr._lastThread && _kInst->info->file < rr._kInst->info->file)
            return true;
        if (_currThread == rr._currThread && _lastThread == rr._lastThread && _kInst->info->file == rr._kInst->info->file && _kInst->info->line < rr._kInst->info->line)
            return true;
        return false;
*/

 /*       bool len = _file.size() < rr._file.size();
        bool line = _line < rr._line;
        bool t1 = _thread1 < rr._thread1;
        bool t2 = _thread2 < rr._thread2;


        bool elen = _file.size() == rr._file.size();
        bool eline = _line == rr._line;
        bool et1 = _thread1 == rr._thread1;
        bool et2 = _thread2 == rr._thread2;

        bool inverse = _thread1 == rr._thread2 && _thread2 == rr._thread1;

        if (len)
            return true;
        if (elen && line)
            return true;
        if (elen && eline && inverse)
            return false;
        if (elen && eline && t1)
            return true;
        if (elen && eline && et1 && t2)
            return true;
        return false;*/



    }
}
