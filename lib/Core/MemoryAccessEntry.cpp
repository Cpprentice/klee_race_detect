#include "MemoryAccessEntry.h"

#include "../../include/klee/Internal/Module/InstructionInfoTable.h"

#include "llvm/IR/Instructions.h"

namespace klee
{

    bool MemoryAccessEntry::operator<(const MemoryAccessEntry &other) const
    {
      //  bool ethread = _threadID == other._threadID;
        //if (ethread)
          //  return false;

        bool file = _kInst->info->file < other._kInst->info->file;
        bool opcode = _kInst->inst->getOpcode() < other._kInst->inst->getOpcode();
        bool line = _kInst->info->line < other._kInst->info->line;

        bool efile = _kInst->info->file == other._kInst->info->file;
        bool eopcode = _kInst->inst->getOpcode() == other._kInst->inst->getOpcode();
        bool eline = _kInst->info->line == other._kInst->info->line;

        if (opcode) return true;
        if (eopcode && file) return true;
        if (eopcode && efile && line) return true;
        return false;
    }

    MemoryAccessEntry::MemoryAccessEntry(KInstruction *kInst)
    {
        _kInst = kInst;
    }
}
