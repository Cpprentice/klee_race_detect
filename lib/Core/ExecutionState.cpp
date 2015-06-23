//===-- ExecutionState.cpp ------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/ExecutionState.h"

#include "klee/Internal/Module/Cell.h"
#include "klee/Internal/Module/InstructionInfoTable.h"
#include "klee/Internal/Module/KInstruction.h"
#include "klee/Internal/Module/KModule.h"

#include "klee/Expr.h"

#include "Memory.h"
#if LLVM_VERSION_CODE >= LLVM_VERSION(3, 3)
#include "llvm/IR/Function.h"
#else
#include "llvm/Function.h"
#endif
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/raw_ostream.h"

#include <iomanip>
#include <sstream>
#include <cassert>
#include <map>
#include <set>
#include <stdarg.h>

using namespace llvm;
using namespace klee;

namespace {
  cl::opt<bool>
  DebugLogStateMerge("debug-log-state-merge");
}

ExecutionState::ExecutionState(KFunction *kf)
  : fakeState(false),
    depth(0),
    queryCost(0.),
    weight(1),
    instsSinceCovNew(0),
    coveredNew(false),
    forkDisabled(false),
    ptreeNode(0),
    wlistCounter(1),
    preemptions(0) {
        setupMain(kf);
}

ExecutionState::ExecutionState(const std::vector<ref<Expr> > &assumptions)
  : fakeState(true),
    constraints(assumptions),
    queryCost(0.),
    ptreeNode(0),
    wlistCounter(1),
    preemptions(0) {
   setupMain(NULL);
}

void ExecutionState::setupMain(KFunction *kf) {
    Thread mainThread = Thread(0, kf);
    threads.insert(std::make_pair(mainThread.tid, mainThread));
    crtThreadIt = threads.begin();

    ///MODIFICATION
    uint32_t initVC = 1;
    VectorClock vc(&initVC, 1);
    ///MODIFCATION END
}

ExecutionState::~ExecutionState() {
  for (unsigned int i=0; i<symbolics.size(); i++)
  {
    const MemoryObject *mo = symbolics[i].first;
    assert(mo->refCount > 0);
    mo->refCount--;
    if (mo->refCount == 0)
      delete mo;
  }

  for (threads_ty::iterator it = threads.begin(); it != threads.end(); it++) {
      Thread &t = it->second;
      while (!t.stack.empty())
          popFrame(t);
  }

}

ExecutionState::ExecutionState(const ExecutionState& state)
  : fnAliases(state.fnAliases),
    fakeState(state.fakeState),
    depth(state.depth),
    constraints(state.constraints),
    queryCost(state.queryCost),
    weight(state.weight),
    addressSpace(state.addressSpace),
    pathOS(state.pathOS),
    symPathOS(state.symPathOS),
    instsSinceCovNew(state.instsSinceCovNew),
    coveredNew(state.coveredNew),
    forkDisabled(state.forkDisabled),
    coveredLines(state.coveredLines),
    ptreeNode(state.ptreeNode),
    symbolics(state.symbolics),
    arrayNames(state.arrayNames),
    shadowObjects(state.shadowObjects),
    threads(state.threads),
    waitingLists(state.waitingLists),
    wlistCounter(state.wlistCounter),
    preemptions(state.preemptions),
    schedulingHistory(state.schedulingHistory)
{
  for (unsigned int i=0; i<symbolics.size(); i++)
    symbolics[i].first->refCount++;

}

ExecutionState *ExecutionState::branch() {
  depth++;

  ExecutionState *falseState = new ExecutionState(*this);
  falseState->coveredNew = false;
  falseState->coveredLines.clear();

  weight *= .5;
  falseState->weight -= weight;
  falseState->crtThreadIt = falseState->threads.find(crtThreadIt->second.tid);

  return falseState;
}

void ExecutionState::pushFrame(KInstIterator caller, KFunction *kf) {
  stack().push_back(StackFrame(caller,kf));
}

void ExecutionState::popFrame(Thread &t) {
    StackFrame &sf = t.stack.back();
    for (std::vector<const MemoryObject*>::iterator it = sf.allocas.begin(),
            ie = sf.allocas.end(); it != ie; ++it)
        addressSpace.unbindObject(*it);
    t.stack.pop_back();
}

void ExecutionState::popFrame() {
    popFrame(crtThread());
}

void ExecutionState::addSymbolic(const MemoryObject *mo, const Array *array) {
  mo->refCount++;
  symbolics.push_back(std::make_pair(mo, array));
}
///

std::string ExecutionState::getFnAlias(std::string fn) {
  std::map < std::string, std::string >::iterator it = fnAliases.find(fn);
  if (it != fnAliases.end())
    return it->second;
  else return "";
}

void ExecutionState::addFnAlias(std::string old_fn, std::string new_fn) {
  fnAliases[old_fn] = new_fn;
}

void ExecutionState::removeFnAlias(std::string fn) {
  fnAliases.erase(fn);
}

Thread& ExecutionState::createThread(Thread::thread_id_t tid, KFunction *kf) {
    Thread newThread = Thread(tid,  kf);
    threads.insert(std::make_pair(newThread.tid, newThread));
    return threads.find(tid)->second;
}

void ExecutionState::terminateThread(threads_ty::iterator thrIt) {
    assert(threads.size() > 1);
    assert(thrIt != crtThreadIt); // We assume the scheduler found a new thread first
    assert(!thrIt->second.enabled);
    assert(thrIt->second.waitingList == 0);
    threads.erase(thrIt);
}

void ExecutionState::sleepThread(Thread::wlist_id_t wlist) {
    assert(crtThread().enabled);
    assert(wlist > 0);

    crtThread().enabled = false;
    crtThread().waitingList = wlist;

    std::set<Thread::thread_id_t> &wl = waitingLists[wlist];

    wl.insert(crtThread().tid);
}

void ExecutionState::notifyOne(Thread::wlist_id_t wlist, Thread::thread_id_t tid) {
    assert(wlist > 0);

    std::set<Thread::thread_id_t> &wl = waitingLists[wlist];

    if (wl.erase(tid) != 1) {
        assert(0 && "thread was not waiting");
    }

    Thread &thread = threads.find(tid)->second;
    assert(!thread.enabled);
    thread.enabled = true;
    thread.waitingList = 0;

    if (wl.size() == 0)
        waitingLists.erase(wlist);
}

void ExecutionState::notifyAll(Thread::wlist_id_t wlist) {
    assert(wlist > 0);

    std::set<Thread::thread_id_t> &wl = waitingLists[wlist];

    if (wl.size() > 0) {
        for (std::set<Thread::thread_id_t>::iterator it = wl.begin(); it != wl.end(); it++) {
            Thread &thread = threads.find(*it)->second;
            thread.enabled = true;
            thread.waitingList = 0;
        }

        wl.clear();
    }

    waitingLists.erase(wlist);
}


/**/

llvm::raw_ostream &klee::operator<<(llvm::raw_ostream &os, const MemoryMap &mm) {
  os << "{";
  MemoryMap::iterator it = mm.begin();
  MemoryMap::iterator ie = mm.end();
  if (it!=ie) {
    os << "MO" << it->first->id << ":" << it->second;
    for (++it; it!=ie; ++it)
      os << ", MO" << it->first->id << ":" << it->second;
  }
  os << "}";
  return os;
}

bool ExecutionState::merge(const ExecutionState &b) {
  if (DebugLogStateMerge)
    llvm::errs() << "-- attempting merge of A:" << this << " with B:" << &b
                 << "--\n";
  if (pc() != b.pc())
    return false;

  // XXX being conservative, how should be merged states with multiple threads?
  if (threads.size() != 1 || b.threads.size() != 1)
      return false;

  if (crtThread().tid != b.crtThread().tid) // No merge if different thread
    return false;

  // XXX is it even possible for these to differ? does it matter? probably
  // implies difference in object states?
  if (symbolics!=b.symbolics)
    return false;

  {
    std::vector<StackFrame>::const_iterator itA = stack().begin();
    std::vector<StackFrame>::const_iterator itB = b.stack().begin();
    while (itA!=stack().end() && itB!=b.stack().end()) {
      // XXX vaargs?
      if (itA->caller!=itB->caller || itA->kf!=itB->kf)
        return false;
      ++itA;
      ++itB;
    }
    if (itA!=stack().end() || itB!=b.stack().end())
      return false;
  }

  std::set< ref<Expr> > aConstraints(constraints.begin(), constraints.end());
  std::set< ref<Expr> > bConstraints(b.constraints.begin(),
                                     b.constraints.end());
  std::set< ref<Expr> > commonConstraints, aSuffix, bSuffix;
  std::set_intersection(aConstraints.begin(), aConstraints.end(),
                        bConstraints.begin(), bConstraints.end(),
                        std::inserter(commonConstraints, commonConstraints.begin()));
  std::set_difference(aConstraints.begin(), aConstraints.end(),
                      commonConstraints.begin(), commonConstraints.end(),
                      std::inserter(aSuffix, aSuffix.end()));
  std::set_difference(bConstraints.begin(), bConstraints.end(),
                      commonConstraints.begin(), commonConstraints.end(),
                      std::inserter(bSuffix, bSuffix.end()));
  if (DebugLogStateMerge) {
    llvm::errs() << "\tconstraint prefix: [";
    for (std::set<ref<Expr> >::iterator it = commonConstraints.begin(),
                                        ie = commonConstraints.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
    llvm::errs() << "\tA suffix: [";
    for (std::set<ref<Expr> >::iterator it = aSuffix.begin(),
                                        ie = aSuffix.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
    llvm::errs() << "\tB suffix: [";
    for (std::set<ref<Expr> >::iterator it = bSuffix.begin(),
                                        ie = bSuffix.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
  }

  // We cannot merge if addresses would resolve differently in the
  // states. This means:
  //
  // 1. Any objects created since the branch in either object must
  // have been free'd.
  //
  // 2. We cannot have free'd any pre-existing object in one state
  // and not the other

  if (DebugLogStateMerge) {
    llvm::errs() << "\tchecking object states\n";
    llvm::errs() << "A: " << addressSpace.objects << "\n";
    llvm::errs() << "B: " << b.addressSpace.objects << "\n";
  }

  std::set<const MemoryObject*> mutated;
  MemoryMap::iterator ai = addressSpace.objects.begin();
  MemoryMap::iterator bi = b.addressSpace.objects.begin();
  MemoryMap::iterator ae = addressSpace.objects.end();
  MemoryMap::iterator be = b.addressSpace.objects.end();
  for (; ai!=ae && bi!=be; ++ai, ++bi) {
    if (ai->first != bi->first) {
      if (DebugLogStateMerge) {
        if (ai->first < bi->first) {
          llvm::errs() << "\t\tB misses binding for: " << ai->first->id << "\n";
        } else {
          llvm::errs() << "\t\tA misses binding for: " << bi->first->id << "\n";
        }
      }
      return false;
    }
    if (ai->second != bi->second) {
      if (DebugLogStateMerge)
        llvm::errs() << "\t\tmutated: " << ai->first->id << "\n";
      mutated.insert(ai->first);
    }
  }
  if (ai!=ae || bi!=be) {
    if (DebugLogStateMerge)
      llvm::errs() << "\t\tmappings differ\n";
    return false;
  }

  // merge stack

  ref<Expr> inA = ConstantExpr::alloc(1, Expr::Bool);
  ref<Expr> inB = ConstantExpr::alloc(1, Expr::Bool);
  for (std::set< ref<Expr> >::iterator it = aSuffix.begin(),
         ie = aSuffix.end(); it != ie; ++it)
    inA = AndExpr::create(inA, *it);
  for (std::set< ref<Expr> >::iterator it = bSuffix.begin(),
         ie = bSuffix.end(); it != ie; ++it)
    inB = AndExpr::create(inB, *it);

  // XXX should we have a preference as to which predicate to use?
  // it seems like it can make a difference, even though logically
  // they must contradict each other and so inA => !inB

  std::vector<StackFrame>::iterator itA = stack().begin();
  std::vector<StackFrame>::const_iterator itB = b.stack().begin();
  for (; itA!=stack().end(); ++itA, ++itB) {
    StackFrame &af = *itA;
    const StackFrame &bf = *itB;
    for (unsigned i=0; i<af.kf->numRegisters; i++) {
      ref<Expr> &av = af.locals[i].value;
      const ref<Expr> &bv = bf.locals[i].value;
      if (av.isNull() || bv.isNull()) {
        // if one is null then by implication (we are at same pc)
        // we cannot reuse this local, so just ignore
      } else {
        av = SelectExpr::create(inA, av, bv);
      }
    }
  }

  for (std::set<const MemoryObject*>::iterator it = mutated.begin(),
         ie = mutated.end(); it != ie; ++it) {
    const MemoryObject *mo = *it;
    const ObjectState *os = addressSpace.findObject(mo);
    const ObjectState *otherOS = b.addressSpace.findObject(mo);
    assert(os && !os->readOnly &&
           "objects mutated but not writable in merging state");
    assert(otherOS);

    ObjectState *wos = addressSpace.getWriteable(mo, os);
    for (unsigned i=0; i<mo->size; i++) {
      ref<Expr> av = wos->read8(i);
      ref<Expr> bv = otherOS->read8(i);
      wos->write(i, SelectExpr::create(inA, av, bv));
    }
  }

  constraints = ConstraintManager();
  for (std::set< ref<Expr> >::iterator it = commonConstraints.begin(),
         ie = commonConstraints.end(); it != ie; ++it)
    constraints.addConstraint(*it);
  constraints.addConstraint(OrExpr::create(inA, inB));

  return true;
}

void ExecutionState::dumpStack(llvm::raw_ostream &out) const {
  unsigned idx = 0;
  const KInstruction *target = prevPC();
  for (Thread::stack_ty::const_reverse_iterator
         it = stack().rbegin(), ie = stack().rend();
       it != ie; ++it) {
    const StackFrame &sf = *it;
    Function *f = sf.kf->function;
    const InstructionInfo &ii = *target->info;
    out << "\t#" << idx++;
    std::stringstream AssStream;
    AssStream << std::setw(8) << std::setfill('0') << ii.assemblyLine;
    out << AssStream.str();
    out << " in " << f->getName().str() << " (";
    // Yawn, we could go up and print varargs if we wanted to.
    unsigned index = 0;
    for (Function::arg_iterator ai = f->arg_begin(), ae = f->arg_end();
         ai != ae; ++ai) {
      if (ai!=f->arg_begin()) out << ", ";

      out << ai->getName().str();
      // XXX should go through function
      ref<Expr> value = sf.locals[sf.kf->getArgRegister(index++)].value;
      if (isa<ConstantExpr>(value))
        out << "=" << value;
    }
    out << ")";
    if (ii.file != "")
      out << " at " << ii.file << ":" << ii.line;
    out << "\n";
    target = sf.caller;
  }
}

///MODIFICATION

#include "Common.h"

    void ExecutionState::updateVC(uint32_t tid, VectorClock &vc)
    {
        //llvm::errs() << "called updateVC\n";
        std::map<Thread::thread_id_t, Thread>::iterator iter = threads.find(tid);
        //Thread& value =

        //value.vc.import(vc);
        if (iter == threads.end())
            llvm::errs() << "updateVC error\n";
            //printf("updateVC Error\n");
        else
        {
            //printf("happens before? %i\n", iter->second.vc.happensBefore(vc));
            iter->second.vc.import(vc);
     //       std::string Str;
     //       llvm::raw_string_ostream msg(Str);
     //       msg << "thread: " << tid  << "\t(" << iter->second.vc.toString() << ")";

     //       klee::klee_message("%s", msg.str().c_str());
            //llvm::errs() << "thread: " << tid << " vc: " << vc.toString();
        }

    }
/*
    void ExecutionState::handleMemoryAccess(MemoryObject *mo, std::map<uint32_t, VectorClock>& accessContainer)
    {
        //printf("handleMemoryAccess(0x%x, %s)\n", mo, &accessContainer == &mo->lastReadAccesses? "read":"write");
        //accessContainer.iterator_type iter;
        //iter = accessContainer.find(crtThread()->getTid());
        //if (iter == accessContainer.end())
        {
            accessContainer[crtThread().getTid()] = crtThread().vc;
        }
        //else
        //{
         //   accessContainer
        //}

        analyzeForRaceCondition(mo);
    }

    void ExecutionState::handleMemoryReadAccess(MemoryObject *mo)
    {
        handleMemoryAccess(mo, mo->lastReadAccesses);
    }

    void ExecutionState::handleMemoryWriteAccess(MemoryObject *mo)
    {
        handleMemoryAccess(mo, mo->lastWriteAccesses);
    }

    void ExecutionState::handleMemoryReadAccess(ObjectState *os, KInstruction *kInst)
    {
        //handleMemoryAccess(os, os->lastReadAccesses, kInst);
        handleMemoryAccess(os, os->readAccesses, kInst);
    }

    void ExecutionState::handleMemoryWriteAccess(ObjectState *os, KInstruction *kInst)
    {
        //handleMemoryAccess(os, os->lastWriteAccesses, kInst);
        handleMemoryAccess(os, os->writeAccesses, kInst);
    }*/
    bool ExecutionState::handleMemoryReadAccess(ObjectState *os, KInstruction *kInst)
    {
        return handleMemoryAccess(os, kInst, false);
    }

    bool ExecutionState::handleMemoryWriteAccess(ObjectState *os, KInstruction *kInst)
    {
        return handleMemoryAccess(os, kInst, true);
    }

    bool ExecutionState::handleMemoryAccess(ObjectState *os, KInstruction *kInst, bool write)
    {
        std::stringstream ss;
        ss << kInst->info->file << ":" << kInst->info->line;
        std::pair<ObjectState::access_iterator_t, bool> insertInfo = os->memOperations.insert(MemoryAccessEntry(crtThread().getTid(),
                                                                                                                crtThread().vc,
                                                                                                                os->getObject()->allocSite->getName().str(),
                                                                                                                ss.str(),
                                                                                                                write));
        if (insertInfo.second && !insertInfo.first->isRuntime())
            return analyzeForRaceCondition(os, insertInfo.first);
        return false;
    }

    bool ExecutionState::analyzeForRaceCondition(ObjectState *os, ObjectState::access_iterator_t newElement)
    {
        bool needsTest = false;
        ObjectState::access_iterator_t iter = os->memOperations.begin();
        while (iter != os->memOperations.end())
        {
            if (iter != newElement)
            {
                if (iter->isRace(*newElement))
                {
                    RaceReport rr(*iter, *newElement);
                    std::pair<std::set<RaceReport>::iterator, bool> insertState = RaceReport::overallReports.insert(rr);
                    if (insertState.second)
                    {
                        needsTest = true;
                        klee::klee_message("%u\t%s\n", RaceReport::overallReports.size(), rr.toString().c_str());
                    }
                }
            }
            iter++;
        }
        return needsTest;
    }

/*

    void ExecutionState::handleMemoryAccess(ObjectState *os, ObjectState::access_containter_t &container, KInstruction *kInst)
    {
        container[crtThread().getTid()] = crtThread().vc;
        analyzeForRaceCondition(os, kInst);
    }

    void ExecutionState::handleMemoryAccess(ObjectState *os, ObjectState::access_register_t &container, KInstruction *kInst)
    {
        //container[crtThread().getTid()] = crtThread().vc;

        std::pair<uint32_t, MemoryAccessEntry> index(crtThread().getTid(), kInst);
        container[index] = crtThread().vc;
        //container[index].setOperation(crtThread().getTid(), kInst)

        analyzeForRaceCondition(os, kInst);
    }


//#include "llvm/IR/Instructions.h"

    void ExecutionState::analyzeForRaceCondition(ObjectState::access_register_t::iterator iter1,
                                                 ObjectState::access_register_t::iterator end1,
                                                 ObjectState::access_register_t::iterator iter2,
                                                 ObjectState::access_register_t::iterator end2,
                                                 ObjectState *os, KInstruction *kInst)
    {
        ObjectState::access_register_t::iterator start2 = iter2;

        while(iter1 != end1)
        {
            iter2 = start2;
            while (iter2 != end2)
            {
                if (iter1->first.first != iter2->first.first)
                {
                    if (!iter1->second.happensBefore(iter2->second) && !iter2->second.happensBefore(iter1->second))
                    {
                        if (kInst->info->file.find("POSIX") == std::string::npos && kInst->info->file.find("Intrinsic") == std::string::npos)
                        {
                            //RaceReport rr(iter2->first.first,iter1->first.first, kInst, os);
                            RaceReport rr(iter1->first.first, kInst, os, iter2->first.second, iter2->first.first, iter1->second, iter2->second);

                            std::pair<std::set<RaceReport>::iterator, bool> insertState = RaceReport::overallReports.insert(rr);
                            if (insertState.second)
                            {
                                printf("%u\t%s\n", RaceReport::overallReports.size(), rr.toString().c_str());
                            }
                        }
                    }
                }
                iter2++;
            }
            iter1++;
        }
    }

    void ExecutionState::analyzeForRaceCondition(std::map<uint32_t, VectorClock>::iterator iter1,
                                                std::map<uint32_t, VectorClock>::iterator end1,
                                                std::map<uint32_t, VectorClock>::iterator iter2,
                                                std::map<uint32_t, VectorClock>::iterator end2,
                                                ObjectState *os, KInstruction *kInst)
    {
        //KInstruction *kInst = pc();
        //const InstructionInfo *info = kInst->info;

        std::map<uint32_t, VectorClock>::iterator start2 = iter2;

        while(iter1 != end1)
        {
            iter2 = start2;
            while (iter2 != end2)
            {
                if (iter1->first != iter2->first)
                {
                    if (!iter1->second.happensBefore(iter2->second) && !iter2->second.happensBefore(iter1->second))
                    {
                        if (kInst->info->file.find("POSIX") == std::string::npos && kInst->info->file.find("Intrinsic") == std::string::npos)
                        {
                            //llvm::Instruction *inst = kInst->inst;

                                //std::string opcodeName = kInst->inst->getOpcodeName();
                                //if (kInst->inst->getOpcode() == llvm::Instruction::Add) //if (opcodeName == "add") //if (opcodeName == "store" || opcodeName == "load")
                                {
                            //RaceReport rr(iter2->first, iter1->first, info->file, info->line);
                            RaceReport rr(iter2->first,iter1->first, kInst, os);
                            if (RaceReport::overallReports.find(rr) == RaceReport::overallReports.end())
                            {


                                RaceReport::overallReports.insert(rr);

                                printf("%u\t%s\n", RaceReport::overallReports.size(), rr.toString().c_str());
                                }
                            }
                        }
                    }
                    //printf("Race detected for mo: 0x%x between thread%i and thread%i at %s:%i\n", mo, rIter->first, wIter->first, info->file.c_str(), info->line);
                }
                iter2++;
            }
            iter1++;
        }
    }

    void ExecutionState::analyzeForRaceCondition(ObjectState *os, KInstruction *kInst)
    {
 /*       std::map<uint32_t, VectorClock> &reads = os->lastReadAccesses;
        std::map<uint32_t, VectorClock> &writes = os->lastWriteAccesses;

        std::map<uint32_t, VectorClock>::iterator rIter = reads.begin();
        std::map<uint32_t, VectorClock>::iterator wIter = writes.begin();

        analyzeForRaceCondition(rIter, reads.end(), wIter, writes.end(), os, kInst);
        analyzeForRaceCondition(wIter, writes.end(), wIter, writes.end(), os, kInst);
 * /
        ObjectState::access_register_t &reads = os->readAccesses;
        ObjectState::access_register_t &writes = os->writeAccesses;

        ObjectState::access_register_t::iterator rIter = reads.begin();
        ObjectState::access_register_t::iterator wIter = writes.begin();

        analyzeForRaceCondition(rIter, reads.end(), wIter, writes.end(), os, kInst);
        analyzeForRaceCondition(wIter, writes.end(), wIter, writes.end(), os, kInst);
    }
/*
    void ExecutionState::analyzeForRaceCondition(MemoryObject *mo)
    {
        ///TODO the Race Detector goes here

        std::map<uint32_t, VectorClock> &reads = mo->lastReadAccesses;
        std::map<uint32_t, VectorClock> &writes = mo->lastWriteAccesses;

        //MemoryObject::access_containter_t::iterator_type iter = mo->lastReadAccesses.begin();
        std::map<uint32_t, VectorClock>::iterator rIter = reads.begin();
        std::map<uint32_t, VectorClock>::iterator wIter = writes.begin();

        analyzeForRaceCondition(rIter, reads.end(), wIter, writes.end());
        analyzeForRaceCondition(wIter, writes.end(), wIter, writes.end());
*/
        //printf("raceAnalysis(0x%x) readers: %i\twriters: %i\n", mo, reads.size(), writes.size());
/*
        KInstruction *kInst = pc();
        const InstructionInfo *info = kInst->info;


        ///ANALYZE FOR RAW and WAR
        while(rIter != reads.end())
        {
            wIter = writes.begin();
            while (wIter != writes.end())
            {
                if (rIter->first != wIter->first)
                {
                    if (!rIter->second.happensBefore(wIter->second) && !wIter->second.happensBefore(rIter->second))
                    {
                        RaceReport rr(wIter->first, rIter->first, info->file, info->line);
                        if (reports.find(rr) == reports.end())
                        {
                            reports.insert(rr);
                            printf("%u\t%s\n", reports.size(), rr.toString().c_str());
                        }
                    }
                    //printf("Race detected for mo: 0x%x between thread%i and thread%i at %s:%i\n", mo, rIter->first, wIter->first, info->file.c_str(), info->line);
                }
                wIter++;
            }
            rIter++;
        }

        std::map<uint32_t, VectorClock>::iterator wIter2 = writes.begin();
        while (wIter2 != writes.end())
        {
            wIter = writes.begin();
            while (wIter != writes.end())
            {
                if (wIter->first != wIter2->first)
                {
                    if (!wIter2->second.happensBefore(wIter->second) && !wIter->second.happensBefore(wIter2->second))
                    {
                        RaceReport rr(wIter->first, wIter2->first, info->file, info->line);
                        if (reports.find(rr) == reports.end())
                        {
                            reports.insert(rr);
                            printf("%u\t%s\n", reports.size(), rr.toString().c_str());
                        }
                    }
                    //printf("Race detected for mo: 0x%x between thread%i and thread%i at %s:%i\n", mo, wIter2->first, wIter->first, info->file.c_str(), info->line);
                }
                wIter++;
            }
            wIter2++;
        }*/
/*
        while(rIter != mo->lastReadAccesses.end() || wIter != mo->lastWriteAccesses.end())
        {
            if (rIter == mo->lastReadAccesses.end())
            {
                wIter++;
            }
            else if (wIter == mo->lastWriteAccesses.end())
            {
                rIter++;
            }
            else
            {
                wIter++;
                rIter++;
            }
        }*/
  //  }

    //const VectorClock& ExecutionState::getCurrentVC() const
    //{
   //     return crtThread().vc;
 //   }

///MODIFICATION END
