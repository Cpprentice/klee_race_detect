//===-- ExecutionState.h ----------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_EXECUTIONSTATE_H
#define KLEE_EXECUTIONSTATE_H

#include "klee/Constraints.h"
#include "klee/Expr.h"
#include "klee/Internal/ADT/TreeStream.h"

// FIXME: We do not want to be exposing these? :(
#include "../../lib/Core/AddressSpace.h"
#include "../../lib/Core/Threading.h"
#include "klee/Internal/Module/KInstIterator.h"

#include <map>
#include <set>
#include <vector>

///MODIFICATION
#include "../../lib/Core/RaceReport.h"
#include "../../lib/Core/VectorClock.h"
#include "../../lib/Core/Memory.h"

///MODIFCIATION END

namespace klee {
  class Array;
  struct KFunction;
  struct KInstruction;
  class MemoryObject;
  class PTreeNode;
  struct InstructionInfo;

llvm::raw_ostream &operator<<(llvm::raw_ostream &os, const MemoryMap &mm);

class ExecutionState {

public:
  typedef std::map<Thread::thread_id_t, Thread> threads_ty;
  typedef std::map<Thread::wlist_id_t, std::set<Thread::thread_id_t> > wlists_ty;

private:
  // unsupported, use copy constructor
  ExecutionState &operator=(const ExecutionState&);
  std::map< std::string, std::string > fnAliases;

public:
  bool fakeState;
  unsigned depth;

  ConstraintManager constraints;
  mutable double queryCost;
  double weight;
  AddressSpace addressSpace;
  TreeOStream pathOS, symPathOS;
  unsigned instsSinceCovNew;
  bool coveredNew;

  /// Disables forking, set by user code.
  bool forkDisabled;

  std::map<const std::string*, std::set<unsigned> > coveredLines;
  PTreeNode *ptreeNode;

  /// ordered list of symbolics: used to generate test cases.
  //
  // FIXME: Move to a shared list structure (not critical).
  std::vector< std::pair<const MemoryObject*, const Array*> > symbolics;

  /// Set of used array names.  Used to avoid collisions.
  std::set<std::string> arrayNames;

  // Used by the checkpoint/rollback methods for fake objects.
  // FIXME: not freeing things on branch deletion.
  MemoryMap shadowObjects;

  std::string getFnAlias(std::string fn);
  void addFnAlias(std::string old_fn, std::string new_fn);
  void removeFnAlias(std::string fn);

  // Thread scheduling
  // For a multi threaded ExecutionState
  threads_ty threads;

  wlists_ty waitingLists;
  Thread::wlist_id_t wlistCounter;
  unsigned int preemptions;

  Thread& createThread(Thread::thread_id_t tid, KFunction *kf);
  void terminateThread(threads_ty::iterator it);

  threads_ty::iterator nextThread(threads_ty::iterator it) {
      if (it == threads.end())
          it = threads.begin();
      else {
          it++;
          if (it == threads.end())
              it = threads.begin();
      }

      return it;
  }

  void scheduleNext(threads_ty::iterator it) {
      assert(it != threads.end());
      crtThreadIt = it;
      schedulingHistory.push_back(crtThread().tid);
  }

  Thread::wlist_id_t getWaitingList() { return wlistCounter++; }
  void sleepThread(Thread::wlist_id_t wlist);
  void notifyOne(Thread::wlist_id_t wlist, Thread::thread_id_t tid);
  void notifyAll(Thread::wlist_id_t wlist);

  threads_ty::iterator crtThreadIt;

  std::vector<Thread::thread_id_t> schedulingHistory;

  /* Shortcut methods */

  ///MODIFICATION
  void updateVC(uint32_t tid, VectorClock &vc);

  //const VectorClock& getCurrentVC() const;
  //typedef std::map<uint32_t, VectorClock> access_containter_t;
//  void handleMemoryAccess(MemoryObject *mo, MemoryObject::access_containter_t& accessContainer);
//  void handleMemoryReadAccess(MemoryObject *mo);
 // void handleMemoryWriteAccess(MemoryObject *mo);

  bool handleMemoryWriteAccess(ObjectState *os, KInstruction *kInst);
  bool handleMemoryReadAccess(ObjectState *os, KInstruction *kInst);
  bool handleMemoryAccess(ObjectState *os, KInstruction *kInst, bool write);
  bool analyzeForRaceCondition(ObjectState *os, ObjectState::access_iterator_t newElement);

//  void handleMemoryAccess(ObjectState *os, ObjectState::access_containter_t &container, KInstruction *kInst);
 // void handleMemoryAccess(ObjectState *os, ObjectState::access_register_t &container, KInstruction *kInst);

//  void analyzeForRaceCondition(ObjectState *os, KInstruction *kInst);
  //void analyzeForRaceCondition(MemoryObject *mo);
 /* void analyzeForRaceCondition(std::map<uint32_t, VectorClock>::iterator iter1,
                               std::map<uint32_t, VectorClock>::iterator end1,
                               std::map<uint32_t, VectorClock>::iterator iter2,
                               std::map<uint32_t, VectorClock>::iterator end2,
                               ObjectState *os, KInstruction *kInst );
  void analyzeForRaceCondition(ObjectState::access_register_t::iterator iter1,
                               ObjectState::access_register_t::iterator end1,
                               ObjectState::access_register_t::iterator iter2,
                               ObjectState::access_register_t::iterator end2,
                               ObjectState *os, KInstruction *kInst );*/

 // std::set<RaceReport> reports;

  ///MODIFICATION END

  Thread &crtThread() { return crtThreadIt->second; }
  const Thread &crtThread() const { return crtThreadIt->second; }

  KInstIterator& pc() { return crtThread().pc; }
  const KInstIterator& pc() const { return crtThread().pc; }

  KInstIterator& prevPC() { return crtThread().prevPC; }
  const KInstIterator& prevPC() const { return crtThread().prevPC; }

  Thread::stack_ty& stack() { return crtThread().stack; }
  const Thread::stack_ty& stack() const { return crtThread().stack; }

  unsigned incomingBBIndex() { return crtThread().incomingBBIndex; }
  void incomingBBIndex(unsigned ibbi) { crtThread().incomingBBIndex = ibbi; }

private:
  ExecutionState() : fakeState(false), ptreeNode(0) {}

  void setupMain(KFunction *kf);
public:
  ExecutionState(KFunction *kf);

  // XXX total hack, just used to make a state so solver can
  // use on structure
  ExecutionState(const std::vector<ref<Expr> > &assumptions);

  ExecutionState(const ExecutionState& state);

  ~ExecutionState();

  ExecutionState *branch();

  void pushFrame(KInstIterator caller, KFunction *kf);
  void popFrame(Thread &t);
  void popFrame();

  void addSymbolic(const MemoryObject *mo, const Array *array);
  void addConstraint(ref<Expr> e) {
    constraints.addConstraint(e);
  }

  bool merge(const ExecutionState &b);
  void dumpStack(llvm::raw_ostream &out) const;
};

}

#endif
