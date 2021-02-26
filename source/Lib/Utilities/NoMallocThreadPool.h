/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software,
especially patent licenses, a separate Agreement needs to be closed.
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2018-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

#ifndef NOMALLOCTHREADPOOL_H
#define NOMALLOCTHREADPOOL_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>
#include <array>

#include "CommonLib/CommonDef.h"

#ifdef TRACE_ENABLE_ITT
static __itt_domain* itt_domain_thrd = __itt_domain_create( "Threading" );

static __itt_string_handle* itt_handle_TPspinWait = __itt_string_handle_create( "Spin_Wait" );
static __itt_string_handle* itt_handle_TPblocked  = __itt_string_handle_create( "Blocked" );
static __itt_string_handle* itt_handle_TPaddTask  = __itt_string_handle_create( "Add_Task" );

// static long itt_TP_blocked = 1;
#endif   // TRACE_ENABLE_ITT


// enable this if tasks need to be added from mutliple threads
#define ADD_TASK_THREAD_SAFE 0


// ---------------------------------------------------------------------------
// Synchronization tools
// ---------------------------------------------------------------------------

struct Barrier
{
  void unlock()
  {
    m_lockState.store( false );
  }

  void lock()
  {
    m_lockState.store( true );
  }

  bool isBlocked() const
  {
    return m_lockState;
  }

  Barrier()  = default;
  ~Barrier() = default;
  explicit Barrier( bool locked ) : m_lockState( locked ) {}

  Barrier( const Barrier & ) = delete;
  Barrier( Barrier && )      = delete;

  Barrier& operator=( const Barrier & ) = delete;
  Barrier& operator=( Barrier && )      = delete;

private:
  std::atomic_bool m_lockState{ true };
};

struct BlockingBarrier
{
  void unlock()
  {
    std::unique_lock<std::mutex> l( m_lock );
    m_intBarrier.unlock();
    m_cond.notify_all();
  }

  void lock()
  {
    std::unique_lock<std::mutex> l( m_lock );
    m_intBarrier.lock();
  }

  bool isBlocked() const
  {
    return m_intBarrier.isBlocked();
  }

  void wait() const
  {
    BlockingBarrier* nonconst = const_cast<BlockingBarrier*>(this);

    std::unique_lock<std::mutex> l( nonconst->m_lock );
    nonconst->m_cond.wait( l, [=] { return !m_intBarrier.isBlocked(); } );
  }

  BlockingBarrier()  = default;
  ~BlockingBarrier() { std::unique_lock<std::mutex> l( m_lock ); } // ensure all threads have unlocked the mutex, when we start destruction

  BlockingBarrier( const BlockingBarrier& ) = delete;
  BlockingBarrier( BlockingBarrier&& )      = delete;

  BlockingBarrier& operator=( const BlockingBarrier& ) = delete;
  BlockingBarrier& operator=( BlockingBarrier&& ) = delete;

  // cast to const ref Barrier, so we can use it for thread pool tasks:
  operator const Barrier&() const { return m_intBarrier; }

private:
  Barrier                 m_intBarrier;
  std::condition_variable m_cond;
  std::mutex              m_lock;
};

struct WaitCounter
{
  int operator++()
  {
    std::unique_lock<std::mutex> l( m_lock );
    m_done.lock();
    return ++m_count;
  }

  int operator--()
  {
    std::unique_lock<std::mutex> l( m_lock );
    const unsigned int new_count = --m_count;
    if( new_count == 0 )
    {
      m_done.unlock();
      m_cond.notify_all();
    }
    l.unlock(); // unlock mutex after done-barrier to prevent race between barrier and counter
    return new_count;
  }

  bool isBlocked() const
  {
    std::unique_lock<std::mutex> l( const_cast<std::mutex&>( m_lock ) );
    return 0 != m_count;
  }

  void wait() const
  {
    WaitCounter* nonconst = const_cast<WaitCounter*>(this);

    std::unique_lock<std::mutex> l( nonconst->m_lock );
    nonconst->m_cond.wait( l, [=] { return m_count == 0; } );
  }

  WaitCounter() = default;
  ~WaitCounter() { std::unique_lock<std::mutex> l( m_lock ); }   // ensure all threads have unlocked the mutex, when we start destruction

  WaitCounter( const WaitCounter & ) = delete;
  WaitCounter( WaitCounter && )      = delete;

  WaitCounter &operator=( const WaitCounter & ) = delete;
  WaitCounter &operator=( WaitCounter && )      = delete;

  const Barrier* donePtr() const { return &m_done; }

private:
  std::condition_variable m_cond;
  std::mutex              m_lock;
  unsigned int            m_count = 0;
  Barrier                 m_done{ false };
};



// ---------------------------------------------------------------------------
// Thread Pool
// ---------------------------------------------------------------------------

using CBarrierVec = std::vector<const Barrier*>;

class NoMallocThreadPool
{
  typedef enum
  {
    FREE = 0,
    PREPARING,
    WAITING,
    RUNNING
  } TaskState;

  using TaskFunc = bool ( * )( int, void * );

  struct Slot
  {
    TaskFunc               func      { nullptr };
    TaskFunc               readyCheck{ nullptr };
    void*                  param     { nullptr };
    WaitCounter*           counter   { nullptr };
    Barrier*               done      { nullptr };
    CBarrierVec            barriers;
    std::atomic<TaskState> state     { FREE };
  };


  class ChunkedTaskQueue
  {
    constexpr static int ChunkSize = 128;

    class Chunk
    {
      std::array<Slot, ChunkSize> m_slots;
      std::atomic<Chunk*>         m_next{ nullptr };
      Chunk&                      m_firstChunk;

      Chunk( Chunk* firstPtr ) : m_firstChunk{ *firstPtr } {}

      friend class ChunkedTaskQueue;
    };

  public:
    class Iterator : public std::iterator<std::forward_iterator_tag, Slot>
    {
      Slot*  m_slot  = nullptr;
      Chunk* m_chunk = nullptr;

    public:
      Iterator() = default;
      Iterator( Slot* slot, Chunk* chunk ) : m_slot( slot ), m_chunk( chunk ) {}

      Iterator& operator++();

      // increment iterator and wrap around, if end is reached
      Iterator& incWrap();

      bool operator==( const Iterator& rhs ) const { return m_slot == rhs.m_slot; } // don't need to compare m_chunk, because m_slot is a pointer
      bool operator!=( const Iterator& rhs ) const { return m_slot != rhs.m_slot; } // don't need to compare m_chunk, because m_slot is a pointer

      Slot& operator*() { return *m_slot; }

      bool isValid() const { return m_slot != nullptr && m_chunk != nullptr; }
    };

    ChunkedTaskQueue() = default;
    ~ChunkedTaskQueue();

    ChunkedTaskQueue( const ChunkedTaskQueue& ) = delete;
    ChunkedTaskQueue( ChunkedTaskQueue&& )      = delete;

    // grow the queue by adding a chunk and return an iterator to the first new task-slot
    Iterator grow();

    Iterator begin() { return Iterator{ &m_firstChunk.m_slots.front(), &m_firstChunk }; }
    Iterator end()   { return Iterator{ nullptr, nullptr }; }

  private:
    Chunk  m_firstChunk{ &m_firstChunk };
    Chunk* m_lastChunk = &m_firstChunk;

    std::mutex m_resizeMutex;
  };


public:
  NoMallocThreadPool( int numThreads = 1, const char *threadPoolName = nullptr );
  ~NoMallocThreadPool();

  template<class TParam>
  bool addBarrierTask( bool       ( *func )( int, TParam* ),
                       TParam*       param,
                       WaitCounter*  counter                      = nullptr,
                       Barrier*      done                         = nullptr,
                       CBarrierVec&& barriers                     = {},
                       bool       ( *readyCheck )( int, TParam* ) = nullptr )
  {
    if( m_threads.empty() )
    {
      // if singlethreaded, execute all pending tasks

      bool waiting_tasks = m_nextFillSlot != m_tasks.begin();
      bool is_ready      = checkTaskReady( 0, barriers, (TaskFunc)readyCheck, param );
      if( !is_ready && waiting_tasks )
      {
        waiting_tasks = processTasksOnMainThread();
        is_ready      = checkTaskReady( 0, barriers, (TaskFunc)readyCheck, param );
      }

      // when no barriers block this task, execute it directly
      if( is_ready )
      {
        if( func( 0, param ) )
        {
          if( done != nullptr )
          {
            done->unlock();
          }

          if( waiting_tasks )
          {
            processTasksOnMainThread();
          }
          return true;
        }
      }
    }

    while( true )
    {
#if ADD_TASK_THREAD_SAFE
      std::unique_lock<std::mutex> l(m_nextFillSlotMutex);
#endif
      CHECKD( !m_nextFillSlot.isValid(), "Next fill slot iterator should always be valid" );
      const auto startIt = m_nextFillSlot;

#if ADD_TASK_THREAD_SAFE
      l.unlock();
#endif

      bool first = true;
      for( auto it = startIt; it != startIt || first; it.incWrap() )
      {
        first = false;

        auto& t = *it;
        auto expected = FREE;
        if( t.state.load( std::memory_order_relaxed ) == FREE && t.state.compare_exchange_strong( expected, PREPARING ) )
        {
          if( counter )
          {
            counter->operator++();
          }

          t.func       = (TaskFunc)func;
          t.readyCheck = (TaskFunc)readyCheck;
          t.param      = param;
          t.done       = done;
          t.counter    = counter;
          t.barriers   = std::move( barriers );
          t.state      = WAITING;

#if ADD_TASK_THREAD_SAFE
          l.lock();
#endif
          m_nextFillSlot.incWrap();
          return true;
        }
      }

#if ADD_TASK_THREAD_SAFE
      l.lock();
#endif
      m_nextFillSlot = m_tasks.grow();
    }
    return false;
  }

  bool processTasksOnMainThread();

  void shutdown( bool block );
  void waitForThreads();

  int numThreads() const { return (int)m_threads.size(); }

private:

  using TaskIterator = ChunkedTaskQueue::Iterator;

  // members
  std::string              m_poolName;
  std::atomic_bool         m_exitThreads{ false };
  std::vector<std::thread> m_threads;
  ChunkedTaskQueue         m_tasks;
  TaskIterator             m_nextFillSlot = m_tasks.begin();
#if ADD_TASK_THREAD_SAFE
  std::mutex               m_nextFillSlotMutex;
#endif
  std::mutex               m_idleMutex;
  std::atomic_uint         m_waitingThreads{ 0 };

  // internal functions
  void         threadProc    ( int threadId );
  static bool  checkTaskReady( int threadId, CBarrierVec& barriers, TaskFunc readyCheck, void* taskParam );
  TaskIterator findNextTask  ( int threadId, TaskIterator startSearch );
  bool         processTask   ( int threadId, Slot& task );
};

#endif   // NOMALLOCTHREADPOOL_H
