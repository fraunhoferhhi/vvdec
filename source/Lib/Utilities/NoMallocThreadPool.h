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
#define THREAD_POOL_HANDLE_EXCEPTIONS 1


// ---------------------------------------------------------------------------
// Synchronization tools
// ---------------------------------------------------------------------------

struct Barrier
{
  virtual void unlock()
  {
#if THREAD_POOL_HANDLE_EXCEPTIONS
    checkAndRethrowException();
#endif   // THREAD_POOL_HANDLE_EXCEPTIONS

    m_lockState.store( false );
  }

  virtual void lock()
  {
#if THREAD_POOL_HANDLE_EXCEPTIONS
    checkAndRethrowException();
#endif   // THREAD_POOL_HANDLE_EXCEPTIONS

    m_lockState.store( true );
  }

  bool isBlocked() const
  {
#if THREAD_POOL_HANDLE_EXCEPTIONS
    checkAndRethrowException();
#endif   // THREAD_POOL_HANDLE_EXCEPTIONS

    return m_lockState;
  }

#if THREAD_POOL_HANDLE_EXCEPTIONS
  virtual void setException( std::exception_ptr e )
  {
    std::lock_guard<std::mutex> l( s_exceptionLock );
    if( m_hasException )
    {
      CHECK( m_exception == nullptr, "no exception currently stored, but flag is set" );
      // exception is already set -> no-op
      return;
    }
    m_exception    = e;
    m_hasException = true;
  }

  virtual void clearException()
  {
    if( m_hasException )
    {
      std::lock_guard<std::mutex> l( s_exceptionLock );
      m_hasException = false;
      m_exception    = nullptr;
    }
  }

  const std::exception_ptr getException() const
  {
    if( !m_hasException )
    {
      return nullptr;
    }

    std::lock_guard<std::mutex> l( s_exceptionLock );
    return m_exception;
  }

  bool hasException() const { return m_hasException; }

  inline void checkAndRethrowException() const
  {
    if( !m_hasException )
    {
      return;
    }

    std::lock_guard<std::mutex> l( s_exceptionLock );
    if( m_hasException )
    {
      CHECK( m_exception == nullptr, "no exception currently stored, but flag is set" );
      std::rethrow_exception( m_exception );
    }
  }
#endif   // THREAD_POOL_HANDLE_EXCEPTIONS

  Barrier()  = default;
  virtual ~Barrier() = default;
  explicit Barrier( bool locked ) : m_lockState( locked ) {}

  Barrier( const Barrier & ) = delete;
  Barrier( Barrier && )      = delete;

  Barrier& operator=( const Barrier & ) = delete;
  Barrier& operator=( Barrier && )      = delete;

private:
  std::atomic_bool   m_lockState{ true };
#if THREAD_POOL_HANDLE_EXCEPTIONS
  std::atomic_bool   m_hasException{ false };
  std::exception_ptr m_exception;
  static std::mutex  s_exceptionLock;   // we use one shared mutex for all barriers here. It is only involved, when exceptions actually happen, so there should
                                        // be no contention during normal operations
#endif
};

struct BlockingBarrier: public Barrier
{
  void unlock() override
  {
    std::unique_lock<std::mutex> l( m_lock );
    Barrier::unlock();
    m_cond.notify_all();
  }

  void lock() override
  {
    std::unique_lock<std::mutex> l( m_lock );
    Barrier::lock();
  }

  void wait() const
  {
    std::unique_lock<std::mutex> l( m_lock );
    m_cond.wait( l, [=] { return !Barrier::isBlocked(); } );
  }

#if THREAD_POOL_HANDLE_EXCEPTIONS
  void setException( std::exception_ptr e ) override
  {
    std::unique_lock<std::mutex> l( m_lock );
    Barrier::setException( e );
    m_cond.notify_all();
  }

  void clearException() override
  {
    std::unique_lock<std::mutex> l( m_lock );
    Barrier::clearException();
  }
#endif

  BlockingBarrier()  = default;
  ~BlockingBarrier() { std::unique_lock<std::mutex> l( m_lock ); } // ensure all threads have unlocked the mutex, when we start destruction

  BlockingBarrier( const BlockingBarrier& ) = delete;
  BlockingBarrier( BlockingBarrier&& )      = delete;

  BlockingBarrier& operator=( const BlockingBarrier& ) = delete;
  BlockingBarrier& operator=( BlockingBarrier&& ) = delete;

private:
  mutable std::condition_variable m_cond;
  mutable std::mutex              m_lock;
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
#if THREAD_POOL_HANDLE_EXCEPTIONS
    m_done.checkAndRethrowException();
#endif   // THREAD_POOL_HANDLE_EXCEPTIONS
    return 0 != m_count;
  }

  void wait() const
  {
    std::unique_lock<std::mutex> l( m_lock );
#if THREAD_POOL_HANDLE_EXCEPTIONS
    m_cond.wait( l, [=] { return m_count == 0 || m_done.hasException(); } );
    m_done.checkAndRethrowException();
#else
    m_cond.wait( l, [=] { return m_count == 0; } );
#endif   // THREAD_POOL_HANDLE_EXCEPTIONS
  }

#if THREAD_POOL_HANDLE_EXCEPTIONS
  void setException( std::exception_ptr e )
  {
    std::unique_lock<std::mutex> l( m_lock );
    m_done.setException( e );
    m_cond.notify_all();
  }

  void clearException()
  {
    std::unique_lock<std::mutex> l( m_lock );
    m_done.clearException();
  }

  bool             hasException() const { return m_done.hasException(); }
  const std::exception_ptr getException() const { return m_done.getException(); }
#endif   // THREAD_POOL_HANDLE_EXCEPTIONS

  WaitCounter() = default;
  ~WaitCounter() { std::unique_lock<std::mutex> l( m_lock ); }   // ensure all threads have unlocked the mutex, when we start destruction

  WaitCounter( const WaitCounter & ) = delete;
  WaitCounter( WaitCounter && )      = delete;

  WaitCounter &operator=( const WaitCounter & ) = delete;
  WaitCounter &operator=( WaitCounter && )      = delete;

  const Barrier* donePtr() const { return &m_done; }

private:
  mutable std::condition_variable m_cond;
  mutable std::mutex              m_lock;
  unsigned int                    m_count = 0;
  Barrier                         m_done{ false };
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
      // in the single threaded case try to exectute the task directly
      if( bypassTaskQueue( (TaskFunc)func, param, counter, done, barriers, (TaskFunc)readyCheck ) )
      {
        return true;
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
#if THREAD_POOL_HANDLE_EXCEPTIONS
  struct TaskException;
#endif

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
  void         threadProc     ( int threadId );
  static bool  checkTaskReady ( int threadId, CBarrierVec& barriers, TaskFunc readyCheck, void* taskParam );
  TaskIterator findNextTask   ( int threadId, TaskIterator startSearch );
  static bool  processTask    ( int threadId, Slot& task );
  bool         bypassTaskQueue( TaskFunc func, void* param, WaitCounter* counter, Barrier* done, CBarrierVec& barriers, TaskFunc readyCheck );
#if THREAD_POOL_HANDLE_EXCEPTIONS
  static void handleTaskException( const std::exception_ptr e, Barrier* done, WaitCounter* counter, std::atomic<TaskState>* slot_state );
#endif
};

#endif   // NOMALLOCTHREADPOOL_H
