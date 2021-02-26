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

#include "NoMallocThreadPool.h"

#include <chrono>

#if __linux
# include <pthread.h>
#endif

// block threads after busy-waiting this long
const static auto BUSY_WAIT_TIME = [] {
  const char* env = getenv( "BUSY_WAIT_TIME" );
  if( env )
    return std::chrono::milliseconds( atoi( env ) );
  return std::chrono::milliseconds( 1 );
}();

class ScopeIncDecCounter
{
  std::atomic_uint& m_cntr;

public:
  explicit ScopeIncDecCounter( std::atomic_uint& counter ): m_cntr( counter ) { m_cntr.fetch_add( 1, std::memory_order_relaxed ); }
  ~ScopeIncDecCounter()                                                       { m_cntr.fetch_sub( 1, std::memory_order_relaxed ); }

  ScopeIncDecCounter( const ScopeIncDecCounter& ) = delete;
  ScopeIncDecCounter( ScopeIncDecCounter&& )      = delete;
};

// ---------------------------------------------------------------------------
// Thread Pool
// ---------------------------------------------------------------------------

NoMallocThreadPool::NoMallocThreadPool( int numThreads, const char* threadPoolName )
  : m_poolName( threadPoolName )
  , m_threads ( numThreads < 0 ? std::thread::hardware_concurrency() : numThreads )
{
  int tid = 0;
  for( auto& t: m_threads )
  {
    t = std::thread( &NoMallocThreadPool::threadProc, this, tid++ );
  }
}

NoMallocThreadPool::~NoMallocThreadPool()
{
  m_exitThreads = true;

  waitForThreads();
}

bool NoMallocThreadPool::processTasksOnMainThread()
{
  CHECK( m_threads.size() != 0, "should not be used with multiple threads" );

  auto taskIt = findNextTask( 0, m_tasks.begin() );
  while( taskIt.isValid() )
  {
    processTask( 0, *taskIt );
    taskIt = findNextTask( 0, taskIt );
  }

  // return true if all done (-> false if some tasks blocked due to barriers)
  return std::all_of( m_tasks.begin(), m_tasks.end(), []( Slot& t ) { return t.state == FREE; } );
}

void NoMallocThreadPool::shutdown( bool block )
{
  m_exitThreads = true;
  if( block )
  {
    waitForThreads();
  }
}

void NoMallocThreadPool::waitForThreads()
{
  for( auto& t: m_threads )
  {
    if( t.joinable() )
      t.join();
  }
}

void NoMallocThreadPool::threadProc( int threadId )
{
#if __linux
  if( !m_poolName.empty() )
  {
    std::string threadName( m_poolName + std::to_string( threadId ) );
    pthread_setname_np( pthread_self(), threadName.c_str() );
  }
#endif

  auto nextTaskIt = m_tasks.begin();
  while( !m_exitThreads )
  {
    auto taskIt = findNextTask( threadId, nextTaskIt );
    if( !taskIt.isValid() )
    {
      std::unique_lock<std::mutex> l( m_idleMutex, std::defer_lock );

      ITT_TASKSTART( itt_domain_thrd, itt_handle_TPspinWait );
      ScopeIncDecCounter cntr( m_waitingThreads );

      const auto startWait = std::chrono::steady_clock::now();
      while( !m_exitThreads )
      {
        taskIt = findNextTask( threadId, nextTaskIt );
        if( taskIt.isValid() || m_exitThreads )
        {
          break;
        }

        if( !l.owns_lock()
            && m_waitingThreads.load( std::memory_order_relaxed ) > 1
            && ( BUSY_WAIT_TIME.count() == 0 || std::chrono::steady_clock::now() - startWait > BUSY_WAIT_TIME )
            && !m_exitThreads )
        {
          ITT_TASKSTART( itt_domain_thrd, itt_handle_TPblocked );
          l.lock();
          ITT_TASKEND( itt_domain_thrd, itt_handle_TPblocked );
        }
        else
        {
          std::this_thread::yield();
        }
      }

      ITT_TASKEND( itt_domain_thrd, itt_handle_TPspinWait );
    }
    if( m_exitThreads )
    {
      return;
    }

    processTask( threadId, *taskIt );

    nextTaskIt = taskIt;
    nextTaskIt.incWrap();
  }
}

bool NoMallocThreadPool::checkTaskReady( int threadId, CBarrierVec& barriers, NoMallocThreadPool::TaskFunc readyCheck, void* taskParam )
{
  if( !barriers.empty() && std::any_of( barriers.cbegin(), barriers.cend(), []( const Barrier* b ) { return b && b->isBlocked(); } ) )
  {
    return false;
  }
  barriers.clear();   // clear barriers, so we don't need to check them on the next try (we assume they won't get locked again)

  if( readyCheck && readyCheck( threadId, taskParam ) == false )
  {
    return false;
  }

  return true;
}

NoMallocThreadPool::TaskIterator NoMallocThreadPool::findNextTask( int threadId, TaskIterator startSearch )
{
  if( !startSearch.isValid() )
  {
    startSearch = m_tasks.begin();
  }
  bool first = true;
  for( auto it = startSearch; it != startSearch || first; it.incWrap() )
  {
    first = false;

    Slot& task = *it;
    auto expected = WAITING;
    if( task.state == expected && task.state.compare_exchange_strong( expected, RUNNING ) )
    {
      if( checkTaskReady( threadId, task.barriers, task.readyCheck, task.param ) )
      {
        return it;
      }

      // reschedule
      task.state = WAITING;
    }
  }
  return {};
}

bool NoMallocThreadPool::processTask( int threadId, NoMallocThreadPool::Slot& task )
{
  const bool success = task.func( threadId, task.param );
  if( !success )
  {
    task.state = WAITING;
    return false;
  }

  if( task.done != nullptr )
  {
    task.done->unlock();
  }
  if( task.counter != nullptr )
  {
    --(*task.counter);
  }

  task.state = FREE;

  return true;
}

// ---------------------------------------------------------------------------
// Chunked Task Queue
// ---------------------------------------------------------------------------

NoMallocThreadPool::ChunkedTaskQueue::~ChunkedTaskQueue()
{
  Chunk* next = m_firstChunk.m_next;
  while( next )
  {
    Chunk* curr = next;
    next = curr->m_next;
    delete curr;
  }
}

NoMallocThreadPool::ChunkedTaskQueue::Iterator NoMallocThreadPool::ChunkedTaskQueue::grow()
{
  std::unique_lock<std::mutex> l( m_resizeMutex );   // prevent concurrent growth of the queue. Read access while growing is no problem

  m_lastChunk->m_next = new Chunk( &m_firstChunk );
  m_lastChunk         = m_lastChunk->m_next;

  return Iterator{ &m_lastChunk->m_slots.front(), m_lastChunk };
}

NoMallocThreadPool::ChunkedTaskQueue::Iterator& NoMallocThreadPool::ChunkedTaskQueue::Iterator::operator++()
{
  CHECKD( m_slot == nullptr, "incrementing invalid iterator" );
  CHECKD( m_chunk == nullptr, "incrementing invalid iterator" );

  if( m_slot != &m_chunk->m_slots.back() )
  {
    ++m_slot;
  }
  else
  {
    m_chunk = m_chunk->m_next;
    if( m_chunk )
    {
      m_slot = &m_chunk->m_slots.front();
    }
    else
    {
      m_slot = nullptr;
    }
  }
  return *this;
}

NoMallocThreadPool::ChunkedTaskQueue::Iterator& NoMallocThreadPool::ChunkedTaskQueue::Iterator::incWrap()
{
  CHECKD( m_slot == nullptr, "incrementing invalid iterator" );
  CHECKD( m_chunk == nullptr, "incrementing invalid iterator" );

  if( m_slot != &m_chunk->m_slots.back() )
  {
    ++m_slot;
  }
  else
  {
    if( m_chunk->m_next )
    {
      m_chunk = m_chunk->m_next;
    }
    else
    {
      m_chunk = &m_chunk->m_firstChunk;
    }
    m_slot = &m_chunk->m_slots.front();
  }
  return *this;
}
