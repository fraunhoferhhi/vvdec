/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

     * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

     * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

#include "ThreadPool.h"

#include <chrono>

#if __linux
# include <pthread.h>
#endif

namespace vvdec
{

std::mutex Barrier::s_exceptionLock{};

// block threads after busy-waiting this long
const static auto BUSY_WAIT_TIME = [] {
  const char* env = getenv( "BUSY_WAIT_TIME" );
  if( env )
    return std::chrono::milliseconds( atoi( env ) );
  return std::chrono::milliseconds( 1 );
}();

struct ThreadPool::TaskException : public std::exception
{
  explicit TaskException( std::exception_ptr e, ThreadPool::Slot& task )
    : m_originalException( e )
    , m_task( task )
  {}
  std::exception_ptr m_originalException;
  ThreadPool::Slot&  m_task;
};

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

ThreadPool::ThreadPool( int numThreads, const char* threadPoolName )
  : m_poolName( threadPoolName )
  , m_threads ( numThreads < 0 ? std::thread::hardware_concurrency() : numThreads )
{
  int tid = 0;
  m_poolPause.setNrThreads(m_threads.size());
  for( auto& t: m_threads )
  {
    t = std::thread( &ThreadPool::threadProc, this, tid++ );
  }
}

ThreadPool::~ThreadPool()
{
  m_exitThreads = true;

  waitForThreads();
}

bool ThreadPool::processTasksOnMainThread()
{
  CHECK( m_threads.size() != 0, "should not be used with multiple threads" );

  TaskIterator taskIt;
  while( true )
  {
    try
    {
      if( !taskIt.isValid() )
      {
        taskIt = findNextTask( 0, m_tasks.begin() );
      }
      else
      {
        taskIt = findNextTask( 0, taskIt );
      }

      if( !taskIt.isValid() )
      {
        break;
      }
      processTask( 0, *taskIt );
    }
    catch( TaskException& e )
    {
      handleTaskException( e.m_originalException, e.m_task.done, e.m_task.counter, &e.m_task.state );
    }
  }

  // return true if all done (-> false if some tasks blocked due to barriers)
  return std::all_of( m_tasks.begin(), m_tasks.end(), []( Slot& t ) { return t.state == FREE; } );
}

void ThreadPool::shutdown( bool block )
{
  m_exitThreads = true;
  if( block )
  {
    waitForThreads();
  }
}

void ThreadPool::waitForThreads()
{
  m_poolPause.unpauseIfPaused();

  for( auto& t: m_threads )
  {
    if( t.joinable() )
      t.join();
  }
}

void ThreadPool::checkAndThrowThreadPoolException()
{
  if( !m_exceptionFlag.load() )
  {
    return;
  }

  msg( WARNING, "ThreadPool is in exception state." );

  std::exception_ptr tmp = m_threadPoolException;
  m_threadPoolException  = tmp;
  m_exceptionFlag.store( false );

  std::rethrow_exception( tmp );
}

void ThreadPool::threadProc( int threadId )
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
    try
    {
      auto taskIt = findNextTask( threadId, nextTaskIt );
      if( !taskIt.isValid() )
      {
        std::unique_lock<std::mutex> l( m_idleMutex, std::defer_lock );

        ITT_TASKSTART( itt_domain_thrd, itt_handle_TPspinWait );

        const auto startWait = std::chrono::steady_clock::now();
        while( !m_exitThreads )
        {
          taskIt = findNextTask( threadId, nextTaskIt );
          if( taskIt.isValid() || m_exitThreads )
          {
            break;
          }

          if( !l.owns_lock()
              && ( BUSY_WAIT_TIME.count() == 0 || std::chrono::steady_clock::now() - startWait > BUSY_WAIT_TIME )
              && !m_exitThreads )
          {
            ITT_TASKSTART( itt_domain_thrd, itt_handle_TPblocked );
            ScopeIncDecCounter cntr( m_poolPause.m_waitingForLockThreads );
            l.lock();
            ITT_TASKEND( itt_domain_thrd, itt_handle_TPblocked );
          }
          else if (std::chrono::steady_clock::now() - startWait > std::chrono::milliseconds(500))
          {
#if THREAD_POOL_TASK_NAMES
            printWaitingTasks();
#endif
            m_poolPause.pauseIfAllOtherThreadsWaiting();
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
    catch( TaskException& e )
    {
      handleTaskException( e.m_originalException, e.m_task.done, e.m_task.counter, &e.m_task.state );
    }
    catch( std::exception& e )
    {
      msg( ERROR, "ERROR: Caught unexpected exception from within the thread pool: %s", e.what() );

      if( m_exceptionFlag.exchange( true ) )
      {
        msg( ERROR, "ERROR: Another exception has already happend in the thread pool, but we can only store one." );
        return;
      }
      m_threadPoolException = std::current_exception();
      return;
    }
  }
}

bool ThreadPool::checkTaskReady( int threadId, CBarrierVec& barriers, ThreadPool::TaskFunc readyCheck, void* taskParam )
{
  if( !barriers.empty() )
  {
    // don't break early, because isBlocked() also checks exception state
    if( std::count_if( barriers.cbegin(), barriers.cend(), []( const Barrier* b ) { return b && b->isBlocked(); } ) )
    {
      return false;
    }
    barriers.clear();
  }

  if( readyCheck && readyCheck( threadId, taskParam ) == false )
  {
    return false;
  }

  return true;
}

ThreadPool::TaskIterator ThreadPool::findNextTask( int threadId, TaskIterator startSearch )
{
  if( !startSearch.isValid() )
  {
    startSearch = m_tasks.begin();
  }
  bool first = true;
  for( auto it = startSearch; it != startSearch || first; it.incWrap() )
  {
    first = false;
    try
    {
      Slot& task     = *it;
      auto  expected = WAITING;
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
    catch( ... )
    {
      throw TaskException( std::current_exception(), *it );
    }
  }
  return {};
}

bool ThreadPool::processTask( int threadId, ThreadPool::Slot& task )
{
  try
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
  }
  catch( ... )
  {
    throw TaskException( std::current_exception(), task );
  }

  task.state = FREE;

  return true;
}

bool ThreadPool::bypassTaskQueue( TaskFunc func, void* param, WaitCounter* counter, Barrier* done, CBarrierVec& barriers, TaskFunc readyCheck )
{
  CHECKD( numThreads() > 0, "the task queue should only be bypassed, when running single-threaded." );
  try
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
  catch( ... )
  {
    handleTaskException( std::current_exception(), done, counter, nullptr );
  }

  // direct execution of the task failed
  return false;
}

void ThreadPool::handleTaskException( const std::exception_ptr e, Barrier* done, WaitCounter* counter, std::atomic<TaskState>* slot_state )
{
  if( done != nullptr )
  {
    done->setException( e );
  }
  if( counter != nullptr )
  {
    counter->setException( e );
    // Barrier::unlock() in the decrement operator throws, when the counter reaches zero, so we catch it here
    try
    {
      --( *counter );
    }
    catch( ... )
    {
    }
  }

  if( slot_state != nullptr )
  {
    *slot_state = FREE;
  }
}

#if THREAD_POOL_TASK_NAMES
void ThreadPool::printWaitingTasks()
{
  std::cerr << "Waiting tasks:" << std::endl;
  int count = 0;
  for( auto& t: m_tasks )
  {
    if( t.state == WAITING )
    {
      ++count;
      std::cerr << t.taskName << std::endl;
    }
  }
  std::cerr << std::endl << count << " total tasks waiting" << std::endl;
}
#endif  // THREAD_POOL_TASK_NAMES

// ---------------------------------------------------------------------------
// Chunked Task Queue
// ---------------------------------------------------------------------------

ThreadPool::ChunkedTaskQueue::~ChunkedTaskQueue()
{
  Chunk* next = m_firstChunk.m_next;
  while( next )
  {
    Chunk* curr = next;
    next = curr->m_next;
    delete curr;
  }
}

ThreadPool::ChunkedTaskQueue::Iterator ThreadPool::ChunkedTaskQueue::grow()
{
  std::unique_lock<std::mutex> l( m_resizeMutex );   // prevent concurrent growth of the queue. Read access while growing is no problem

  m_lastChunk->m_next = new Chunk( &m_firstChunk );
  m_lastChunk         = m_lastChunk->m_next;

  return Iterator{ &m_lastChunk->m_slots.front(), m_lastChunk };
}

ThreadPool::ChunkedTaskQueue::Iterator& ThreadPool::ChunkedTaskQueue::Iterator::operator++()
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

ThreadPool::ChunkedTaskQueue::Iterator& ThreadPool::ChunkedTaskQueue::Iterator::incWrap()
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

}
