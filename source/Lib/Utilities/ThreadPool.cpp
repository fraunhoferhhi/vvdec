/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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
using namespace std::chrono_literals;

std::mutex Barrier::s_exceptionLock{};

// block threads after busy-waiting this long
const static auto VVDEC_BUSY_WAIT_TIME_MIN = [] {
  const char* env = getenv( "VVDEC_BUSY_WAIT_TIME_MIN" );
  if( env )
    return std::chrono::microseconds( int( atof( env ) * 1000 ) );
  return std::chrono::microseconds( 1ms );
}();

// block last waiting thread after this time
const static auto VVDEC_BUSY_WAIT_TIME_MAX = [] {
  const char* env = getenv( "VVDEC_BUSY_WAIT_TIME_MAX" );
  if( env )
    return atoi( env ) * 1ms;
  return 5ms;
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
  CLASS_COPY_MOVE_DELETE( ScopeIncDecCounter )
};

// ---------------------------------------------------------------------------
// Thread Pool
// ---------------------------------------------------------------------------

ThreadPool::ThreadPool( int numThreads, const char* threadPoolName )
  : m_poolName( threadPoolName )
  , m_threads( numThreads < 0 ? std::thread::hardware_concurrency() : numThreads )
  , m_poolPause( m_threads.size() )
{
  int tid = 0;
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
  CHECK_FATAL( m_threads.size() != 0, "should not be used with multiple threads" );

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
  m_poolPause.unpauseIfPaused( m_poolPause.acquireLock() );

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
      if( !taskIt.isValid() )   // immediately try again without any delay
      {
        taskIt = findNextTask( threadId, nextTaskIt );
      }
      if( !taskIt.isValid() )   // still nothing found, go into idle loop searching for more tasks
      {
        ITT_TASKSTART( itt_domain_thrd, itt_handle_TPspinWait );

        std::unique_lock<std::mutex> idleLock( m_idleMutex, std::defer_lock );

        const auto startWait = std::chrono::steady_clock::now();
        bool       didBlock  = false;   // if the previous iteration did block we don't want to yield in this iteration
        while( !m_exitThreads )
        {
          if( !didBlock )
          {
            std::this_thread::yield();
          }
          didBlock = false;

          taskIt = findNextTask( threadId, nextTaskIt );
          if( taskIt.isValid() || m_exitThreads.load( std::memory_order_relaxed ) )
          {
            break;
          }

          if( !idleLock.owns_lock() )
          {
            if( VVDEC_BUSY_WAIT_TIME_MIN.count() == 0 || std::chrono::steady_clock::now() - startWait > VVDEC_BUSY_WAIT_TIME_MIN )
            {
              ITT_TASKSTART( itt_domain_thrd, itt_handle_TPblocked );
              ScopeIncDecCounter cntr( m_poolPause.m_waitingForLockThreads );
              idleLock.lock();
              didBlock = true;
              ITT_TASKEND( itt_domain_thrd, itt_handle_TPblocked );
            }
          }
          else if( std::chrono::steady_clock::now() - startWait > VVDEC_BUSY_WAIT_TIME_MAX )
          {
#if THREAD_POOL_TASK_NAMES
            printWaitingTasks();
#endif
            didBlock = m_poolPause.pauseIfAllOtherThreadsWaiting(
              [&]
              {
                taskIt = findNextTask( threadId, nextTaskIt );
                return taskIt.isValid() || m_exitThreads;
              } );
            if( taskIt.isValid() )
            {
              break;
            }
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
  std::lock_guard<std::mutex> l( m_resizeMutex );   // prevent concurrent growth of the queue. Read access while growing is no problem

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

void ThreadPool::PoolPause::unpauseIfPaused( std::unique_lock<std::mutex> lockOwnership )
{
  CHECKD( lockOwnership.mutex() != &m_allThreadsWaitingMutex, "wrong mutex passed into ThreadPool::PoolPause::unpauseIfPaused()" );
  CHECKD( !lockOwnership.owns_lock(), "lock passed into ThreadPool::PoolPause::unpauseIfPaused() does not own lock" );
  // All threads may be sleeping. If so, wake up.
  m_allThreadsWaiting = false;
  m_allThreadsWaitingCV.notify_all();
}

template<typename Predicate>
bool ThreadPool::PoolPause::pauseIfAllOtherThreadsWaiting( Predicate predicate )
{
  if( m_nrThreads == 0 )
  {
    return false;
  }
  const auto nrWaiting = m_waitingForLockThreads.load( std::memory_order_relaxed );
  if( nrWaiting < m_nrThreads - 1 )
  {
    return false;
  }

  // All threads are waiting. This (current) threads is the one which locked `idleLock`. All
  // other threads are waiting in the above condition for `idleLock.lock();`.
  // The only way how more work for the threads can come in is if addBarrierTask is called
  // or if the thread pool is closed or destroyed.
  std::unique_lock<std::mutex> lock( m_allThreadsWaitingMutex );
  m_allThreadsWaiting = true;
  m_allThreadsWaitingCV.wait( lock, [this, &predicate] { return !m_allThreadsWaiting || predicate(); } );
  return true;
}

}   // namespace vvdec
