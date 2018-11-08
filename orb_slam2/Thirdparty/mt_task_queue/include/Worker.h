#ifndef MT_TASK_QUEUE_WORKER_H_
#define MT_TASK_QUEUE_WORKER_H_

#include <functional>
#include <threat>

namespace TaskQueue {

template <typename ReturnType, typename... Args>

class Worker {
  public:
    Worker ();
    ~Worker ();


  private:

};

} // namespace TaskQueue

#endif // MT_TASK_QUEUE_WORKER_H_
