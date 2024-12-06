#ifndef _THREADSAFE_QUEUE_H_
#define _THREADSAFE_QUEUE_H_

#include <memory>
#include <mutex>
#include <condition_variable>

template<typename T>
class threadsafe_queue
{
 public:
  threadsafe_queue() :
      head(new node), tail(head.get()) {}

  std::shared_ptr<T> try_pop()
  {
    std::unique_ptr<node> old_head = try_pop_head();
    return old_head ? old_head->data : std::shared_ptr<T>();
  }

  bool try_pop(T &value)
  {
    std::unique_ptr<node> const old_head = try_pop_head(value);
    return old_head.get();
  }

  std::shared_ptr<T> wait_and_pop()
  {
    std::unique_ptr<node> const old_head = wait_pop_head();
    return old_head->data;
  }

  void wait_and_pop(T &value)
  {
    std::unique_ptr<node> const old_head = wait_pop_head(value);
  }

  void push(T new_value)
  {
    std::shared_ptr<T> new_data(std::make_shared<T>(std::move(new_value)));
    std::unique_ptr<node> p(new node);
    {
      std::lock_guard<std::mutex> tail_lock(tail_mutex);
      tail->data = new_data;
      node *const new_tail = p.get();
      tail->next = std::move(p);
      tail = new_tail;
    }
    data_cond.notify_one();
  }

  bool empty()
  {
    std::lock_guard<std::mutex> head_lock(head_mutex);
    return (head.get() == get_tail());
  }

  threadsafe_queue(const threadsafe_queue &) = delete;
  threadsafe_queue &operator=(const threadsafe_queue &) = delete;

 private:
  struct node
  {
    std::shared_ptr<T> data;
    std::unique_ptr<node> next;
  };

  // lock tail mutex and return tail node
  node *get_tail()
  {
    std::lock_guard<std::mutex> tail_lock(tail_mutex);
    return tail;
  }

  // pop head node from queue, return old head node
  std::unique_ptr<node> pop_head()
  {
    std::unique_ptr<node> old_head = std::move(head);
    head = std::move(old_head->next);
    return old_head;
  }

  // wait for data, return std::unique_lock<std::mutex> head_lock
  std::unique_lock<std::mutex> wait_for_data()
  {
    std::unique_lock<std::mutex> head_lock(head_mutex);
    // wait until not empty
    data_cond.wait(head_lock, [&] { return head.get() != get_tail(); });
    return std::move(head_lock);
  }

  std::unique_ptr<node> wait_pop_head()
  {
    std::unique_lock<std::mutex> head_lock(wait_for_data());
    return pop_head();
  }

  std::unique_ptr<node> wait_pop_head(T& value)
  {
    std::unique_lock<std::mutex> head_lock(wait_for_data());
    value = std::move(*head->data);
    return pop_head();
  }

  std::unique_ptr<node> try_pop_head()
  {
    std::lock_guard<std::mutex> head_lock(head_mutex);
    if (head.get() == get_tail())
    {
      return std::unique_ptr<node>();
    }
    return pop_head();
  }

  std::unique_ptr<node> try_pop_head(T &value)
  {
    std::lock_guard<std::mutex> head_lock(head_mutex);
    if (head.get() == get_tail())
    {
      return std::unique_ptr<node>();
    }
    value = std::move(*head->data);
    return pop_head();
  }

  std::mutex head_mutex; // head mutex
  std::unique_ptr<node> head; // head node
  std::mutex tail_mutex; // tail mutex
  node *tail; // tail node
  std::condition_variable data_cond; // condition variable
};

#endif