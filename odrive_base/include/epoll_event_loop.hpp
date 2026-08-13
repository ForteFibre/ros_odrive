#ifndef EPOLL_EVENT_LOOP_HPP
#define EPOLL_EVENT_LOOP_HPP

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <atomic>
#include <functional>
#include <iostream>
#include <mutex>
#include <vector>

using std::placeholders::_1;
using Callback = std::function<void(uint32_t)>;

class EpollEventLoop
{
public:
  struct EventContext
  {
    int fd;
    Callback callback;
    bool active{true};
  };

  using EvtId = EventContext *;

  EpollEventLoop();

  ~EpollEventLoop();

  bool register_event(EvtId * p_evt, int fd, uint32_t events, const Callback & callback);

  bool deregister_event(EvtId evt);

  bool has_events() const { return n_events_.load() != 0; }

  bool run_until_empty();

  void drop_event(EvtId evt);

private:
  static constexpr size_t kMaxEventsPerIteration = 16;
  int epollfd = -1;
  int wake_fd_ = -1;
  std::atomic_size_t n_events_{0};
  std::recursive_mutex registration_mutex_;
  std::vector<EventContext *> retired_events_;
  EventContext wake_context_{};
  int n_triggered_events_ = 0;
  struct epoll_event triggered_events_[kMaxEventsPerIteration];
};

class EpollEvent
{
public:
  bool init(EpollEventLoop * event_loop, const Callback & callback);
  void deinit();

  bool set();

private:
  void on_trigger(uint32_t event_id);

  EpollEventLoop * event_loop_;
  int fd_ = -1;
  EpollEventLoop::EventContext * evt_;
  Callback callback_;
};

#endif  // EPOLL_EVENT_LOOP_HPP
