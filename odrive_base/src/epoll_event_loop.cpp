#include "epoll_event_loop.hpp"

EpollEventLoop::EpollEventLoop()
{
  epollfd = epoll_create1(0);
  wake_fd_ = eventfd(0, EFD_NONBLOCK);
  wake_context_ = EventContext{wake_fd_, [](uint32_t) {}, true};
  struct epoll_event ev {};
  ev.events = EPOLLIN;
  ev.data.ptr = &wake_context_;
  if (epollfd >= 0 && wake_fd_ >= 0) {
    epoll_ctl(epollfd, EPOLL_CTL_ADD, wake_fd_, &ev);
  }
}

EpollEventLoop::~EpollEventLoop()
{
  for (auto * event : retired_events_) {
    delete event;
  }
  if (wake_fd_ >= 0) {
    close(wake_fd_);
  }
  close(epollfd);
}

bool EpollEventLoop::register_event(
  EvtId * p_evt, int fd, uint32_t events, const Callback & callback)
{
  std::lock_guard<std::recursive_mutex> guard(registration_mutex_);
  EventContext * ctx = new EventContext{fd, callback, true};
  struct epoll_event ev {};
  ev.events = events;
  ev.data.ptr = ctx;
  if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &ev) == -1) {
    delete ctx;  // Cleanup the dynamically allocated EventContext in case of failure
    return false;
  }

  if (p_evt) *p_evt = ctx;

  n_events_.fetch_add(1);
  return true;
}

bool EpollEventLoop::deregister_event(EvtId evt)
{
  std::lock_guard<std::recursive_mutex> guard(registration_mutex_);
  if (evt == nullptr) return false;
  if (epoll_ctl(epollfd, EPOLL_CTL_DEL, evt->fd, nullptr) == -1) return false;
  evt->active = false;
  // A concurrent epoll_wait may already have copied evt into its returned
  // array. Keep the tiny context alive until the joined loop is destroyed.
  retired_events_.push_back(evt);
  n_events_.fetch_sub(1);
  const uint64_t wake_value = 1;
  if (wake_fd_ >= 0) {
    (void)write(wake_fd_, &wake_value, sizeof(wake_value));
  }
  return true;
}

bool EpollEventLoop::run_until_empty()
{
  while (has_events()) {
    n_triggered_events_ = epoll_wait(epollfd, triggered_events_, kMaxEventsPerIteration, -1);
    if (n_triggered_events_ == -1) return false;
    for (int i = 0; i < n_triggered_events_; ++i) {
      EventContext * handler = static_cast<EventContext *>(triggered_events_[i].data.ptr);
      if (handler == &wake_context_) {
        uint64_t wake_value;
        while (read(wake_fd_, &wake_value, sizeof(wake_value)) == sizeof(wake_value)) {}
      } else if (handler != nullptr) {
        std::lock_guard<std::recursive_mutex> guard(registration_mutex_);
        if (handler->active) {
          handler->callback(triggered_events_[i].events);
        }
      }
    }
  }
  return true;
}

void EpollEventLoop::drop_event(EvtId evt)
{
  for (int i = 0; i < n_triggered_events_; ++i) {
    if (reinterpret_cast<EventContext *>(triggered_events_[i].data.ptr) == evt) {
      triggered_events_[i].data.ptr = nullptr;
    }
  }
}

bool EpollEvent::init(EpollEventLoop * event_loop, const Callback & callback)
{
  event_loop_ = event_loop;
  callback_ = callback;

  fd_ = eventfd(0, 0);
  if (fd_ < 0) return false;

  if (!event_loop->register_event(
        &evt_, fd_, EPOLLIN, std::bind(&EpollEvent::on_trigger, this, _1))) {
    close(fd_);
    std::cerr << "Failed to register event" << std::endl;
    return false;
  }

  return true;
}

void EpollEvent::deinit()
{
  event_loop_->deregister_event(evt_);
  close(fd_);
  fd_ = -1;
}

bool EpollEvent::set()
{
  const uint64_t val = 1;
  if (write(fd_, &val, sizeof(val)) != sizeof(val)) return false;
  return true;
}

void EpollEvent::on_trigger(uint32_t event_id)
{
  uint64_t val;
  if (read(fd_, &val, sizeof(val)) != sizeof(val)) {
    std::cerr << "Failed to read eventfd" << std::endl;
    return;
  }

  callback_(event_id);
}
