#include "epoll_event_loop.hpp"

#include <chrono>
#include <future>
#include <thread>

#include <gtest/gtest.h>

TEST(EpollEventLoop, DeinitWakesAndAllowsJoin)
{
  EpollEventLoop event_loop;
  EpollEvent event;
  std::promise<void> callback_called;
  auto callback_future = callback_called.get_future();

  ASSERT_TRUE(event.init(
      &event_loop,
      [&callback_called](uint32_t) { callback_called.set_value(); }));
  std::thread loop_thread([&event_loop]() { event_loop.run_until_empty(); });

  ASSERT_TRUE(event.set());
  EXPECT_EQ(callback_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);

  event.deinit();
  loop_thread.join();
}

