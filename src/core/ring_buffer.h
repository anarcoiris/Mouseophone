#ifndef MOUSEOFONO_CORE_RING_BUFFER_H
#define MOUSEOFONO_CORE_RING_BUFFER_H

#include <algorithm>
#include <atomic>
#include <cstdint>


namespace mouseofono::core {

/**
 * @brief Single Producer Single Consumer (SPSC) Lock-Free Ring Buffer.
 *
 * @tparam T The type of elements in the buffer.
 * @tparam Capacity Must be a power of 2 for optimization.
 */
template <typename T, uint32_t Capacity = 16384> class RingBuffer {
  static_assert((Capacity & (Capacity - 1)) == 0,
                "Capacity must be a power of 2");

public:
  RingBuffer() : head(0), tail(0) {}

  /**
   * @brief Pushes an item into the buffer.
   * @return true if success, false if buffer is full.
   */
  bool push(const T &item) {
    uint32_t h = head.load(std::memory_order_relaxed);
    uint32_t next = (h + 1) & (Capacity - 1);

    if (next == tail.load(std::memory_order_acquire)) {
      return false; // Buffer full
    }

    buffer[h] = item;
    head.store(next, std::memory_order_release);
    return true;
  }

  /**
   * @brief Pops an item from the buffer.
   * @return true if success, false if buffer is empty.
   */
  bool pop(T &item) {
    uint32_t t = tail.load(std::memory_order_relaxed);

    if (t == head.load(std::memory_order_acquire)) {
      return false; // Buffer empty
    }

    item = buffer[t];
    tail.store((t + 1) & (Capacity - 1), std::memory_order_release);
    return true;
  }

  /**
   * @brief Returns the approximate number of elements currently in the buffer.
   */
  uint32_t size() const {
    uint32_t h = head.load(std::memory_order_acquire);
    uint32_t t = tail.load(std::memory_order_acquire);
    return (h - t) & (Capacity - 1);
  }

  /**
   * @brief Returns true if buffer appears empty (approximate - SPSC safe for
   * consumer).
   */
  bool empty() const {
    return tail.load(std::memory_order_acquire) ==
           head.load(std::memory_order_acquire);
  }

private:
  T buffer[Capacity];
  std::atomic<uint32_t> head;
  std::atomic<uint32_t> tail;
};

} // namespace mouseofono::core

#endif // MOUSEOFONO_CORE_RING_BUFFER_H
