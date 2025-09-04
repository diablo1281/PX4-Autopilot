#pragma once
#include <cstdint>
#include <cstring>

template <size_t N>
class RingBuffer {
public:
    RingBuffer() { clear(); }

    void   clear()               { head_=tail_=size_=0; }
    size_t capacity()     const  { return N; }
    size_t size()         const  { return size_; }
    size_t free_space()   const  { return N - size_; }
    bool   empty()        const  { return size_ == 0; }
    bool   full()         const  { return size_ == N; }

    // Zapis z bufora zewnętrznego; przy allow_overflow nadmiar nadpisuje najstarsze dane.
    size_t write(const uint8_t* src, size_t len, bool allow_overflow = true) {
        if (len > free_space()) {
            if (!allow_overflow) len = free_space();
            else drop(len - free_space());
        }
        size_t first = min_size_t(len, N - tail_);
        memcpy(buf_ + tail_, src, first);
        size_t remain = len - first;
        if (remain) memcpy(buf_, src + first, remain);
        tail_ = (tail_ + len) % N;
        size_ += len;
        return len;
    }

    // Podgląd bez konsumowania (od offsetu od początku danych).
    size_t peek(size_t offset, uint8_t* dst, size_t len) const {
        if (offset >= size_) return 0;
        size_t n = min_size_t(len, size_ - offset);
        size_t idx = (head_ + offset) % N;
        size_t first = min_size_t(n, N - idx);
        memcpy(dst, buf_ + idx, first);
        size_t remain = n - first;
        if (remain) memcpy(dst + first, buf_, remain);
        return n;
    }

    // Zjedz n bajtów.
    void drop(size_t n) {
        n = min_size_t(n, size_);
        head_ = (head_ + n) % N;
        size_ -= n;
    }

    // Skopiuj i zjedz.
    size_t read(uint8_t* dst, size_t len) {
        size_t n = peek(0, dst, len);
        drop(n);
        return n;
    }

    // Szukaj 2‑bajtowego wzorca (nagłówka).
    bool find2(uint8_t b0, uint8_t b1, size_t& out_off) const {
        if (size_ < 2) return false;
        for (size_t i = 0; i + 1 < size_; ++i) {
            size_t i0 = (head_ + i) % N;
            size_t i1 = (i0 + 1) % N;
            if (buf_[i0] == b0 && buf_[i1] == b1) { out_off = i; return true; }
        }
        return false;
    }

private:
    static inline size_t min_size_t(size_t a, size_t b) {
        return (a < b) ? a : b;
    }

    uint8_t buf_[N];
    size_t head_{0}, tail_{0}, size_{0};
};
