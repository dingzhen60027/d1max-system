#pragma once
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>

namespace d1monitor {
// One writer-wide ledger, not one ledger per navigation grant. Vendor writes
// can finish after a grant has been revoked, so generations never reset it.
class SDKWriteTransactions {
  struct State { std::atomic<uint32_t> pending{0}; };
public:
  static constexpr uint32_t capacity=64;
  class Token {
  public:
    Token(const Token&)=delete;
    Token& operator=(const Token&)=delete;
    // Claim once BEFORE changing ACK/error state. An immediate return error
    // and its delayed callback cannot both mutate the writer. Keep the slot
    // borrowed until the winner's complete effect is visible to pending().
    // Effects must not throw; if one does, the slot deliberately stays pending
    // rather than inventing successful retirement of an unfinished callback.
    template<class F> bool finish(F&& effect) {
      bool expected=false;
      if(!finalized_.compare_exchange_strong(expected,true,std::memory_order_acq_rel,
                                             std::memory_order_relaxed))return false;
      std::forward<F>(effect)();
      finished_.store(true,std::memory_order_release);
      releaseIfFinished();
      return true;
    }
    bool complete() noexcept {return finish([]{});}
    // A callback may be inline, before the vendor function returns. Call-mode
    // tokens stay pending until BOTH the callback effect and the submitting
    // stack frame have retired. Otherwise an inline ACK could make a blocked
    // old Move appear retired and admit a new task while it is still running.
    void returned() noexcept {
      returned_.store(true,std::memory_order_release);releaseIfFinished();
    }
    // Dropping a token is NOT completion: a missing vendor callback must
    // remain visible as outstanding work, never a no-motion retirement proof.
  private:
    friend class SDKWriteTransactions;
    explicit Token(std::shared_ptr<State> state,bool call_mode):state_(std::move(state)),returned_(!call_mode){}
    void releaseIfFinished() noexcept {
      if(!finished_.load(std::memory_order_acquire)||!returned_.load(std::memory_order_acquire))return;
      bool expected=false;
      if(released_.compare_exchange_strong(expected,true,std::memory_order_acq_rel))
        state_->pending.fetch_sub(1,std::memory_order_acq_rel);
    }
    std::shared_ptr<State> state_;
    std::atomic<bool> finalized_{false};
    std::atomic<bool> finished_{false},returned_{true},released_{false};
  };
  // Keep this guard in the state-locked submitting frame until its immediate
  // return result has been applied to the captured task identity. Never hold
  // the general state lock during the vendor call itself.
  class CallReturn {
  public:
    explicit CallReturn(std::shared_ptr<Token> token):token_(std::move(token)){}
    ~CallReturn(){if(token_)token_->returned();}
    CallReturn(const CallReturn&)=delete;
    CallReturn& operator=(const CallReturn&)=delete;
  private:std::shared_ptr<Token> token_;
  };
  SDKWriteTransactions():state_(std::make_shared<State>()){}
  SDKWriteTransactions(const SDKWriteTransactions&)=delete;
  SDKWriteTransactions& operator=(const SDKWriteTransactions&)=delete;
  // Register before entering the SDK: even an inline callback must see its
  // transaction. Reject saturation instead of allocating an unbounded queue.
  std::shared_ptr<Token> begin() noexcept {return beginImpl(false);}
  std::shared_ptr<Token> beginCall() noexcept {return beginImpl(true);}
  uint32_t pending()const noexcept {return state_->pending.load(std::memory_order_acquire);}
private:
  std::shared_ptr<Token> beginImpl(bool call_mode) noexcept {
    auto count=state_->pending.load(std::memory_order_acquire);
    do {
      if(count>=capacity)return {};
    }while(!state_->pending.compare_exchange_weak(count,count+1,std::memory_order_acq_rel,
                                                  std::memory_order_relaxed));
    try {return std::shared_ptr<Token>(new Token(state_,call_mode));}
    catch(...) {state_->pending.fetch_sub(1,std::memory_order_acq_rel);return {};}
  }
  // Tokens own this shared state, so a late completion never dereferences a
  // destroyed ledger. RMW release sequences also publish all final effects.
  std::shared_ptr<State> state_;
};

// The caller has already admitted one immutable attempt, registered its
// ledger token, and recorded the possibility of nonzero motion under state.
// That admission is the ordering point: a later revoke cannot recall an
// already admitted vendor operation, but it need not wait for that operation
// to return. There is no stale command queue and the next attempt must select
// current state again. This does NOT bound a blocked vendor call or guarantee
// physical stop. Legacy curve-CAS callers retain their existing locked path.
template<class Submit>
auto invokeSdkOutsideState(std::unique_lock<std::mutex>&state,Submit&&submit)->decltype(submit()) {
  state.unlock();
  try {auto result=submit();state.lock();return result;}
  catch(...){state.lock();throw;}
}
}
