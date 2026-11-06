#include "sdk_write_transactions.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <future>
#include <iostream>
#include <system_error>
#include <thread>

using d1monitor::SDKWriteTransactions;
using namespace std::chrono_literals;

int main(){
  // Both orderings are real: callback before return, and delayed callback
  // after return. Neither may retire the stack/ACK half that is still live.
  for(bool callback_first:{false,true}){
    SDKWriteTransactions ledger;auto token=ledger.beginCall();assert(token&&ledger.pending()==1);
    if(callback_first){assert(token->complete());assert(ledger.pending()==1);token->returned();}
    else {token->returned();assert(ledger.pending()==1);assert(token->complete());}
    assert(ledger.pending()==0&&!token->complete());token->returned();assert(ledger.pending()==0);
  }
  // Two simultaneous completion halves decrement exactly once and publish
  // the callback effect before pending() reaches zero.
  for(unsigned i=0;i<500;++i){
    SDKWriteTransactions ledger;auto token=ledger.beginCall();int outcome=0;
    std::thread callback([&]{token->finish([&]{outcome=7;});});
    std::thread returned([&]{token->returned();});callback.join();returned.join();
    assert(ledger.pending()==0&&outcome==7);
  }
  // A fake vendor blocks *after* an inline ACK. Admission has happened, but
  // data/cancel handling still gets the state lock and old work stays pending.
  {
    SDKWriteTransactions ledger;std::mutex state;bool revoked=false,revived=false;
    std::promise<void>inside,release;auto released=release.get_future().share();
    std::thread writer([&]{std::unique_lock<std::mutex>lock(state);
      auto token=ledger.beginCall();SDKWriteTransactions::CallReturn frame(token);
      const auto result=d1monitor::invokeSdkOutsideState(lock,[&]{
        token->complete();inside.set_value();released.wait();return std::error_code{};});
      assert(!result&&lock.owns_lock());if(!revoked)revived=true;
    });
    inside.get_future().wait();
    auto cancel=std::async(std::launch::async,[&]{std::lock_guard<std::mutex>lock(state);
      revoked=true;return ledger.pending();});
    const bool responsive=cancel.wait_for(200ms)==std::future_status::ready;
    release.set_value();writer.join();const auto outstanding=cancel.get();
    assert(responsive&&outstanding==1&&revoked&&!revived&&ledger.pending()==0);
  }
  // Missing ACK/exception does not pretend successful retirement; state lock
  // is restored so the caller can latch uncertainty before frame exit.
  {
    SDKWriteTransactions ledger;std::mutex state;std::unique_lock<std::mutex>lock(state);
    auto token=ledger.beginCall();{
      SDKWriteTransactions::CallReturn frame(token);bool threw=false;
      try{d1monitor::invokeSdkOutsideState(lock,[]{throw std::runtime_error("fake vendor");return 0;});}
      catch(const std::runtime_error&){threw=true;}
      assert(threw&&lock.owns_lock()&&ledger.pending()==1);
    }
    assert(ledger.pending()==1);token->complete();assert(ledger.pending()==0);
  }
  // Still one fixed ledger across grants, no unbounded queue or token reuse.
  {
    SDKWriteTransactions ledger;std::array<std::shared_ptr<SDKWriteTransactions::Token>,64>tokens;
    for(auto&token:tokens){token=ledger.beginCall();assert(token);token->complete();}
    assert(ledger.pending()==64&&!ledger.beginCall());
    for(auto&token:tokens)token->returned();assert(ledger.pending()==0);
  }
  std::cout<<"PASS unlocked SDK frame: responsive state/revoke; inline/delayed ACK; no early retirement; exception/capacity; no vendor initialized\n";
}
