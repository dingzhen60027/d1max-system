#pragma once
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <openssl/sha.h>
#include <sstream>
#include <string>
namespace d1max_navigation_bt {
// Identity precheck only. The SDK writer validates all physical evidence.
inline bool recordHashMatches(const std::string&path,const std::string&expected) {
  if(!std::filesystem::path(path).is_absolute()||expected.size()!=64||
     expected.find_first_not_of("0123456789abcdef")!=std::string::npos)return false;
  std::ifstream f(path,std::ios::binary);if(!f)return false;
  SHA256_CTX ctx;SHA256_Init(&ctx);char data[16384];
  while(f){f.read(data,sizeof(data));SHA256_Update(&ctx,data,f.gcount());}if(f.bad())return false;
  unsigned char hash[SHA256_DIGEST_LENGTH];SHA256_Final(hash,&ctx);std::ostringstream out;
  for(auto c:hash)out<<std::hex<<std::setfill('0')<<std::setw(2)<<static_cast<int>(c);
  return out.str()==expected;
}
}
