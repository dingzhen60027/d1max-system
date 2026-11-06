#include "d1max_planning_interfaces/bounded_diagnostic_file.hpp"
#include <cassert>
#include <iostream>

int main(int argc, char **argv) {
  assert(argc == 2);
  const std::filesystem::path root(argv[1]);
  d1max_diagnostics::BoundedDiagnosticFile sink("probe", root, 512);
  assert(sink.directory().empty());  // no I/O before diagnostic worker runs
  assert(!sink.write("{\n}"));
  assert(!sink.write(std::string(20000, 'x')));
  for (int i=0; i<250; ++i)
    assert(sink.write("{\"source_ns\":1234567890123456789,\"seq\":"+std::to_string(i)+"}"));
  assert(sink.overwrittenSlots()>0 && sink.failedWrites()==0 && sink.rejectedBatches()==2);
  d1max_diagnostics::BoundedDiagnosticFile second("probe", root, 512);
  assert(second.write("{\"seq\":999}"));
  assert(second.directory()!=sink.directory());
  // A non-directory root is a disk error; it disables only diagnostics.
  const auto blocker=root/"blocker";
  { std::ofstream f(blocker); f << "unchanged"; }
  d1max_diagnostics::BoundedDiagnosticFile failed("probe", blocker);
  assert(!failed.write("{}") && failed.disabled());
  assert(!failed.write("{}") && failed.failedWrites()==2);
  d1max_diagnostics::BoundedDiagnosticFile invalid("../escape", root);
  assert(!invalid.write("{}") && invalid.disabled());
  std::cout << sink.directory().string() << '\n' << second.directory().string() << '\n';
}
