// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @brief Host tests for FileSink over a std::tmpfile(). No PROS.
 */

#include "mclib/telemetry/file_sink.hpp"

#include "test_assert.hpp"

#include <cstdio>
#include <cstring>
#include <string>

namespace {

using mclib::telemetry::FileSink;

/** @brief Bytes written through the sink come back out of the file. */
void testWriteAndReadBack() {
  std::FILE* file = std::tmpfile();
  CHECK(file != nullptr);
  if (file == nullptr) {
    return;
  }

  FileSink sink(file);
  CHECK(sink.isAvailable());

  const char first[] = "t_ms,x_in\n";
  const char second[] = "0,24\n";
  CHECK(sink.write(first, sizeof(first) - 1));
  CHECK(sink.write(second, sizeof(second) - 1));
  sink.flush();
  CHECK_EQ(static_cast<double>(sink.bytesWritten()),
           static_cast<double>(sizeof(first) - 1 + sizeof(second) - 1));
  CHECK(!sink.writeFailed());

  std::rewind(file);
  char buffer[64] = {};
  const std::size_t got = std::fread(buffer, 1, sizeof(buffer) - 1, file);
  const std::string expected = std::string(first) + second;
  CHECK_EQ(static_cast<double>(got), static_cast<double>(expected.size()));
  CHECK(std::string(buffer, got) == expected);

  // close() stops writes but leaves the file itself open for its owner.
  sink.close();
  CHECK(!sink.isAvailable());
  CHECK(!sink.write("x", 1));
  CHECK(std::fputs("owner still writes\n", file) >= 0);
  std::fclose(file);
}

/** @brief A null stream is unavailable and never written. */
void testNullFile() {
  FileSink sink(nullptr);
  CHECK(!sink.isAvailable());
  CHECK(!sink.write("abc", 3));
  CHECK_EQ(static_cast<double>(sink.bytesWritten()), 0.0);
  CHECK(!sink.writeFailed());
  sink.flush();
  sink.close();
  CHECK(!sink.isAvailable());
}

/** @brief A short write marks the sink dead; later writes are refused. */
void testShortWriteKillsSink() {
  // A stream opened for reading accepts no bytes, so fwrite returns 0.
  std::FILE* read_only = std::fopen("/dev/null", "r");
  if (read_only == nullptr) {
    std::printf("  (skipping short-write case: cannot open /dev/null)\n");
    return;
  }
  FileSink sink(read_only);
  CHECK(sink.isAvailable());
  CHECK(!sink.write("abc", 3));
  CHECK(sink.writeFailed());
  CHECK(!sink.isAvailable());
  CHECK(!sink.write("abc", 3));
  std::fclose(read_only);
}

/** @brief stdoutSink() is one object over stdout. */
void testStdoutSinkIsSingleton() {
  FileSink& a = mclib::telemetry::stdoutSink();
  FileSink& b = mclib::telemetry::stdoutSink();
  CHECK(&a == &b);
  CHECK(a.isAvailable());
}

}  // namespace

int main() {
  testWriteAndReadBack();
  testNullFile();
  testShortWriteKillsSink();
  testStdoutSinkIsSingleton();
  return mclib::test::summary("file_sink_test");
}
