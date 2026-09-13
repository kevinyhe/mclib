// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// SdCardSink: the "no card" path, which is the one a host always has. The
// mount point is the compile-time constant kSdMountPoint ("/usd/"), so on a
// machine without that directory every open fails and the sink has to go
// quiet: unavailable, every write refused, nothing counted, no retry.
//
// If this host does happen to have a writable /usd (a developer who created
// one to try the real path), the file-backed behaviour is exercised too:
// non-colliding filenames, the byte cap and its notice, and close(). Those
// checks are skipped, with a printed note, when /usd is absent - which is why
// nothing in that block is a documented-bug marker.

#include "mclib/telemetry/sd_sink.hpp"
#include "test_assert.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <unistd.h>

using mclib::telemetry::SdCardSink;
using mclib::telemetry::SdSinkConfig;

namespace {

/// Read a whole file into a string. Empty if it cannot be opened.
std::string slurp(const char* path) {
  std::string out;
  std::FILE* file = std::fopen(path, "r");
  if (file == nullptr) {
    return out;
  }
  char buf[256];
  std::size_t n = 0;
  while ((n = std::fread(buf, 1, sizeof(buf), file)) > 0) {
    out.append(buf, n);
  }
  std::fclose(file);
  return out;
}

// ---------------------------------------------------------------------------
// 1. No card: unavailable once, and a no-op from then on.
// ---------------------------------------------------------------------------
void noCard() {
  std::printf("-- no card at %s\n", mclib::telemetry::kSdMountPoint);
  SdCardSink sink("mclib_sd_sink_test_absent");

  // Nothing is opened until the first isAvailable(): path() is empty.
  CHECK(std::strcmp(sink.path(), "") == 0);
  CHECK_EQ(static_cast<double>(sink.bytesWritten()), 0.0);

  CHECK(!sink.isAvailable());
  CHECK(std::strcmp(sink.path(), "") == 0);
  // Asked again: the cached answer, not another probe.
  CHECK(!sink.isAvailable());

  // Writes are refused and not counted. This is not the byte cap and not a
  // short write, it is a sink that never opened.
  const char row[] = "0,1,2\n";
  CHECK(!sink.write(row, sizeof(row) - 1));
  CHECK(!sink.write(row, sizeof(row) - 1));
  CHECK_EQ(static_cast<double>(sink.bytesWritten()), 0.0);
  CHECK(!sink.capped());
  CHECK(!sink.writeFailed());

  // flush() and close() on a sink with no file are harmless, and close() is
  // idempotent.
  sink.flush();
  sink.close();
  sink.close();
  CHECK(!sink.isAvailable());
  CHECK(!sink.write(row, sizeof(row) - 1));
}

// ---------------------------------------------------------------------------
// 2. A write before the first isAvailable() probes on its own.
// ---------------------------------------------------------------------------
void writeProbes() {
  std::printf("-- write() probes when nothing has yet\n");
  SdCardSink sink("mclib_sd_sink_test_absent");
  CHECK(!sink.write("x", 1));
  CHECK(!sink.isAvailable());
  CHECK_EQ(static_cast<double>(sink.bytesWritten()), 0.0);
}

// ---------------------------------------------------------------------------
// 3. A null base name falls back to "log"; a name too long for the 64 byte
//    path buffer makes the sink unavailable rather than truncating.
// ---------------------------------------------------------------------------
void baseNames() {
  std::printf("-- base names\n");
  SdCardSink null_name(nullptr);
  // With no card the only observable is that it did not crash and is quiet.
  CHECK(!null_name.isAvailable());
  CHECK(std::strcmp(null_name.path(), "") == 0);

  // "/usd/" + 60 chars + "000.csv" is 72 bytes, past the 64 byte m_path.
  const std::string long_name(60, 'x');
  SdCardSink too_long(long_name.c_str());
  CHECK(!too_long.isAvailable());
  CHECK(std::strcmp(too_long.path(), "") == 0);
  CHECK(!too_long.write("x", 1));
  CHECK_EQ(static_cast<double>(too_long.bytesWritten()), 0.0);
}

// ---------------------------------------------------------------------------
// 4. Only when this host has a writable /usd: the file-backed behaviour.
// ---------------------------------------------------------------------------
void withCard() {
  if (::access(mclib::telemetry::kSdMountPoint, W_OK) != 0) {
    std::printf("-- %s is not writable here: file-backed checks skipped\n",
                mclib::telemetry::kSdMountPoint);
    return;
  }
  std::printf("-- file-backed checks against %s\n", mclib::telemetry::kSdMountPoint);

  char base[32];
  std::snprintf(base, sizeof(base), "mclibsdt%ld", static_cast<long>(::getpid()));

  std::string first_path;
  std::string second_path;
  {
    SdCardSink first(base);
    CHECK(first.isAvailable());
    first_path = first.path();
    CHECK(first_path.size() > 7 && first_path.substr(first_path.size() - 7) == "000.csv");

    // A second sink over the same base name picks the next free index.
    SdCardSink second(base);
    CHECK(second.isAvailable());
    second_path = second.path();
    CHECK(second_path.size() > 7 && second_path.substr(second_path.size() - 7) == "001.csv");

    CHECK(first.write("abc\n", 4));
    CHECK_EQ(static_cast<double>(first.bytesWritten()), 4.0);
    first.flush();
    first.close();
    CHECK(!first.write("abc\n", 4));
    CHECK(!first.isAvailable());
    CHECK_EQ(static_cast<double>(first.bytesWritten()), 4.0);
  }
  CHECK(slurp(first_path.c_str()) == "abc\n");
  std::remove(first_path.c_str());
  std::remove(second_path.c_str());

  // The byte cap: the write that would cross it is refused, a notice lands in
  // the file, and the sink closes.
  {
    SdSinkConfig config;
    config.max_bytes = 10;
    SdCardSink capped(base, config);
    CHECK(capped.isAvailable());
    const std::string path = capped.path();
    CHECK(capped.write("123456", 6));
    CHECK(!capped.write("789012", 6));
    CHECK(capped.capped());
    CHECK(!capped.writeFailed());
    CHECK_EQ(static_cast<double>(capped.bytesWritten()), 6.0);
    CHECK(!capped.isAvailable());
    CHECK(!capped.write("x", 1));
    const std::string contents = slurp(path.c_str());
    CHECK(contents == "123456# telemetry: byte cap reached, log truncated\n");
    std::remove(path.c_str());
  }

  // max_bytes 0 disables the cap.
  {
    SdSinkConfig config;
    config.max_bytes = 0;
    SdCardSink open(base, config);
    CHECK(open.isAvailable());
    const std::string path = open.path();
    for (int i = 0; i < 100; ++i) {
      CHECK(open.write("0123456789", 10));
    }
    CHECK(!open.capped());
    CHECK_EQ(static_cast<double>(open.bytesWritten()), 1000.0);
    open.close();
    std::remove(path.c_str());
  }
}

}  // namespace

int main() {
  noCard();
  writeProbes();
  baseNames();
  withCard();
  return mclib::test::summary("sd_sink");
}
