// Copyright 2026 GitHub Copilot

#include "IRsendRMT.h"

#include "IRsend_test.h"
#include "gtest/gtest.h"

// Verify the RMT send shim API is present and callable on non-ESP32 builds.
TEST(TestIRsendRMT, StubFunctionsCompile) {
  IRsendRMT_begin(4);
  IRsendRMT_enableIROut(4, 38000, 50, true, true);
  EXPECT_EQ(1u, IRsendRMT_mark(4, 100, true, true, 26));
  IRsendRMT_space(4, 200, true);
  IRsendRMT_flush(4);
}

TEST(TestIRsendRMT, DefaultRmtState) {
  IRsendTest irsend(4);
  irsend.begin();
  EXPECT_FALSE(irsend.useRmt());
  EXPECT_EQ(0u, irsend.rmtPeriod());
}
