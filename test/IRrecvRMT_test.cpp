// Copyright 2026 GitHub Copilot

#include "IRrecvRMT.h"

#include "IRrecv.h"
#include "IRrecv_test.h"
#include "gtest/gtest.h"

TEST(TestIRrecvRMT, StubFunctionsCompile) {
  uint16_t rawbuf[10] = {};
  uint16_t rawlen = 0;
  uint8_t overflow = 0;
  uint8_t rcvstate = 2;

  IRrecvRMT_begin(5);
  IRrecvRMT_enableIRIn(5, 10, 15);
  IRrecvRMT_poll(5, rawbuf, 10, rawlen, overflow, rcvstate, 15);
  IRrecvRMT_pause(5);
  IRrecvRMT_resume(5);
  IRrecvRMT_disableIRIn(5);

  EXPECT_EQ(0u, rawlen);
  EXPECT_EQ(0u, overflow);
  EXPECT_EQ(2u, rcvstate);
}

TEST(TestIRrecvRMT, ReceiveBufferSize) {
  IRrecv irrecv(6, 50);
  EXPECT_EQ(50, irrecv.getBufSize());
}
