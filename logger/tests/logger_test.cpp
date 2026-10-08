#include "logger.h"

#include <gtest/gtest.h>

#include <chrono>

TEST(LoggerTest, PreservesLevelTimestampMessageAndOrder) {
  const Logger::TimePoint start{std::chrono::seconds(12345)};
  Logger logger{start};
  logger.Log(Logger::Level::kDebug, start, "initializing");
  logger.Log(Logger::Level::kInfo, start + std::chrono::milliseconds(125),
             "ready");
  logger.Log(Logger::Level::kWarning, start + std::chrono::seconds(1),
             "automatic input timeout");
  logger.Log(Logger::Level::kError, start + std::chrono::seconds(2), "failure");

  EXPECT_EQ(logger.GetText(),
            "[0.000000000] [DEBUG] initializing\n"
            "[0.125000000] [INFO] ready\n"
            "[1.000000000] [WARNING] automatic input timeout\n"
            "[2.000000000] [ERROR] failure\n");
}

TEST(LoggerTest, ReadingDoesNotConsumeAndClearAllowsReuse) {
  Logger logger{Logger::TimePoint{}};
  EXPECT_TRUE(logger.GetText().empty());
  logger.Log(Logger::Level::kInfo, {}, "first");
  const auto text = logger.GetText();
  EXPECT_EQ(logger.GetText(), text);
  logger.Clear();
  EXPECT_TRUE(logger.GetText().empty());
  logger.Log(Logger::Level::kInfo, {}, "second");
  EXPECT_EQ(logger.GetText(), "[0.000000000] [INFO] second\n");
}

TEST(LoggerTest, TakingRecordsPreservesHistoryAndConsumesInOrder) {
  const Logger::TimePoint start{std::chrono::seconds{42}};
  Logger logger{start};
  EXPECT_TRUE(logger.TakeRecords().empty());
  logger.Log(Logger::Level::kInfo, start, "first");
  logger.Log(Logger::Level::kError, start + std::chrono::seconds{1}, "second");
  const auto text = logger.GetText();
  const auto records = logger.TakeRecords();
  ASSERT_EQ(records.size(), 2U);
  EXPECT_EQ(records[0].level, Logger::Level::kInfo);
  EXPECT_EQ(records[0].timestamp, start);
  EXPECT_EQ(records[0].message, "first");
  EXPECT_EQ(records[1].level, Logger::Level::kError);
  EXPECT_EQ(records[1].timestamp, start + std::chrono::seconds{1});
  EXPECT_EQ(records[1].message, "second");
  EXPECT_TRUE(logger.TakeRecords().empty());
  EXPECT_EQ(logger.GetText(), text);
  logger.Log(Logger::Level::kWarning, start, "discard");
  logger.Clear();
  EXPECT_TRUE(logger.TakeRecords().empty());
  EXPECT_TRUE(logger.GetText().empty());
  logger.Log(Logger::Level::kDebug, start, "reused");
  ASSERT_EQ(logger.TakeRecords().size(), 1U);
}
