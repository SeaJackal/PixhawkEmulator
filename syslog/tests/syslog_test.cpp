#include "syslog.h"

#include <gtest/gtest.h>

#include <array>
#include <boost/asio/buffer.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address_v4.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/system/error_code.hpp>
#include <chrono>
#include <cstddef>
#include <string>
#include <thread>

#include "logger.h"
#include "simulation_time.h"

namespace {

using namespace std::chrono_literals;
constexpr simulation::TimePoint kStart{42s};

TEST(SyslogTest, FormatsHeaderSeverityAndSimulationTime) {
  const std::array levels{Logger::Level::kDebug, Logger::Level::kInfo,
                          Logger::Level::kWarning, Logger::Level::kError};
  const std::array priorities{135, 134, 132, 131};
  const std::array names{"DEBUG", "INFO", "WARNING", "ERROR"};
  for (std::size_t index = 0; index < levels.size(); ++index) {
    EXPECT_EQ(FormatSyslog({levels[index], kStart + 125ms, "message"}, kStart),
              "<" + std::to_string(priorities[index]) +
                  ">1 - - pixhawk_simulator - - - [0.125000000] [" +
                  names[index] + "] message\n");
  }
}

TEST(SyslogTest, PreservesEmptyAndMultilineMessages) {
  EXPECT_EQ(FormatSyslog({Logger::Level::kInfo, kStart, ""}, kStart),
            "<134>1 - - pixhawk_simulator - - - [0.000000000] [INFO] \n");
  EXPECT_EQ(
      FormatSyslog({Logger::Level::kInfo, kStart, "first\nsecond"}, kStart),
      "<134>1 - - pixhawk_simulator - - - [0.000000000] [INFO] "
      "first\nsecond\n");
}

TEST(SyslogTest, CapsDatagramsWithoutTruncatingLocalHistory) {
  Logger logger{kStart};
  logger.Log(Logger::Level::kWarning, kStart, std::string(1000, 'x'));
  const auto history = logger.GetText();
  const auto records = logger.TakeRecords();
  const auto packet = FormatSyslog(records.front(), kStart);
  EXPECT_EQ(packet.size(), 480U);
  EXPECT_EQ(packet.find("<132>1 - - pixhawk_simulator - - - "), 0U);
  EXPECT_EQ(packet.substr(packet.size() - 12), " [truncated]");
  EXPECT_EQ(logger.GetText(), history);
  EXPECT_EQ(records.front().message.size(), 1000U);
  const auto overhead =
      FormatSyslog({Logger::Level::kInfo, kStart, ""}, kStart).size();
  const auto exact = FormatSyslog(
      {Logger::Level::kInfo, kStart, std::string(480 - overhead, 'x')}, kStart);
  EXPECT_EQ(exact.size(), 480U);
  EXPECT_EQ(exact.back(), '\n');
  EXPECT_EQ(exact.find("[truncated]"), std::string::npos);
}

TEST(SyslogTest, SendsStandaloneDatagramsToCollector) {
  boost::asio::io_context context;
  boost::asio::ip::udp::socket collector{
      context, {boost::asio::ip::address_v4::loopback(), 0}};
  collector.non_blocking(true);
  SyslogSender sender{{"127.0.0.1", collector.local_endpoint().port()}, kStart};
  const Logger::Record record{Logger::Level::kError, kStart + 1s, "failure"};
  sender.Send(record);
  sender.Send({Logger::Level::kInfo, kStart, "second"});
  for (const auto& expected :
       {FormatSyslog(record, kStart),
        FormatSyslog({Logger::Level::kInfo, kStart, "second"}, kStart)}) {
    std::array<char, 1024> bytes{};
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (true) {
      boost::system::error_code error;
      boost::asio::ip::udp::endpoint source;
      const auto count =
          collector.receive_from(boost::asio::buffer(bytes), source, 0, error);
      if (error == boost::asio::error::would_block ||
          error == boost::asio::error::try_again) {
        ASSERT_LT(std::chrono::steady_clock::now(), deadline);
        std::this_thread::sleep_for(1ms);
        continue;
      }
      ASSERT_FALSE(error) << error.message();
      EXPECT_EQ(std::string(bytes.data(), count), expected);
      break;
    }
  }
}

}
