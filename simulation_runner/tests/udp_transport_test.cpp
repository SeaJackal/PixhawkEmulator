#include "controller.h"
#include "simulation_runner.h"
#include "udp_transport.h"

#include <gtest/gtest.h>

#include <boost/asio/buffer.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/ip/address_v4.hpp>

#include <array>
#include <chrono>
#include <optional>
#include <system_error>
#include <thread>

namespace {

using namespace std::chrono_literals;

template <typename Receive>
std::optional<std::vector<std::byte>> ReceiveUntil(
    Receive receive, std::chrono::milliseconds timeout = 100ms) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  do {
    if (auto bytes = receive()) return bytes;
    std::this_thread::sleep_for(1ms);
  } while (std::chrono::steady_clock::now() < deadline);
  return std::nullopt;
}

class UdpTest : public ::testing::Test {
 protected:
  boost::asio::io_context context;
  boost::asio::ip::udp::socket peer{context};
  UdpEndpoint peer_endpoint;
  boost::asio::ip::udp::endpoint last_source;

  void SetUp() override {
    peer.open(boost::asio::ip::udp::v4());
    peer.bind({boost::asio::ip::address_v4::loopback(), 0});
    peer.non_blocking(true);
    peer_endpoint = {"127.0.0.1", peer.local_endpoint().port()};
  }

  void Send(const UdpEndpoint& endpoint, const std::vector<std::byte>& bytes) {
    const boost::asio::ip::udp::endpoint destination{
        boost::asio::ip::make_address_v4(endpoint.address), endpoint.port};
    ASSERT_EQ(peer.send_to(boost::asio::buffer(bytes), destination), bytes.size());
  }

  std::optional<std::vector<std::byte>> ReceivePeer() {
    return ReceiveUntil([&]() -> std::optional<std::vector<std::byte>> {
      std::array<std::byte, 65536> bytes;
      boost::asio::ip::udp::endpoint source;
      boost::system::error_code error;
      const auto count = peer.receive_from(boost::asio::buffer(bytes), source,
                                           0, error);
      if (error == boost::asio::error::would_block ||
          error == boost::asio::error::try_again ||
          error == boost::asio::error::interrupted) {
        return std::nullopt;
      }
      if (error) {
        throw std::system_error(static_cast<std::error_code>(error),
                                "receive_from");
      }
      last_source = source;
      return std::vector<std::byte>(bytes.begin(), bytes.begin() + count);
    });
  }
};

TEST_F(UdpTest, NonblockingDatagramsRoundTrip) {
  UdpTransport transport{{"127.0.0.1", 0}, peer_endpoint};
  EXPECT_EQ(transport.LocalEndpoint().address, "127.0.0.1");
  EXPECT_NE(transport.LocalEndpoint().port, 0);
  EXPECT_FALSE(transport.Receive());
  const std::vector<std::byte> bytes{std::byte{1}, std::byte{2}};
  Send(transport.LocalEndpoint(), bytes);
  EXPECT_EQ(ReceiveUntil([&] { return transport.Receive(); }), bytes);
  EXPECT_FALSE(transport.Receive());
  transport.Send(bytes);
  EXPECT_EQ(ReceivePeer(), bytes);
}

TEST_F(UdpTest, PreservesEmptyDatagrams) {
  UdpTransport transport{{"127.0.0.1", 0}, peer_endpoint};
  Send(transport.LocalEndpoint(), {});
  const auto empty = ReceiveUntil([&] { return transport.Receive(); });
  ASSERT_TRUE(empty);
  EXPECT_TRUE(empty->empty());
  transport.Send({});
  const auto reply = ReceivePeer();
  ASSERT_TRUE(reply);
  EXPECT_TRUE(reply->empty());
}

TEST_F(UdpTest, DiscardsUnexpectedPeer) {
  UdpTransport transport{{"127.0.0.1", 0}, peer_endpoint};
  UdpTransport stranger{{"127.0.0.1", 0}, transport.LocalEndpoint()};
  stranger.Send({std::byte{9}});
  EXPECT_FALSE(ReceiveUntil([&] { return transport.Receive(); }));
}

TEST_F(UdpTest, ReceivesValidDatagramAfterUnexpectedPeer) {
  UdpTransport transport{{"127.0.0.1", 0}, peer_endpoint};
  UdpTransport stranger{{"127.0.0.1", 0}, transport.LocalEndpoint()};
  stranger.Send({std::byte{9}});
  const std::vector<std::byte> bytes{std::byte{1}, std::byte{2}};
  Send(transport.LocalEndpoint(), bytes);
  EXPECT_EQ(ReceiveUntil([&] { return transport.Receive(); }), bytes);
  EXPECT_FALSE(transport.Receive());
}

TEST_F(UdpTest, BothListenersFilterToSharedPeerAndReleasePorts) {
  UdpEndpoint automatic_endpoint;
  UdpEndpoint manual_endpoint;
  {
    UdpTransport automatic{{"127.0.0.1", 0}, peer_endpoint};
    UdpTransport manual{{"127.0.0.1", 0}, peer_endpoint};
    automatic_endpoint = automatic.LocalEndpoint();
    manual_endpoint = manual.LocalEndpoint();
    EXPECT_NE(automatic_endpoint.port, manual_endpoint.port);
    for (auto* transport : {&automatic, &manual}) {
      UdpTransport stranger{{"127.0.0.1", 0}, transport->LocalEndpoint()};
      stranger.Send({std::byte{9}});
      EXPECT_FALSE(ReceiveUntil([&] { return transport->Receive(); }, 10ms));
      const std::vector<std::byte> bytes{std::byte{5}};
      Send(transport->LocalEndpoint(), bytes);
      EXPECT_EQ(ReceiveUntil([&] { return transport->Receive(); }), bytes);
    }
  }
  EXPECT_NO_THROW((UdpTransport{automatic_endpoint, peer_endpoint}));
  EXPECT_NO_THROW((UdpTransport{manual_endpoint, peer_endpoint}));
}

TEST_F(UdpTest, RejectsInvalidAddressesAndBindConflicts) {
  UdpTransport transport{{"127.0.0.1", 0}, peer_endpoint};
  EXPECT_THROW((UdpTransport{transport.LocalEndpoint(), peer_endpoint}),
               std::system_error);
  for (const auto* address : {"invalid", "localhost", "::1"}) {
    EXPECT_THROW((UdpTransport{{address, 0}, peer_endpoint}),
                 std::invalid_argument);
    EXPECT_THROW((UdpTransport{{"127.0.0.1", 0}, {address, 12345}}),
                 std::invalid_argument);
  }
}

TEST_F(UdpTest, ReleasesBoundPortOnDestruction) {
  UdpEndpoint local;
  {
    UdpTransport transport{{"127.0.0.1", 0}, peer_endpoint};
    local = transport.LocalEndpoint();
  }
  EXPECT_NO_THROW((UdpTransport{local, peer_endpoint}));
}

TEST_F(UdpTest, OversizedSendThrowsStandardSystemError) {
  UdpTransport transport{{"127.0.0.1", 0}, peer_endpoint};
  EXPECT_THROW(transport.Send(std::vector<std::byte>(65536)), std::system_error);
}

TEST_F(UdpTest, RunnerDeliversCommandsAndReturnsTelemetryOverLoopback) {
  const simulation::TimePoint start{};
  DynamicModel model;
  BatteryModel battery{2.0, 2.0};
  Logger logger{start};
  ControllerStateMachine state_machine{model, logger, start, 0ms};
  Controller controller{model, battery, state_machine, start};
  UdpTransport transport{{"127.0.0.1", 0}, peer_endpoint};
  UdpTransport manual_transport{{"127.0.0.1", 0}, peer_endpoint};
  SimulationRunner runner{controller, transport, manual_transport,
      NetworkEmulator{2ms}, NetworkEmulator{3ms}};
  using Command = ControllerStateMachine::CommandType;
  const auto local = transport.LocalEndpoint();
  Send(local, NetworkMessage{CommandMessage{start + 5ms, Command::kUnguard}}.data);
  Send(manual_transport.LocalEndpoint(), NetworkMessage{CommandMessage{start + 6ms, Command::kToManual}}.data);
  Send(manual_transport.LocalEndpoint(), NetworkMessage{ControlMessage{start + 7ms, {1, 0, 0}, {}}}.data);
  for (int i = 0; i < 14; ++i) {
    runner.RunOnce();
  }
  EXPECT_EQ(state_machine.GetState(), ControllerStateMachine::State::kManual);
  EXPECT_NEAR(model.GetPosition().x_meters, 0.094, 1e-12);
  std::vector<NetworkMessage> received;
  for (int index = 0; index < 6; ++index) {
    auto bytes = ReceivePeer();
    ASSERT_TRUE(bytes);
    EXPECT_EQ(last_source.port(), local.port);
    ASSERT_FALSE(bytes->empty());
    received.emplace_back(std::move(*bytes));
  }
  EXPECT_EQ(received.back().type, MessageType::kGlobalPosition);
  auto payload = received.back().data;
  payload.pop_back();
  const auto position = Deserialize<GlobalPositionMessage>(payload);
  EXPECT_EQ(position.timestamp, start + 100ms);
  EXPECT_NEAR(position.position.x_meters, 0.091, 1e-12);
}

TEST_F(UdpTest, RunnerRoutesControlsAndCommandsFromBothListeners) {
  const simulation::TimePoint start{};
  DynamicModel model;
  BatteryModel battery{2.0, 2.0};
  Logger logger{start};
  ControllerStateMachine state_machine{model, logger, start, 0ms};
  Controller controller{model, battery, state_machine, start};
  UdpTransport automatic{{"127.0.0.1", 0}, peer_endpoint};
  UdpTransport manual{{"127.0.0.1", 0}, peer_endpoint};
  SimulationRunner runner{controller, automatic, manual,
      NetworkEmulator{1ms}, NetworkEmulator{0ms}};
  using Command = ControllerStateMachine::CommandType;
  Send(automatic.LocalEndpoint(),
       NetworkMessage{CommandMessage{start + 1ms, Command::kUnguard}}.data);
  Send(manual.LocalEndpoint(),
       NetworkMessage{CommandMessage{start + 2ms, Command::kToManual}}.data);
  Send(manual.LocalEndpoint(),
       NetworkMessage{ControlMessage{start + 3ms, {1, 0, 0}, {}}}.data);
  Send(manual.LocalEndpoint(),
       NetworkMessage{CommandMessage{start + 4ms, Command::kToAutomatic}}.data);
  Send(automatic.LocalEndpoint(),
       NetworkMessage{ControlMessage{start + 5ms, {-1, 0, 0}, {}}}.data);
  for (int i = 0; i < 3; ++i) runner.RunOnce();
  EXPECT_EQ(state_machine.GetState(), ControllerStateMachine::State::kManual);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, 1);
  runner.RunOnce();
  runner.RunOnce();
  EXPECT_EQ(controller.GetTime(), start + 6ms);
  EXPECT_EQ(state_machine.GetState(), ControllerStateMachine::State::kAutomatic);
  EXPECT_DOUBLE_EQ(model.GetLinearVelocity().x_meters_per_second, -1);
}

TEST_F(UdpTest, RunnerSleepsUntilTelemetryAfterDrainingSocket) {
  UdpTransport transport{{"127.0.0.1", 0}, peer_endpoint};
  std::chrono::steady_clock clock;
  const auto simulation_start = clock.now();
  const auto bytes = NetworkMessage{CommandMessage{
      simulation_start + 1s, ControllerStateMachine::CommandType::kHold}}.data;
  for (int i = 0; i < 100; ++i) Send(transport.LocalEndpoint(), bytes);
  DynamicModel model;
  BatteryModel battery{2.0, 2.0};
  Logger logger{simulation_start};
  ControllerStateMachine state_machine{model, logger, simulation_start};
  Controller controller{model, battery, state_machine, simulation_start};
  UdpTransport manual_transport{{"127.0.0.1", 0}, peer_endpoint};
  SimulationRunner runner{controller, transport, manual_transport,
      NetworkEmulator{0ms}, NetworkEmulator{0ms}};
  runner.RunOnce();
  EXPECT_FALSE(transport.Receive());
  runner.RunOnce();
  EXPECT_GE(clock.now() - simulation_start, 20ms);
  EXPECT_EQ(controller.GetTime(), simulation_start + 20ms);
  EXPECT_FALSE(transport.Receive());
}

}
