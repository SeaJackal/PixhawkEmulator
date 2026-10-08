#include "udp_transport.h"

#include <array>
#include <boost/asio/buffer.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/ip/address_v4.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/system/error_code.hpp>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <vector>

namespace {

boost::asio::ip::udp::endpoint Address(const UdpEndpoint& endpoint) {
  boost::system::error_code error;
  const auto address =
      boost::asio::ip::make_address_v4(endpoint.address, error);
  if (error) {
    throw std::invalid_argument("Expected numeric IPv4 address: " +
                                endpoint.address);
  }
  return {address, endpoint.port};
}

void Check(const boost::system::error_code& error, const char* operation) {
  if (error) {
    throw std::system_error(static_cast<std::error_code>(error), operation);
  }
}

}

UdpTransport::UdpTransport(const UdpEndpoint& local, const UdpEndpoint& peer)
    : socket_(context_), peer_(Address(peer)) {
  const auto address = Address(local);
  boost::system::error_code error;
  static_cast<void>(socket_.open(boost::asio::ip::udp::v4(), error));
  Check(error, "open");
  static_cast<void>(socket_.bind(address, error));
  Check(error, "bind");
  static_cast<void>(socket_.non_blocking(true, error));
  Check(error, "non_blocking");
}

UdpTransport::~UdpTransport() = default;

std::optional<std::vector<std::byte>> UdpTransport::Receive() {
  constexpr std::size_t kReceiveBufferSize =
      static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max()) + 1;
  std::array<std::byte, kReceiveBufferSize> bytes{};
  while (true) {
    boost::asio::ip::udp::endpoint source;
    boost::system::error_code error;
    const auto count =
        socket_.receive_from(boost::asio::buffer(bytes), source, 0, error);
    if (error == boost::asio::error::would_block ||
        error == boost::asio::error::try_again ||
        error == boost::asio::error::interrupted) {
      return std::nullopt;
    }
    Check(error, "receive_from");
    if (source != peer_) {
      continue;
    }
    return std::vector<std::byte>(
        bytes.begin(),
        std::next(bytes.begin(), static_cast<std::ptrdiff_t>(count)));
  }
}

void UdpTransport::Send(const std::vector<std::byte>& bytes) {
  boost::system::error_code error;
  const auto count =
      socket_.send_to(boost::asio::buffer(bytes), peer_, 0, error);
  Check(error, "send_to");
  if (count != bytes.size()) {
    throw std::runtime_error("Incomplete UDP datagram send");
  }
}

UdpEndpoint UdpTransport::LocalEndpoint() const {
  boost::system::error_code error;
  const auto endpoint = socket_.local_endpoint(error);
  Check(error, "local_endpoint");
  return {endpoint.address().to_string(), endpoint.port()};
}
