#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/udp.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

struct UdpEndpoint {
  std::string address = "127.0.0.1";
  std::uint16_t port = 0;
};

class UdpTransport {
 public:
  UdpTransport(const UdpEndpoint& local, const UdpEndpoint& peer);
  ~UdpTransport();

  UdpTransport(const UdpTransport&) = delete;
  UdpTransport& operator=(const UdpTransport&) = delete;
  UdpTransport(UdpTransport&&) = delete;
  UdpTransport& operator=(UdpTransport&&) = delete;

  std::optional<std::vector<std::byte>> Receive();
  void Send(const std::vector<std::byte>& bytes);
  [[nodiscard]] UdpEndpoint LocalEndpoint() const;

 private:
  boost::asio::io_context context_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint peer_;
};
