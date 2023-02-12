/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "nie/lidar/io/network_packet_producer.hpp"

#include <gtest/gtest.h>
#include <boost/asio.hpp>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
using Packet = std::vector<std::uint8_t>;
constexpr char kIpAddress[] = "127.0.0.1";
constexpr std::uint16_t kPort = 2368;
std::condition_variable kCondVar;
std::mutex kMutex;
bool kReady = false;

/*
 * Send udp packet to (local) network
 */
void Send(Packet const& in) {
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service);
    boost::asio::ip::udp::endpoint remote_endpoint =
            boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(std::string(kIpAddress)), kPort);
    socket.open(boost::asio::ip::udp::v4());

    boost::system::error_code err;
    socket.send_to(boost::asio::buffer(in), remote_endpoint, 0, err);
    socket.close();
}
}  // namespace

struct NetworkProducerWrapper {
    void Receive(std::string const& test_data) {
        std::unique_ptr<Packet> packet;
        auto producer = nie::io::NetworkProducer<Packet>{std::string(kIpAddress), kPort};
        producer.Start();

        {
            std::lock_guard<std::mutex> lock(kMutex);
            kReady = true;
        }
        kCondVar.notify_one();

        EXPECT_EQ(producer.Produce(&packet), true);
        EXPECT_EQ(strncmp(test_data.c_str(), reinterpret_cast<char*>((*packet).data()), test_data.length()), 0);
    }
};
/*
 * Simple test to check if NetworkProducer is able to receive udp packets from the network
 */
TEST(NetworkPacketProducerTest, ReceiveUDPPacket) {
    std::string test_data("Hello");
    NetworkProducerWrapper networkProducer;
    std::thread receiver_thread([&] { networkProducer.Receive(test_data); });

    Packet send_data(test_data.length());
    uint8_t* buffer = send_data.data();
    test_data.copy(reinterpret_cast<char*>(buffer), test_data.length(), 0);

    // Wait until receiver is ready
    {
        std::unique_lock<std::mutex> lock(kMutex);
        kCondVar.wait(lock, [] { return kReady; });
    }

    Send(send_data);

    receiver_thread.join();
}
