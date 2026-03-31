/*
    OnStep X INDI Driver — OnStepXComm unit tests

    Uses socketpair() to create a pair of connected file descriptors.
    fds[0] is given to OnStepXComm (the "device" end).
    fds[1] is used by the test to inject responses and inspect commands.

    No INDI device instance is needed; setDevice() is never called so all
    logging guards (if m_dev) are skipped silently.
*/

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "OnStepXComm.h"

// ---------------------------------------------------------------------------
// Helpers for the simulated-device end (fds[1])
// ---------------------------------------------------------------------------

// Read bytes from fd until '#' (inclusive); return everything before '#'.
static std::string readCmd(int fd)
{
    std::string result;
    char c;
    while (read(fd, &c, 1) == 1 && c != '#')
        result += c;
    return result;
}

// Write reply + '#' to fd.
static void writeReply(int fd, const std::string &reply)
{
    std::string r = reply + "#";
    if (write(fd, r.c_str(), r.size()) < 0) {}  // best-effort; test will fail if reply not received
}

// ---------------------------------------------------------------------------
// test_sendCommand_happy_path
//
// A responder thread waits for the command then sends back a valid '#'-
// terminated reply. sendCommand must return true and deliver the reply.
// ---------------------------------------------------------------------------
TEST(OnStepXCommTest, test_sendCommand_happy_path)
{
    int fds[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    std::thread responder([&]()
    {
        readCmd(fds[1]);            // consume ":GVP#"
        writeReply(fds[1], "OnStepX");
    });

    OnStepXComm comm;
    comm.setFd(fds[0]);

    char reply[256];
    EXPECT_TRUE(comm.sendCommand(":GVP#", reply));
    EXPECT_STREQ(reply, "OnStepX");

    responder.join();
    close(fds[0]);
    close(fds[1]);
}

// ---------------------------------------------------------------------------
// test_sendCommand_timeout
//
// No responder writes anything. sendCommand must return false when the timeout
// expires. Uses a short timeout so the test completes quickly.
// ---------------------------------------------------------------------------
TEST(OnStepXCommTest, test_sendCommand_timeout)
{
    int fds[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    OnStepXComm comm;
    comm.setFd(fds[0]);

    char reply[256];
    EXPECT_FALSE(comm.sendCommand(":GVP#", reply, 100 /* ms */));

    close(fds[0]);
    close(fds[1]);
}

// ---------------------------------------------------------------------------
// test_sendCommand_no_hash
//
// The responder sends 260 bytes with no '#'. readUntilHash fills its 256-byte
// buffer (maxLen-1 = 255 data bytes) and returns false without a hash ever
// arriving. sendCommand must return false.
// ---------------------------------------------------------------------------
TEST(OnStepXCommTest, test_sendCommand_no_hash)
{
    int fds[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    std::thread responder([&]()
    {
        readCmd(fds[1]);                        // consume the command
        std::string noHash(260, 'X');
        if (write(fds[1], noHash.c_str(), noHash.size()) < 0) {}
    });

    OnStepXComm comm;
    comm.setFd(fds[0]);

    char reply[256];
    EXPECT_FALSE(comm.sendCommand(":GVP#", reply, 2000));

    responder.join();
    close(fds[0]);
    close(fds[1]);
}

// ---------------------------------------------------------------------------
// test_flushIO_clears_stale
//
// Stale bytes are written to the device end before any command is sent.
// After flushIO() the driver end must have no readable data — verified with
// select(timeout=0). A subsequent sendCommand then completes successfully.
// ---------------------------------------------------------------------------
TEST(OnStepXCommTest, test_flushIO_clears_stale)
{
    int fds[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    OnStepXComm comm;
    comm.setFd(fds[0]);

    // Inject stale data (includes a '#' to make sure it's fully drained,
    // not just stopped at the hash).
    if (write(fds[1], "STALE#DATA_WITHOUT_HASH", 23) < 0) {}
    usleep(10000); // let the kernel deliver the bytes

    comm.flushIO();

    // Nothing should remain readable on fds[0].
    fd_set rfd;
    FD_ZERO(&rfd);
    FD_SET(fds[0], &rfd);
    struct timeval tv { 0, 5000 }; // 5 ms
    EXPECT_EQ(select(fds[0] + 1, &rfd, nullptr, nullptr, &tv), 0)
        << "flushIO must drain all stale data";

    // Verify a clean sendCommand works after the flush.
    std::thread responder([&]()
    {
        readCmd(fds[1]);
        writeReply(fds[1], "Clean");
    });

    char reply[256];
    EXPECT_TRUE(comm.sendCommand(":GVP#", reply, 1000));
    EXPECT_STREQ(reply, "Clean");

    responder.join();
    close(fds[0]);
    close(fds[1]);
}

// ---------------------------------------------------------------------------
// test_mutex_concurrent
//
// N client threads each call sendCommand simultaneously. A single server
// thread handles them one at a time (the mutex inside OnStepXComm ensures
// commands are serialised — each send+receive is atomic from the server's
// perspective). All N must succeed and receive a valid reply.
// ---------------------------------------------------------------------------
TEST(OnStepXCommTest, test_mutex_concurrent)
{
    int fds[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    OnStepXComm comm;
    comm.setFd(fds[0]);

    const int N = 4;

    // Server: process N commands serially, reply to each.
    std::thread server([&]()
    {
        for (int i = 0; i < N; i++)
        {
            readCmd(fds[1]);
            writeReply(fds[1], "OK");
        }
    });

    // Clients: N threads, all hammering sendCommand at the same time.
    std::vector<std::thread> clients;
    std::atomic<int> successCount { 0 };

    for (int i = 0; i < N; i++)
    {
        clients.emplace_back([&]()
        {
            char reply[256];
            if (comm.sendCommand(":GVP#", reply, 2000))
                successCount.fetch_add(1);
        });
    }

    for (auto &t : clients)
        t.join();
    server.join();

    EXPECT_EQ(successCount.load(), N);

    close(fds[0]);
    close(fds[1]);
}

// ---------------------------------------------------------------------------
// main() — defined here so it is compiled directly into the .o and takes
// precedence over indidrivermain.c's main() exported from libindidriver.so.
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
