/*
    OnStep X INDI Driver — Stage 7 model-builder transaction tests

    These tests exercise the firmware-model replacement transaction through
    the real OnStepXComm implementation.

    A local socketpair stands in for the serial connection.  StatefulFirmware
    models the firmware-side coefficient store:

        :GX0n#       reads the current RAM model
        :SX0n,value# modifies the current RAM model
        :SX09,2#     activates the model
        :AW#         persists the current model

    The tests deliberately call the transaction helper directly so that the
    numerical fitter is not part of these tests.  Fitter behaviour is tested
    separately by the existing model-fitter tests.
*/

#include <defaultdevice.h>
#include <vector>

#include "OnStepXModelMath.h"
#include "OnStepXModelProtocol.h"
#include "OnStepXStatus.h"

#define private public
#include "OnStepXModelBuilder.h"
#undef private

#include "OnStepXComm.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <optional>

#include <sys/socket.h>
#include <unistd.h>

namespace
{

using Values = OnStepXModelProtocol::Values;

struct ExpectedCommand
{
    std::string command;
    std::string reply;
};

/*
 * A small stateful model of the firmware protocol used by Stage 7.
 *
 * This is deliberately not a general OnStepX emulator.  It implements only
 * the protocol operations used by replaceFirmwareModel(), while retaining
 * actual model state so that the tests can verify rollback and persistence.
 */
class StatefulFirmware
{
    public:
        explicit StatefulFirmware(
            int fd,
            Values initialModel,
            std::vector<ExpectedCommand> expectedCommands)
            : m_fd(fd),
              m_model(initialModel),
              m_persistentModel(initialModel),
              m_expectedCommands(std::move(expectedCommands))
        {
        }

        ~StatefulFirmware()
        {
            stop();
        }

        void start()
        {
            m_thread = std::thread([this]() { run(); });
        }

        void stop()
        {
            m_stop = true;

            /*
             * Wake any blocking read/write in the responder thread.
             */
            if (m_fd >= 0)
                shutdown(m_fd, SHUT_RDWR);

            if (m_thread.joinable())
                m_thread.join();

            if (m_fd >= 0)
            {
                close(m_fd);
                m_fd = -1;
            }
        }

        bool complete() const
        {
            return m_complete.load();
        }

        bool protocolError() const
        {
            return m_protocolError.load();
        }

        Values model() const
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_model;
        }

        Values persistentModel() const
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_persistentModel;
        }

        bool activated() const
        {
            return m_activated.load();
        }

        std::vector<std::string> commands() const
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_commands;
        }

        /*
         * Make one GX read return a non-numeric response.
         *
         * readNumber is zero-based over all GX reads received by the fake.
         */
        void malformedRead(std::size_t readNumber,
                           std::string reply)
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            m_malformedReadNumber = readNumber;
            m_malformedReadReply = std::move(reply);
        }

        /*
         * Make one GX read return the actual value plus delta.
         *
         * This is used to simulate a firmware readback mismatch.
         */
        void mismatchedRead(std::size_t readNumber,
                            char index,
                            std::int64_t delta)
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            m_mismatchReadNumber = readNumber;
            m_mismatchIndex = index;
            m_mismatchDelta = delta;
        }

        /*
         * Make a particular SX coefficient write return failure without
         * modifying the firmware model.
         */
        void failedWrite(char index)
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            m_failedWriteIndex = index;
        }

        /*
         * Make :SX09,2# return failure instead of activating the model.
         */
        void failedActivation()
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            m_failActivation = true;
        }

        /*
         * Make :AW# return failure instead of persisting the current model.
         */
        void failedPersistence()
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            m_failPersistence = true;
        }

    private:
        static bool parseIndex(const std::string &command,
                               char &index)
        {
            if (command.size() != 6 ||
                command.compare(0, 4, ":GX0") != 0 ||
                command[5] != '#')
                return false;

            index = command[4];
            return true;
        }

        static bool parseWrite(const std::string &command,
                               char &index,
                               std::int64_t &value)
        {
            if (command.size() < 8 ||
                command.compare(0, 4, ":SX0") != 0 ||
                command.back() != '#')
                return false;

            index = command[4];

            /*
             * :SX0n,value#
             *
             * Characters:
             *   0 1 2 3 4 5 ...
             *   : S X 0 n , value #
             */
            const std::string valueText =
                command.substr(6, command.size() - 7);

            if (valueText.empty())
                return false;

            char *end = nullptr;
            errno = 0;

            const long long parsed =
                std::strtoll(valueText.c_str(), &end, 10);

            if (errno == ERANGE ||
                end == valueText.c_str() ||
                *end != '\0')
                return false;

            value = static_cast<std::int64_t>(parsed);
            return true;
        }

        static bool getValue(const Values &values,
                             char index,
                             std::int64_t &value)
        {
            switch (index)
            {
                case '0': value = values.ax1Cor; return true;
                case '1': value = values.ax2Cor; return true;
                case '2': value = values.altCor; return true;
                case '3': value = values.azmCor; return true;
                case '4': value = values.doCor;  return true;
                case '5': value = values.pdCor;  return true;
                case '6':
                case '7':
                    value = values.dfCor;
                    return true;
                case '8': value = values.tfCor; return true;
                case 'a': value = values.hcp;    return true;
                case 'b': value = values.hca;    return true;
                case 'c': value = values.dcp;    return true;
                case 'd': value = values.dca;    return true;
                default:
                    return false;
            }
        }

        static bool setValue(Values &values,
                             char index,
                             std::int64_t value)
        {
            switch (index)
            {
                case '0': values.ax1Cor = value; return true;
                case '1': values.ax2Cor = value; return true;
                case '2': values.altCor = value; return true;
                case '3': values.azmCor = value; return true;
                case '4': values.doCor  = value; return true;
                case '5': values.pdCor  = value; return true;
                case '6':
                case '7':
                    values.dfCor = value;
                    return true;
                case '8': values.tfCor = value; return true;
                case 'a': values.hcp   = value; return true;
                case 'b': values.hca   = value; return true;
                case 'c': values.dcp   = value; return true;
                case 'd': values.dca   = value; return true;
                default:
                    return false;
            }
        }

        bool readCommand(std::string &command)
        {
            command.clear();

            while (!m_stop)
            {
                char c = 0;
                const ssize_t n = read(m_fd, &c, 1);

                if (n != 1)
                    return false;

                command.push_back(c);

                if (c == '#')
                    return true;
            }

            return false;
        }

        bool writeReply(const std::string &reply)
        {
            const std::string framedReply = reply + "#";

            std::size_t written = 0;

            while (written < framedReply.size() && !m_stop)
            {
                const ssize_t n = write(
                    m_fd,
                    framedReply.data() + written,
                    framedReply.size() - written);

                if (n <= 0)
                    return false;

                written += static_cast<std::size_t>(n);
            }

            return written == framedReply.size();
        }

        bool validateExpectedCommand(const std::string &command)
        {
            std::lock_guard<std::mutex> lock(m_mutex);

            m_commands.push_back(command);

            if (m_nextExpected >= m_expectedCommands.size() ||
                command != m_expectedCommands[m_nextExpected].command)
            {
                m_protocolError = true;
                return false;
            }

            ++m_nextExpected;
            return true;
        }

        bool processCommand(const std::string &command,
                            std::string &reply)
        {
            if (!validateExpectedCommand(command))
                return false;

            /*
             * Some lifecycle tests use fixed replies for non-model commands
             * such as :GtH#, :GRH#, :GDH#, and :GSH#.
             */
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                const auto &expected = m_expectedCommands[m_nextExpected - 1];
                if (!expected.reply.empty())
                {
                    reply = expected.reply;
                    return true;
                }
            }

            /*
             * Activation.
             */
            if (command == ":SX09,2#")
            {
                std::lock_guard<std::mutex> lock(m_mutex);

                if (m_failActivation)
                {
                    reply = "0";
                    return true;
                }

                m_activated = true;
                reply = "1";
                return true;
            }

            /*
             * Persistence.
             *
             * A successful AW copies the current firmware model into the
             * simulated persistent store.
             */
            if (command == ":AW#")
            {
                std::lock_guard<std::mutex> lock(m_mutex);

                if (m_failPersistence)
                {
                    reply = "0";
                    return true;
                }

                m_persistentModel = m_model;
                reply = "1";
                return true;
            }

            char index = 0;

            /*
             * Coefficient read.
             */
            if (parseIndex(command, index))
            {
                const std::size_t readNumber = m_readCount++;

                std::lock_guard<std::mutex> lock(m_mutex);

                if (m_malformedReadNumber.has_value() &&
                    readNumber == *m_malformedReadNumber)
                {
                    reply = m_malformedReadReply;
                    return true;
                }

                std::int64_t value = 0;

                if (!getValue(m_model, index, value))
                {
                    m_protocolError = true;
                    return false;
                }

                if (m_mismatchReadNumber.has_value() &&
                    readNumber == *m_mismatchReadNumber &&
                    index == m_mismatchIndex)
                {
                    value += m_mismatchDelta;
                }

                reply = std::to_string(value);
                return true;
            }

            /*
             * Coefficient write.
             */
            std::int64_t value = 0;

            if (parseWrite(command, index, value))
            {
                std::lock_guard<std::mutex> lock(m_mutex);

                if (m_failedWriteIndex.has_value() &&
                    index == *m_failedWriteIndex)
                {
                    /*
                     * Fail exactly one write.  Rollback must subsequently be
                     * allowed to write the original value successfully.
                     */
                    m_failedWriteIndex.reset();
                    reply = "0";
                    return true;
                }

                if (!setValue(m_model, index, value))
                {
                    m_protocolError = true;
                    return false;
                }

                reply = "1";
                return true;
            }

            m_protocolError = true;
            return false;
        }

        void run()
        {
            while (!m_stop)
            {
                std::string command;

                if (!readCommand(command))
                    return;

                std::string reply;

                if (!processCommand(command, reply))
                    return;

                if (!writeReply(reply))
                    return;

                {
                    std::lock_guard<std::mutex> lock(m_mutex);

                    if (m_nextExpected == m_expectedCommands.size())
                    {
                        m_complete = true;
                        return;
                    }
                }
            }
        }

        int m_fd { -1 };

        mutable std::mutex m_mutex;

        /*
         * Current firmware-side RAM model.
         */
        Values m_model {};

        /*
         * Simulated non-volatile model.
         */
        Values m_persistentModel {};

        std::vector<ExpectedCommand> m_expectedCommands;
        std::vector<std::string> m_commands;

        std::size_t m_nextExpected { 0 };
        std::size_t m_readCount { 0 };

        /*
         * GX failure injection.
         */
        std::optional<std::size_t> m_malformedReadNumber;
        std::string m_malformedReadReply;

        std::optional<std::size_t> m_mismatchReadNumber;
        char m_mismatchIndex { 0 };
        std::int64_t m_mismatchDelta { 0 };

        /*
         * SX/SX09/AW failure injection.
         */
        std::optional<char> m_failedWriteIndex;
        bool m_failActivation { false };
        bool m_failPersistence { false };

        std::thread m_thread;

        std::atomic<bool> m_stop { false };
        std::atomic<bool> m_complete { false };
        std::atomic<bool> m_protocolError { false };
        std::atomic<bool> m_activated { false };
};

Values makeValues(std::int64_t base)
{
    Values values;
    values.ax1Cor = base + 0;
    values.ax2Cor = base + 1;
    values.altCor = base + 2;
    values.azmCor = base + 3;
    values.doCor  = base + 4;
    values.pdCor  = base + 5;
    values.dfCor  = base + 6;
    values.tfCor  = base + 7;
    values.hcp    = base + 8;
    values.hca    = base + 9;
    values.dcp    = base + 10;
    values.dca    = base + 11;
    return values;
}

std::vector<std::pair<char, std::int64_t>>
valueEntries(const Values &values, char dfIndex)
{
    return {
        {'0', values.ax1Cor},
        {'1', values.ax2Cor},
        {'2', values.altCor},
        {'3', values.azmCor},
        {'4', values.doCor},
        {'5', values.pdCor},
        {dfIndex, values.dfCor},
        {'8', values.tfCor},
        {'a', values.hcp},
        {'b', values.hca},
        {'c', values.dcp},
        {'d', values.dca},
    };
}

std::vector<ExpectedCommand>
readCommands(const Values &values, char dfIndex)
{
    std::vector<ExpectedCommand> commands;

    for (const auto &[index, value] : valueEntries(values, dfIndex))
    {
        (void)value;

        commands.push_back({
            std::string(":GX0") + index + "#"
        });
    }

    return commands;
}

void appendWriteCommands(std::vector<ExpectedCommand> &commands,
                         const Values &values,
                         char dfIndex)
{
    for (const auto &[index, value] : valueEntries(values, dfIndex))
    {
        commands.push_back({
            std::string(":SX0") + index + "," + std::to_string(value) + "#"
        });
    }
}

void appendActivationAndPersistence(
    std::vector<ExpectedCommand> &commands)
{
    commands.push_back({":SX09,2#"});
    commands.push_back({":AW#"});
}

static void expectProtocolValuesEqual(
    const OnStepXModelProtocol::Values &actual,
    const OnStepXModelProtocol::Values &expected)
{
    EXPECT_EQ(actual.ax1Cor, expected.ax1Cor);
    EXPECT_EQ(actual.ax2Cor, expected.ax2Cor);
    EXPECT_EQ(actual.altCor, expected.altCor);
    EXPECT_EQ(actual.azmCor, expected.azmCor);
    EXPECT_EQ(actual.doCor,  expected.doCor);
    EXPECT_EQ(actual.pdCor,  expected.pdCor);
    EXPECT_EQ(actual.dfCor,  expected.dfCor);
    EXPECT_EQ(actual.tfCor,  expected.tfCor);
    EXPECT_EQ(actual.hcp,    expected.hcp);
    EXPECT_EQ(actual.hca,    expected.hca);
    EXPECT_EQ(actual.dcp,    expected.dcp);
    EXPECT_EQ(actual.dca,    expected.dca);
}

} // namespace


TEST(OnStepXModelBuilderStage7,
     SuccessfulReplacementReadsWritesVerifiesActivatesAndPersists)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    const Values original = makeValues(100);
    const Values pending = makeValues(200);

    std::vector<ExpectedCommand> expected =
        readCommands(original, '7');

    appendWriteCommands(expected, pending, '7');

    for (const auto &[index, value] : valueEntries(pending, '7'))
    {
        (void)value;

        expected.push_back({
            std::string(":GX0") + index + "#"
        });
    }

    appendActivationAndPersistence(expected);

    StatefulFirmware peer(fds[1], original, std::move(expected));

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_mountType = MountStatus::MountType::GEM;
    builder.m_pendingProtocol = pending;
    builder.m_hasPendingModel = true;

    peer.start();

    EXPECT_TRUE(builder.replaceFirmwareModel());

    peer.stop();
    close(fds[0]);

    EXPECT_TRUE(peer.complete());
    EXPECT_FALSE(peer.protocolError());
    EXPECT_TRUE(peer.activated());
    expectProtocolValuesEqual(peer.model(), pending);
    expectProtocolValuesEqual(peer.persistentModel(), pending);
}


TEST(OnStepXModelBuilderStage7,
     WriteFailureRollsBackAllOriginalCoefficients)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    const Values original = makeValues(100);
    const Values pending = makeValues(200);

    std::vector<ExpectedCommand> expected =
        readCommands(original, '7');

    /*
     * The transaction writes through index 4 and then receives failure.
     * No further pending writes occur.
     */
    const auto pendingEntries = valueEntries(pending, '7');

    for (const auto &[index, value] : pendingEntries)
    {
        expected.push_back({
            std::string(":SX0") + index + "," + std::to_string(value) + "#"
        });

        if (index == '4')
            break;
    }

    /*
     * Rollback writes all original coefficients.
     */
    appendWriteCommands(expected, original, '7');

    StatefulFirmware peer(fds[1], original, std::move(expected));
    peer.failedWrite('4');

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_mountType = MountStatus::MountType::GEM;
    builder.m_pendingProtocol = pending;
    builder.m_hasPendingModel = true;

    peer.start();

    EXPECT_FALSE(builder.replaceFirmwareModel());

    peer.stop();
    close(fds[0]);

    EXPECT_TRUE(peer.complete());
    EXPECT_FALSE(peer.protocolError());
    EXPECT_FALSE(peer.activated());
    expectProtocolValuesEqual(peer.model(), original);
    expectProtocolValuesEqual(peer.persistentModel(), original);
}


TEST(OnStepXModelBuilderStage7,
     ReadbackMismatchRollsBackAllOriginalCoefficients)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    const Values original = makeValues(100);
    const Values pending = makeValues(200);

    std::vector<ExpectedCommand> expected =
        readCommands(original, '7');

    appendWriteCommands(expected, pending, '7');

    /*
     * The fake injects a mismatch into the read of coefficient 3, but the
     * builder reads the complete firmware model before comparing it with
     * the pending model.  Therefore all twelve GX readback commands must
     * be expected, followed by the complete rollback.
     */
    for (const auto &[index, value] : valueEntries(pending, '7'))
    {
        (void)value;

        expected.push_back({
            std::string(":GX0") + index + "#"
        });
    }

    appendWriteCommands(expected, original, '7');

    StatefulFirmware peer(fds[1], original, std::move(expected));
    peer.mismatchedRead(15, '3', 1);

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_mountType = MountStatus::MountType::GEM;
    builder.m_pendingProtocol = pending;
    builder.m_hasPendingModel = true;

    peer.start();

    EXPECT_FALSE(builder.replaceFirmwareModel());

    peer.stop();
    close(fds[0]);

    EXPECT_TRUE(peer.complete());
    EXPECT_FALSE(peer.protocolError());
    EXPECT_FALSE(peer.activated());
    expectProtocolValuesEqual(peer.model(), original);
    expectProtocolValuesEqual(peer.persistentModel(), original);
}


TEST(OnStepXModelBuilderStage7,
     ActivationFailureDoesNotAttemptRollback)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    const Values original = makeValues(100);
    const Values pending = makeValues(200);

    std::vector<ExpectedCommand> expected =
        readCommands(original, '7');

    appendWriteCommands(expected, pending, '7');

    for (const auto &[index, value] : valueEntries(pending, '7'))
    {
        (void)value;

        expected.push_back({
            std::string(":GX0") + index + "#"
        });
    }

    expected.push_back({":SX09,2#"});

    StatefulFirmware peer(fds[1], original, std::move(expected));
    peer.failedActivation();

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_mountType = MountStatus::MountType::GEM;
    builder.m_pendingProtocol = pending;
    builder.m_hasPendingModel = true;

    peer.start();

    EXPECT_FALSE(builder.replaceFirmwareModel());

    peer.stop();
    close(fds[0]);

    EXPECT_TRUE(peer.complete());
    EXPECT_FALSE(peer.protocolError());
    EXPECT_FALSE(peer.activated());
    expectProtocolValuesEqual(peer.model(), pending);
    expectProtocolValuesEqual(peer.persistentModel(), original);
}


TEST(OnStepXModelBuilderStage7,
     PersistenceFailureDoesNotAttemptRollback)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    const Values original = makeValues(100);
    const Values pending = makeValues(200);

    std::vector<ExpectedCommand> expected =
        readCommands(original, '7');

    appendWriteCommands(expected, pending, '7');

    for (const auto &[index, value] : valueEntries(pending, '7'))
    {
        (void)value;

        expected.push_back({
            std::string(":GX0") + index + "#"
        });
    }

    appendActivationAndPersistence(expected);

    StatefulFirmware peer(fds[1], original, std::move(expected));
    peer.failedPersistence();

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_mountType = MountStatus::MountType::GEM;
    builder.m_pendingProtocol = pending;
    builder.m_hasPendingModel = true;

    peer.start();

    EXPECT_FALSE(builder.replaceFirmwareModel());

    peer.stop();
    close(fds[0]);

    EXPECT_TRUE(peer.complete());
    EXPECT_FALSE(peer.protocolError());
    EXPECT_TRUE(peer.activated());
    expectProtocolValuesEqual(peer.model(), pending);
    expectProtocolValuesEqual(peer.persistentModel(), original);
}


TEST(OnStepXModelBuilderStage7,
     ForkUsesCoefficientIndexSixForDf)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    const Values original = makeValues(100);
    const Values pending = makeValues(200);

    std::vector<ExpectedCommand> expected =
        readCommands(original, '6');

    appendWriteCommands(expected, pending, '6');

    for (const auto &[index, value] : valueEntries(pending, '6'))
    {
        (void)value;

        expected.push_back({
            std::string(":GX0") + index + "#"
        });
    }

    appendActivationAndPersistence(expected);

    StatefulFirmware peer(fds[1], original, std::move(expected));

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_mountType = MountStatus::MountType::FORK;
    builder.m_pendingProtocol = pending;
    builder.m_hasPendingModel = true;

    peer.start();

    EXPECT_TRUE(builder.replaceFirmwareModel());

    peer.stop();
    close(fds[0]);

    EXPECT_TRUE(peer.complete());
    EXPECT_FALSE(peer.protocolError());
    EXPECT_TRUE(peer.activated());
    expectProtocolValuesEqual(peer.model(), pending);
    expectProtocolValuesEqual(peer.persistentModel(), pending);
}


TEST(OnStepXModelBuilderStage7,
     MalformedInitialReadPreventsAnyWrite)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    const Values original = makeValues(100);
    const Values pending = makeValues(200);

    /*
     * Only six GX commands are expected because the malformed sixth reply
     * terminates the initial read before any coefficient write occurs.
     */
    std::vector<ExpectedCommand> expected;

    const auto entries = valueEntries(original, '7');

    for (std::size_t i = 0; i < 6; ++i)
    {
        expected.push_back({
            std::string(":GX0") + entries[i].first + "#"
        });
    }

    StatefulFirmware peer(fds[1], original, std::move(expected));
    peer.malformedRead(5, "not-an-integer");

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_mountType = MountStatus::MountType::GEM;
    builder.m_pendingProtocol = pending;
    builder.m_hasPendingModel = true;

    peer.start();

    EXPECT_FALSE(builder.replaceFirmwareModel());

    peer.stop();
    close(fds[0]);

    EXPECT_TRUE(peer.complete());
    EXPECT_FALSE(peer.protocolError());
    EXPECT_FALSE(peer.activated());
    expectProtocolValuesEqual(peer.model(), original);
    expectProtocolValuesEqual(peer.persistentModel(), original);

    /*
     * Six GX reads and no SX writes.
     */
    const auto commands = peer.commands();
    ASSERT_EQ(commands.size(), 6u);

    for (const auto &command : commands)
        EXPECT_EQ(command.compare(0, 4, ":GX0"), 0);
}


TEST(OnStepXModelBuilderLifecycle,
     StartBuildRequiresTracking)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_tracking = false;
    builder.m_mountType = MountStatus::MountType::GEM;

    EXPECT_FALSE(builder.startBuild());
    EXPECT_FALSE(builder.isBuilding());

    close(fds[0]);
    close(fds[1]);
}


TEST(OnStepXModelBuilderLifecycle,
     StartBuildRequiresKnownMountType)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_tracking = true;
    builder.m_mountType = MountStatus::MountType::UNKNOWN;

    EXPECT_FALSE(builder.startBuild());
    EXPECT_FALSE(builder.isBuilding());

    close(fds[0]);
    close(fds[1]);
}


TEST(OnStepXModelBuilderLifecycle,
     StartBuildReadsLatitudeAndEntersBuildMode)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    std::vector<ExpectedCommand> expected = {
        {":GtH#", "51:30:00"}
    };

    StatefulFirmware peer(fds[1], Values {}, std::move(expected));

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_tracking = true;
    builder.m_mountType = MountStatus::MountType::GEM;
    builder.m_building = false;

    peer.start();

    EXPECT_TRUE(builder.startBuild());
    EXPECT_TRUE(builder.isBuilding());
    EXPECT_NEAR(builder.m_latitudeRad,
                51.5 * 3.14159265358979323846 / 180.0,
                1e-12);

    peer.stop();
    close(fds[0]);

    EXPECT_TRUE(peer.complete());
    EXPECT_FALSE(peer.protocolError());
}


TEST(OnStepXModelBuilderLifecycle,
     StartBuildRejectsLatitudeReadFailure)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    std::vector<ExpectedCommand> expected = {
        {":GtH#", "not-a-coordinate"}
    };

    StatefulFirmware peer(fds[1], Values {}, std::move(expected));

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_tracking = true;
    builder.m_mountType = MountStatus::MountType::GEM;

    peer.start();

    EXPECT_FALSE(builder.startBuild());
    EXPECT_FALSE(builder.isBuilding());

    peer.stop();
    close(fds[0]);

    EXPECT_TRUE(peer.complete());
    EXPECT_FALSE(peer.protocolError());
}


TEST(OnStepXModelBuilderLifecycle,
     CaptureSyncStoresObservation)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    std::vector<ExpectedCommand> expected = {
        {":GRH#", "05:00:00"},
        {":GDH#", "+20:00:00"},
        {":GSH#", "08:00:00"}
    };

    StatefulFirmware peer(fds[1], Values {}, std::move(expected));

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_building = true;
    builder.m_mountType = MountStatus::MountType::GEM;
    builder.m_latitudeRad = 51.5 * 3.14159265358979323846 / 180.0;

    peer.start();

    EXPECT_TRUE(builder.captureSync(6.0, 30.0,
                                    MountStatus::PierSide::WEST,
                                    MountStatus::MountType::GEM));
    ASSERT_EQ(builder.observationCount(), 1u);

    const auto &observation = builder.m_observations.front();
    EXPECT_DOUBLE_EQ(observation.actualRAHours, 6.0);
    EXPECT_DOUBLE_EQ(observation.actualDecDeg, 30.0);
    EXPECT_DOUBLE_EQ(observation.mountRAHours, 5.0);
    EXPECT_DOUBLE_EQ(observation.mountDecDeg, 20.0);
    EXPECT_DOUBLE_EQ(observation.lstHours, 8.0);

    peer.stop();
    close(fds[0]);

    EXPECT_TRUE(peer.complete());
    EXPECT_FALSE(peer.protocolError());
}


TEST(OnStepXModelBuilderLifecycle,
     CaptureSyncConsumesObservationWhenMountReadFails)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    std::vector<ExpectedCommand> expected = {
        {":GRH#", "not-an-angle"}
    };

    StatefulFirmware peer(fds[1], Values {}, std::move(expected));

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_building = true;
    builder.m_mountType = MountStatus::MountType::GEM;

    peer.start();

    EXPECT_TRUE(builder.captureSync(6.0, 30.0,
                                    MountStatus::PierSide::EAST,
                                    MountStatus::MountType::GEM));
    EXPECT_EQ(builder.observationCount(), 0u);

    peer.stop();
    close(fds[0]);

    EXPECT_TRUE(peer.complete());
    EXPECT_FALSE(peer.protocolError());
}


TEST(OnStepXModelBuilderLifecycle,
     CaptureSyncConsumesObservationWhenMountTypeChanges)
{
    int fds[2] = {-1, -1};
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);

    std::vector<ExpectedCommand> expected = {
        {":GRH#", "05:00:00"},
        {":GDH#", "+20:00:00"},
        {":GSH#", "08:00:00"}
    };

    StatefulFirmware peer(fds[1], Values {}, std::move(expected));

    OnStepXComm comm;
    comm.setFd(fds[0]);

    OnStepXModelBuilder builder;
    builder.setComm(&comm);
    builder.m_building = true;
    builder.m_mountType = MountStatus::MountType::GEM;

    peer.start();

    EXPECT_TRUE(builder.captureSync(6.0, 30.0,
                                    MountStatus::PierSide::EAST,
                                    MountStatus::MountType::FORK));
    EXPECT_EQ(builder.observationCount(), 0u);

    peer.stop();
    close(fds[0]);

    EXPECT_TRUE(peer.complete());
    EXPECT_FALSE(peer.protocolError());
}


TEST(OnStepXModelBuilderLifecycle,
     TrackingOffAbortsActiveBuild)
{
    OnStepXModelBuilder builder;
    builder.m_tracking = true;
    builder.m_building = true;
    builder.m_observations.emplace_back();

    builder.updateTrackingState(false);

    EXPECT_FALSE(builder.m_tracking);
    EXPECT_FALSE(builder.isBuilding());
    EXPECT_EQ(builder.observationCount(), 0u);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}