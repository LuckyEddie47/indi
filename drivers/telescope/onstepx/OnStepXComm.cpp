/*
    OnStep X INDI Driver — Communication layer

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "OnStepXComm.h"

#include <defaultdevice.h>
#include <indilogger.h>

#include <cstring>
#include <ctime>
#include <unistd.h>
#include <sys/select.h>

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void OnStepXComm::setFd(int fd)
{
    m_fd = fd;
}

void OnStepXComm::setDevice(INDI::DefaultDevice *dev)
{
    m_dev = dev;
}

// Send '#'-terminated command; read '#'-terminated reply.
// buf must be at least 256 bytes.
bool OnStepXComm::sendCommand(const char *cmd, char *reply, int timeout_ms, bool quiet)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_fd < 0)
    {
        LOG_ERROR("sendCommand: fd not set");
        return false;
    }

    doFlush();

    if (!writeCommand(cmd))
    {
        if (quiet)
        {
            LOGF_DEBUG("sendCommand: write failed for cmd '%s'", cmd);
        }
        else
        {
            LOGF_ERROR("sendCommand: write failed for cmd '%s'", cmd);
        }
        return false;
    }

    LOGF_DEBUG("CMD: %s", cmd);

    reply[0] = '\0';
    if (!readReply(reply, 256, timeout_ms))
    {
        if (quiet)
        {
            LOGF_DEBUG("sendCommand: no reply for cmd '%s'", cmd);
        }
        else
        {
            LOGF_ERROR("sendCommand: no reply for cmd '%s'", cmd);
        }
        return false;
    }

    LOGF_DEBUG("REPLY: %s", reply);
    return true;
}

// Send command; no reply expected or needed.
bool OnStepXComm::sendCommandBlind(const char *cmd)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_fd < 0)
    {
        LOG_ERROR("sendCommandBlind: fd not set");
        return false;
    }

    if (!writeCommand(cmd))
    {
        LOGF_ERROR("sendCommandBlind: write failed for cmd '%s'", cmd);
        return false;
    }

    LOGF_DEBUG("CMD (blind): %s", cmd);
    return true;
}

// Send command; read a single-char reply (no '#' terminator).
bool OnStepXComm::sendCommandSingleChar(const char *cmd, char &reply, int timeout_ms, bool quiet)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_fd < 0)
    {
        LOG_ERROR("sendCommandSingleChar: fd not set");
        return false;
    }

    doFlush();

    if (!writeCommand(cmd))
    {
        if (quiet)
        {
            LOGF_DEBUG("sendCommandSingleChar: write failed for cmd '%s'", cmd);
        }
        else
        {
            LOGF_ERROR("sendCommandSingleChar: write failed for cmd '%s'", cmd);
        }
        return false;
    }

    LOGF_DEBUG("CMD (single-char): %s", cmd);

    fd_set rfd;
    FD_ZERO(&rfd);
    FD_SET(m_fd, &rfd);
    struct timeval tv
    {
        timeout_ms / 1000, (timeout_ms % 1000) * 1000L
    };

    if (select(m_fd + 1, &rfd, nullptr, nullptr, &tv) <= 0)
    {
        LOGF_ERROR("sendCommandSingleChar: no reply for cmd '%s'", cmd);
        return false;
    }

    if (read(m_fd, &reply, 1) != 1)
    {
        LOGF_ERROR("sendCommandSingleChar: read failed for cmd '%s'", cmd);
        return false;
    }

    LOGF_DEBUG("REPLY (single-char): %c (0x%02X)", reply, (unsigned char)reply);
    return true;
}

// Drain all pending input — public entry point acquires lock.
void OnStepXComm::flushIO()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    doFlush();
}

// Send command; read exactly nbytes bytes (for binary protocols with no '#' terminator).
bool OnStepXComm::sendCommandReadN(const char *cmd, uint8_t *buf, int nbytes, int timeout_ms, bool quiet)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_fd < 0)
    {
        LOG_ERROR("sendCommandReadN: fd not set");
        return false;
    }

    doFlush();

    if (!writeCommand(cmd))
    {
        if (quiet)
        {
            LOGF_DEBUG("sendCommandReadN: write failed for cmd '%s'", cmd);
        }
        else
        {
            LOGF_ERROR("sendCommandReadN: write failed for cmd '%s'", cmd);
        }
        return false;
    }

    LOGF_DEBUG("CMD (readN=%d): %s", nbytes, cmd);

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    int received = 0;
    while (received < nbytes)
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000L
                          + (now.tv_nsec - start.tv_nsec) / 1000000L;
        long remaining_ms = timeout_ms - elapsed_ms;

        if (remaining_ms <= 0)
        {
            if (quiet)
            {
                LOGF_DEBUG("sendCommandReadN: timeout after %d/%d bytes for cmd '%s'",
                          received, nbytes, cmd);
            }
            else
            {
                LOGF_ERROR("sendCommandReadN: timeout after %d/%d bytes for cmd '%s'",
                          received, nbytes, cmd);
            }
            return false;
        }

        fd_set rfd;
        FD_ZERO(&rfd);
        FD_SET(m_fd, &rfd);
        struct timeval tv
        {
            remaining_ms / 1000, (remaining_ms % 1000) * 1000L
        };

        if (select(m_fd + 1, &rfd, nullptr, nullptr, &tv) <= 0)
        {
            if (quiet)
            {
                LOGF_DEBUG("sendCommandReadN: no reply for cmd '%s'", cmd);
            }
            else
            {
                LOGF_ERROR("sendCommandReadN: no reply for cmd '%s'", cmd);
            }
            return false;
        }

        ssize_t n = read(m_fd, buf + received, nbytes - received);
        if (n <= 0)
        {
            if (quiet)
            {
                LOGF_DEBUG("sendCommandReadN: read error for cmd '%s'", cmd);
            }
            else
            {
                LOGF_ERROR("sendCommandReadN: read error for cmd '%s'", cmd);
            }
            return false;
        }
        received += static_cast<int>(n);
    }

    LOGF_DEBUG("REPLY (readN): %d bytes received", nbytes);
    return true;
}

// ---------------------------------------------------------------------------
// sendCommandFocuser / sendCommandBlindFocuser
//
// Atomically:  :FA[slot]#  (select focuser, consumes single-char reply)
//              :F[cmd]#    (actual command, reads '#'-terminated reply)
// ---------------------------------------------------------------------------
bool OnStepXComm::sendCommandFocuser(int slot, const char *cmd, char *reply, int timeout_ms)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_fd < 0)
    {
        LOG_ERROR("sendCommandFocuser: fd not set");
        return false;
    }

    doFlush();

    // Select focuser n
    char selectCmd[CMD_MAX_LEN];
    snprintf(selectCmd, sizeof(selectCmd), ":FA%d#", slot);
    if (!writeCommand(selectCmd))
    {
        LOGF_ERROR("sendCommandFocuser: select write failed (slot %d)", slot);
        return false;
    }

    // Consume single-char reply ('1' = exists, '0' = does not)
    char sel = '0';
    if (!readSingleChar(sel, 500))
    {
        LOGF_ERROR("sendCommandFocuser: no reply to :FA%d#", slot);
        return false;
    }
    if (sel != '1')
    {
        LOGF_ERROR("sendCommandFocuser: focuser %d not found (reply '%c')", slot, sel);
        return false;
    }

    // Send the actual command and read '#'-terminated reply
    if (!writeCommand(cmd))
    {
        LOGF_ERROR("sendCommandFocuser: write failed for cmd '%s'", cmd);
        return false;
    }

    reply[0] = '\0';
    if (!readReply(reply, 256, timeout_ms))
    {
        LOGF_ERROR("sendCommandFocuser: timeout for cmd '%s' (slot %d)", cmd, slot);
        return false;
    }

    LOGF_DEBUG("FOC%d CMD: %s  REPLY: %s", slot, cmd, reply);
    return true;
}

bool OnStepXComm::sendCommandBlindFocuser(int slot, const char *cmd)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_fd < 0)
    {
        LOG_ERROR("sendCommandBlindFocuser: fd not set");
        return false;
    }

    doFlush();

    // Select focuser n
    char selectCmd[CMD_MAX_LEN];
    snprintf(selectCmd, sizeof(selectCmd), ":FA%d#", slot);
    if (!writeCommand(selectCmd))
    {
        LOGF_ERROR("sendCommandBlindFocuser: select write failed (slot %d)", slot);
        return false;
    }

    // Consume single-char reply
    char sel = '0';
    if (!readSingleChar(sel, 500) || sel != '1')
    {
        LOGF_ERROR("sendCommandBlindFocuser: focuser %d not found", slot);
        return false;
    }

    if (!writeCommand(cmd))
    {
        LOGF_ERROR("sendCommandBlindFocuser: write failed for cmd '%s'", cmd);
        return false;
    }

    LOGF_DEBUG("FOC%d CMD (blind): %s", slot, cmd);
    return true;
}

// ---------------------------------------------------------------------------
// Private helpers — callers must already hold m_mutex
// ---------------------------------------------------------------------------

// Drain all pending input bytes without blocking.
void OnStepXComm::doFlush()
{
    if (m_fd < 0)
        return;

    char buf[256];
    while (true)
    {
        fd_set rfd;
        FD_ZERO(&rfd);
        FD_SET(m_fd, &rfd);
        struct timeval tv
        {
            0, 0
        };
        if (select(m_fd + 1, &rfd, nullptr, nullptr, &tv) <= 0)
            break;
        if (read(m_fd, buf, sizeof(buf)) <= 0)
            break;
    }
}

// Write all bytes of cmd to m_fd; handles short writes.
bool OnStepXComm::writeCommand(const char *cmd)
{
    size_t total   = strlen(cmd);
    size_t written = 0;

    while (written < total)
    {
        ssize_t n = write(m_fd, cmd + written, total - written);
        if (n <= 0)
            return false;
        written += static_cast<size_t>(n);
    }
    return true;
}

// Read bytes into buf until '#' received, inter-character timeout or timeout/buffer-full.
// Stores reply without the trailing '#'. buf must be no more than maxLen bytes.
bool OnStepXComm::readReply(char *buf, int maxLen, int timeout_ms, int inter_char_ms)
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    int len = 0;
    while (len < maxLen - 1)
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000L
                          + (now.tv_nsec - start.tv_nsec) / 1000000L;
        long remaining_ms = timeout_ms - elapsed_ms;
        if (remaining_ms <= 0)
            break;

        // Once we have data, use inter-char timeout instead of overall remaining
        long wait_ms = (len > 0) ? inter_char_ms : remaining_ms;

        fd_set rfd;
        FD_ZERO(&rfd);
        FD_SET(m_fd, &rfd);
        struct timeval tv
        {
            wait_ms / 1000, (wait_ms % 1000) * 1000L
        };

        int sel = select(m_fd + 1, &rfd, nullptr, nullptr, &tv);
        if (sel <= 0)
        {
            // If we have data and inter-char timeout fired, reply is complete
            if (len > 0)
                break;
            // No data at all — I/O failure
            buf[0] = '\0';
            return false;
        }

        char c;
        if (read(m_fd, &c, 1) != 1)
        {
            buf[len] = '\0';
            return false;
        }

        if (c == '#')
        {
            buf[len] = '\0';
            return true;
        }

        buf[len++] = c;
    }

    buf[len] = '\0';
    // Valid if we accumulated something
    return len > 0;
}

// Read exactly one byte with a timeout; no locking (caller holds mutex).
bool OnStepXComm::readSingleChar(char &c, int timeout_ms)
{
    fd_set rfd;
    FD_ZERO(&rfd);
    FD_SET(m_fd, &rfd);
    struct timeval tv
    {
        timeout_ms / 1000, (timeout_ms % 1000) * 1000L
    };

    if (select(m_fd + 1, &rfd, nullptr, nullptr, &tv) <= 0)
        return false;

    return read(m_fd, &c, 1) == 1;
}
