/*
    OnStep X INDI Driver — Communication layer (shared by both binaries)

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

    All LX200-framed I/O in one place.  Zero raw read()/write() calls outside
    this class.  Commands are '#'-terminated; replies are '#'-terminated except
    where noted (sendCommandSingleChar, sendCommandReadN).  A mutex serialises
    all sends and receives; flushIO() drains stale input before each send.
    Focuser-slot variants atomically send the :FA[n]# slot selector and the
    command within one mutex lock to prevent interleaving.
*/

#pragma once

#include <mutex>

namespace INDI { class DefaultDevice; }

// All LX200 framing in one place. Zero raw read()/write() calls outside this class.
class OnStepXComm
{
    public:
        void setFd(int fd);
        void setDevice(INDI::DefaultDevice *dev);

        // Send '#'-terminated command; read '#'-terminated reply into buf (must be >= 256 bytes).
        // m_mutex held for entire send+receive. flushIO called before every send.
        // quiet=true: failure messages logged at DBG_DEBUG instead of DBG_ERROR (use during probing).
        bool sendCommand(const char *cmd, char *reply, int timeout_ms = 2000, bool quiet = false);

        // Send command; discard any reply.
        bool sendCommandBlind(const char *cmd);

        // Focuser-slot variants: atomically send :FA[slot]# (consumes the single-char
        // reply), then send cmd and read '#'-terminated reply — or blind-send.
        // Both operations share one mutex lock, preventing interleaving.
        bool sendCommandFocuser(int slot, const char *cmd, char *reply, int timeout_ms = 2000);
        bool sendCommandBlindFocuser(int slot, const char *cmd);

        // Send command; read a single char reply with no '#' terminator.
        // quiet=true: failure messages logged at DBG_DEBUG instead of DBG_ERROR (use during probing).
        bool sendCommandSingleChar(const char *cmd, char &reply, int timeout_ms = 2000, bool quiet = false);

        // Send command; read exactly nbytes bytes (no '#' terminator — for binary protocols).
        // quiet=true: failure messages logged at DBG_DEBUG instead of DBG_ERROR (use during probing).
        bool sendCommandReadN(const char *cmd, uint8_t *buf, int nbytes, int timeout_ms = 2000, bool quiet = false);

        // Drain all pending input bytes from the fd.
        void flushIO();

    private:
        int                  m_fd  { -1 };
        INDI::DefaultDevice *m_dev { nullptr };
        std::mutex           m_mutex;

        void doFlush();                                        // no-lock; called from within locked context
        bool writeCommand(const char *cmd);                    // no-lock
        bool readUntilHash(char *buf, int maxLen, int timeout_ms);  // no-lock
        bool readSingleChar(char &c, int timeout_ms);          // no-lock
};
