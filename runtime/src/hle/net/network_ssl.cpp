#include "network_internal.h"

#if defined(__APPLE__)
// SecureTransport is the one Apple TLS API that (a) runs over a socket the caller owns, which
// the IOS SSL model requires (the game opens the TCP socket, then hands it to /dev/net/ssl),
// and (b) still negotiates TLS 1.0/1.1 - the only versions the Wii-era servers this HLE talks
// to (the Wiimmfi payload host answers TLS 1.1 at most) accept. It is deprecated, not removed.
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#include <CoreFoundation/CoreFoundation.h>
#include <Security/SecureTransport.h>
#include <Security/Security.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <cerrno>
#endif

namespace NetworkHle {

enum SslError {
    SSL_OK = 0,
    SSL_ERR_FAILED = -1,
    SSL_ERR_RAGAIN = -2,
    SSL_ERR_WAGAIN = -3,
    SSL_ERR_SYSCALL = -5,
    SSL_ERR_ZERO = -6,
    SSL_ERR_ID = -8,
    SSL_ERR_VCOMMONNAME = -9,
};

enum SslIoctlv {
    IOCTLV_NET_SSL_NEW = 0x01,
    IOCTLV_NET_SSL_CONNECT = 0x02,
    IOCTLV_NET_SSL_DOHANDSHAKE = 0x03,
    IOCTLV_NET_SSL_READ = 0x04,
    IOCTLV_NET_SSL_WRITE = 0x05,
    IOCTLV_NET_SSL_SHUTDOWN = 0x06,
    IOCTLV_NET_SSL_SETCLIENTCERT = 0x07,
    IOCTLV_NET_SSL_SETCLIENTCERTDEFAULT = 0x08,
    IOCTLV_NET_SSL_REMOVECLIENTCERT = 0x09,
    IOCTLV_NET_SSL_SETROOTCA = 0x0A,
    IOCTLV_NET_SSL_SETROOTCADEFAULT = 0x0B,
    IOCTLV_NET_SSL_DOHANDSHAKEEX = 0x0C,
    IOCTLV_NET_SSL_SETBUILTINROOTCA = 0x0D,
    IOCTLV_NET_SSL_SETBUILTINCLIENTCERT = 0x0E,
    IOCTLV_NET_SSL_DISABLEVERIFYOPTIONFORDEBUG = 0x0F,
    IOCTLV_NET_SSL_DEBUGGETVERSION = 0x14,
    IOCTLV_NET_SSL_DEBUGGETTIME = 0x15,
};

constexpr int kMaxSslSessions = 4;

struct SslSession {
    bool active = false;
    bool handshaked = false;
    bool plaintextWfc = false;
    uint32_t socketFd = UINT32_MAX;
    NativeSocket native = kInvalidSocket;
    std::string hostname;
    std::vector<uint8_t> nasWriteBuffer;
    std::vector<uint8_t> decrypted;
    std::vector<uint8_t> encryptedExtra;
    // Failure-report one-shots; cleared with the session by ClearSslSession. The
    // handshake re-runs on every read/write, so a failing one repeats forever.
    bool loggedHandshakeFail = false;
    int32_t lastLoggedReadError = 0;
    int32_t lastLoggedWriteError = 0;
#ifdef _WIN32
    bool haveCred = false;
    bool haveContext = false;
    CredHandle cred{};
    CtxtHandle context{};
    SecPkgContext_StreamSizes sizes{};
#elif defined(__APPLE__)
    SSLContextRef context = nullptr;
    // Root certificates the game installed with SSL_SETROOTCA (DER); the peer chain is verified
    // against exactly these when any were given, as IOS does. The built-in/default roots are
    // Nintendo's own CA set, which is not available here, so those calls leave this empty and
    // the chain is accepted unverified - the same policy the Windows backend applies to all.
    std::vector<std::vector<uint8_t>> rootCaDer;
#endif
};

static std::array<SslSession, kMaxSslSessions> g_sslSessions;

// Logging wrapper around the platform handshake implementation; see below.
static int32_t SslHandshake(SslSession& ssl);

static bool IsRetroNasSslHost(std::string_view hostname) {
    if (!RetroRewindProfileActive()) {
        return false;
    }
    const std::string lowered = Lower(hostname);
    return StartsWith(lowered, "nas.") || StartsWith(lowered, "naswii.");
}

static bool IsRetroPlaintextSslHost(std::string_view hostname) {
    if (IsRetroNasSslHost(hostname)) {
        return true;
    }
    if (!RetroRewindProfileActive()) {
        return false;
    }

    const std::string lowered = Lower(hostname);
    return StartsWith(lowered, "sake.gs.") ||
           lowered.find(".sake.gs.") != std::string::npos ||
           StartsWith(lowered, "gamestats.gs.") ||
           lowered.find(".gamestats.gs.") != std::string::npos ||
           StartsWith(lowered, "gamestats2.gs.") ||
           lowered.find(".gamestats2.gs.") != std::string::npos ||
           StartsWith(lowered, "race.gs.") ||
           lowered.find(".race.gs.") != std::string::npos;
}

static std::optional<size_t> ParseHttpContentLength(std::string_view headers) {
    std::optional<size_t> parsedLength;
    size_t lineStart = 0;
    while (lineStart <= headers.size()) {
        const size_t lineEnd = headers.find("\r\n", lineStart);
        const std::string_view line = headers.substr(
            lineStart, lineEnd == std::string_view::npos ? headers.size() - lineStart : lineEnd - lineStart);
        const size_t colon = line.find(':');
        if (colon != std::string_view::npos && Lower(line.substr(0, colon)) == "content-length") {
            size_t valueStart = colon + 1;
            while (valueStart < line.size() && (line[valueStart] == ' ' || line[valueStart] == '\t')) {
                ++valueStart;
            }
            size_t valueEnd = line.size();
            while (valueEnd > valueStart && (line[valueEnd - 1] == ' ' || line[valueEnd - 1] == '\t')) {
                --valueEnd;
            }
            size_t value = 0;
            const auto [end, error] = std::from_chars(
                line.data() + valueStart, line.data() + valueEnd, value, 10);
            if (error != std::errc{} || end != line.data() + valueEnd ||
                (parsedLength && *parsedLength != value)) {
                return std::nullopt;
            }
            parsedLength = value;
        }
        if (lineEnd == std::string_view::npos) {
            break;
        }
        lineStart = lineEnd + 2;
    }
    return parsedLength;
}

// The Retro-WFC server rejects a NAS "POST /ac" auth body split across TCP
// segments, so buffer guest chunks until Content-Length is satisfied, then
// flush as one write. Flush at 16 KiB if it never terminates, and immediately
// if Content-Length can't be parsed.
static NasSslWriteAction AccumulateNasRequest(std::vector<uint8_t>& buffer, const uint8_t* data,
                                              uint32_t size, std::vector<uint8_t>& patched) {
    buffer.insert(buffer.end(), data, data + size);
    if (buffer.size() > 16 * 1024) {
        patched.swap(buffer);
        buffer.clear();
        return NasSslWriteAction::Ready;
    }

    const std::string accumulated(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    const size_t headerEnd = accumulated.find("\r\n\r\n");
    if (headerEnd == std::string::npos) {
        return NasSslWriteAction::Buffered;
    }

    const std::optional<size_t> contentLength =
        ParseHttpContentLength(std::string_view(accumulated).substr(0, headerEnd));
    if (!contentLength) {
        patched.swap(buffer);
        buffer.clear();
        return NasSslWriteAction::Ready;
    }

    const size_t requestSize = headerEnd + 4 + *contentLength;
    if (accumulated.size() < requestSize) {
        return NasSslWriteAction::Buffered;
    }

    patched.assign(buffer.begin(), buffer.begin() + requestSize);
    if (accumulated.size() > requestSize) {
        patched.insert(patched.end(), buffer.begin() + requestSize, buffer.end());
    }
    buffer.clear();
    return NasSslWriteAction::Ready;
}

static bool StartsNasAuthRequest(const uint8_t* data, uint32_t size) {
    return size >= 9 && std::memcmp(data, "POST /ac ", 9) == 0;
}

// SSL route: the session carries the hostname the guest asked for, so the NAS
// host is identified by name. Deliberately not IsRetroNasSslHost - the SSL write
// path re-assembles NAS auth regardless of which profile is active.
static NasSslWriteAction PrepareNasSslWrite(SslSession& ssl, const uint8_t* data, uint32_t size,
                                            std::vector<uint8_t>& patched) {
    if (!data || size == 0) {
        return NasSslWriteAction::PassThrough;
    }
    const std::string loweredHost = Lower(ssl.hostname);
    const bool isNasHost = StartsWith(loweredHost, "nas.") || StartsWith(loweredHost, "naswii.");
    if (!isNasHost || (!StartsNasAuthRequest(data, size) && ssl.nasWriteBuffer.empty())) {
        return NasSslWriteAction::PassThrough;
    }
    return AccumulateNasRequest(ssl.nasWriteBuffer, data, size, patched);
}

// Plain-TCP route: a rerouted 443->80 NAS connection has no hostname on the
// socket, so the peer port and the stream type are what identify it.
NasSslWriteAction PreparePlainNasTcpWrite(WiiSocket& socket, const uint8_t* data, uint32_t size,
                                                 std::vector<uint8_t>& patched) {
    if (!data || size == 0 || socket.type != SOCK_STREAM || socket.peerPort != 80) {
        return NasSslWriteAction::PassThrough;
    }
    if (!StartsNasAuthRequest(data, size) && socket.nasWriteBuffer.empty()) {
        return NasSslWriteAction::PassThrough;
    }
    return AccumulateNasRequest(socket.nasWriteBuffer, data, size, patched);
}

static bool WriteSslReturn(const std::vector<IoVector>& in, int32_t value) {
    if (in.empty() || !in[0].address || in[0].size < 4) {
        return false;
    }
    Memory::Write32(in[0].address, static_cast<uint32_t>(value));
    return true;
}

static int ReadSslId(const std::vector<IoVector>& out) {
    if (out.empty() || !out[0].address || out[0].size < 4) {
        return -1;
    }
    return static_cast<int>(Memory::Read32(out[0].address)) - 1;
}

static bool IsSslIdValid(int id) {
    return id >= 0 && id < kMaxSslSessions && g_sslSessions[id].active;
}

#ifdef _WIN32
static bool IsSecuritySuccess(SECURITY_STATUS status) {
    return status == SEC_E_OK || status == SEC_I_CONTINUE_NEEDED || status == SEC_I_INCOMPLETE_CREDENTIALS;
}

static bool SendAll(NativeSocket socket, const uint8_t* data, size_t size) {
    size_t offset = 0;
    while (offset < size) {
        const int chunk = static_cast<int>(std::min<size_t>(size - offset, 64 * 1024));
        const int ret = send(socket, reinterpret_cast<const char*>(data + offset), chunk, 0);
        if (ret <= 0) {
            // Reported by the SSL_WRITE / handshake caller as SSL_ERR_SYSCALL.
            return false;
        }
        offset += static_cast<size_t>(ret);
    }
    return true;
}

static int RecvBlocking(NativeSocket socket, std::vector<uint8_t>& buffer) {
    std::array<uint8_t, 16 * 1024> temp{};
    const int ret = recv(socket, reinterpret_cast<char*>(temp.data()), static_cast<int>(temp.size()), 0);
    if (ret > 0) {
        buffer.insert(buffer.end(), temp.begin(), temp.begin() + ret);
    }
    return ret;
}

static void KeepExtraBuffer(std::vector<uint8_t>& dest, const SecBuffer& buffer) {
    dest.clear();
    if (buffer.BufferType == SECBUFFER_EXTRA && buffer.pvBuffer && buffer.cbBuffer) {
        const auto* begin = static_cast<const uint8_t*>(buffer.pvBuffer);
        dest.assign(begin, begin + buffer.cbBuffer);
    }
}

static void ClearSslSession(SslSession& ssl) {
    if (ssl.haveContext) {
        DeleteSecurityContext(&ssl.context);
    }
    if (ssl.haveCred) {
        FreeCredentialsHandle(&ssl.cred);
    }
    ssl = {};
}

static int32_t EnsureSslCredentials(SslSession& ssl) {
    if (ssl.haveCred) {
        return SSL_OK;
    }

    SCHANNEL_CRED cred{};
    cred.dwVersion = SCHANNEL_CRED_VERSION;
    cred.dwFlags = SCH_USE_STRONG_CRYPTO | SCH_CRED_NO_DEFAULT_CREDS | SCH_CRED_MANUAL_CRED_VALIDATION;

    TimeStamp expiry{};
    const SECURITY_STATUS status = AcquireCredentialsHandleA(
        nullptr, const_cast<LPSTR>(UNISP_NAME_A), SECPKG_CRED_OUTBOUND, nullptr, &cred, nullptr, nullptr,
        &ssl.cred, &expiry);
    if (status != SEC_E_OK) {
        return SSL_ERR_FAILED;
    }
    ssl.haveCred = true;
    return SSL_OK;
}

static int32_t SslHandshakeImpl(SslSession& ssl) {
    if (ssl.plaintextWfc) {
        ssl.handshaked = true;
        return SSL_OK;
    }

    const int32_t credRet = EnsureSslCredentials(ssl);
    if (credRet != SSL_OK) {
        return credRet;
    }
    if (ssl.native == kInvalidSocket) {
        return SSL_ERR_SYSCALL;
    }
    if (ssl.handshaked) {
        return SSL_OK;
    }

    DWORD attrs = 0;
    TimeStamp expiry{};
    std::vector<uint8_t> incoming = std::move(ssl.encryptedExtra);
    ssl.encryptedExtra.clear();
    constexpr DWORD flags = ISC_REQ_SEQUENCE_DETECT | ISC_REQ_REPLAY_DETECT | ISC_REQ_CONFIDENTIALITY |
                            ISC_REQ_ALLOCATE_MEMORY | ISC_REQ_STREAM | ISC_REQ_EXTENDED_ERROR;

    for (int step = 0; step < 128; ++step) {
        SecBuffer outBuffer{};
        outBuffer.BufferType = SECBUFFER_TOKEN;
        SecBufferDesc outDesc{};
        outDesc.ulVersion = SECBUFFER_VERSION;
        outDesc.cBuffers = 1;
        outDesc.pBuffers = &outBuffer;

        SecBuffer inBuffers[2]{};
        SecBufferDesc inDesc{};
        SecBufferDesc* inDescPtr = nullptr;
        if (!incoming.empty()) {
            inBuffers[0].BufferType = SECBUFFER_TOKEN;
            inBuffers[0].pvBuffer = incoming.data();
            inBuffers[0].cbBuffer = static_cast<unsigned long>(incoming.size());
            inBuffers[1].BufferType = SECBUFFER_EMPTY;
            inDesc.ulVersion = SECBUFFER_VERSION;
            inDesc.cBuffers = 2;
            inDesc.pBuffers = inBuffers;
            inDescPtr = &inDesc;
        }

        const SECURITY_STATUS status = InitializeSecurityContextA(
            &ssl.cred, ssl.haveContext ? &ssl.context : nullptr,
            ssl.hostname.empty() ? nullptr : const_cast<char*>(ssl.hostname.c_str()), flags, 0, SECURITY_NATIVE_DREP,
            inDescPtr, 0, &ssl.context, &outDesc, &attrs, &expiry);
        if (status != SEC_E_INVALID_HANDLE) {
            ssl.haveContext = true;
        }

        if (outBuffer.pvBuffer && outBuffer.cbBuffer) {
            const bool sent = SendAll(ssl.native, static_cast<const uint8_t*>(outBuffer.pvBuffer), outBuffer.cbBuffer);
            FreeContextBuffer(outBuffer.pvBuffer);
            if (!sent) {
                return SSL_ERR_SYSCALL;
            }
        }

        if (status == SEC_E_OK) {
            if (inDescPtr) {
                KeepExtraBuffer(ssl.encryptedExtra, inBuffers[1]);
            }
            const SECURITY_STATUS sizeStatus =
                QueryContextAttributesA(&ssl.context, SECPKG_ATTR_STREAM_SIZES, &ssl.sizes);
            if (sizeStatus != SEC_E_OK) {
                return SSL_ERR_FAILED;
            }
            ssl.handshaked = true;
            return SSL_OK;
        }

        if (status == SEC_E_INCOMPLETE_MESSAGE) {
            const int ret = RecvBlocking(ssl.native, incoming);
            if (ret == 0) {
                return SSL_ERR_ZERO;
            }
            if (ret < 0) {
                return SSL_ERR_RAGAIN;
            }
            continue;
        }

        if (status == SEC_I_CONTINUE_NEEDED || status == SEC_I_INCOMPLETE_CREDENTIALS) {
            std::vector<uint8_t> extra;
            if (inDescPtr) {
                KeepExtraBuffer(extra, inBuffers[1]);
            }
            incoming = std::move(extra);
            const int ret = RecvBlocking(ssl.native, incoming);
            if (ret == 0) {
                return SSL_ERR_ZERO;
            }
            if (ret < 0) {
                return SSL_ERR_RAGAIN;
            }
            continue;
        }

        return status == SEC_E_WRONG_PRINCIPAL ? SSL_ERR_VCOMMONNAME : SSL_ERR_FAILED;
    }

    return SSL_ERR_FAILED;
}

static int32_t SslWrite(SslSession& ssl, const uint8_t* data, uint32_t size) {
    if (!data || size == 0) {
        return SSL_ERR_ZERO;
    }
    const int32_t handshakeRet = SslHandshake(ssl);
    if (handshakeRet != SSL_OK) {
        return handshakeRet;
    }

    if (ssl.plaintextWfc) {
        return SendAll(ssl.native, data, size) ? static_cast<int32_t>(size) : SSL_ERR_SYSCALL;
    }

    uint32_t total = 0;
    while (total < size) {
        const uint32_t chunk = std::min<uint32_t>(size - total, ssl.sizes.cbMaximumMessage);
        std::vector<uint8_t> packet(ssl.sizes.cbHeader + chunk + ssl.sizes.cbTrailer);
        std::memcpy(packet.data() + ssl.sizes.cbHeader, data + total, chunk);

        SecBuffer buffers[4]{};
        buffers[0].BufferType = SECBUFFER_STREAM_HEADER;
        buffers[0].pvBuffer = packet.data();
        buffers[0].cbBuffer = ssl.sizes.cbHeader;
        buffers[1].BufferType = SECBUFFER_DATA;
        buffers[1].pvBuffer = packet.data() + ssl.sizes.cbHeader;
        buffers[1].cbBuffer = chunk;
        buffers[2].BufferType = SECBUFFER_STREAM_TRAILER;
        buffers[2].pvBuffer = packet.data() + ssl.sizes.cbHeader + chunk;
        buffers[2].cbBuffer = ssl.sizes.cbTrailer;
        buffers[3].BufferType = SECBUFFER_EMPTY;

        SecBufferDesc desc{};
        desc.ulVersion = SECBUFFER_VERSION;
        desc.cBuffers = 4;
        desc.pBuffers = buffers;

        const SECURITY_STATUS status = EncryptMessage(&ssl.context, 0, &desc, 0);
        if (status != SEC_E_OK) {
            return SSL_ERR_FAILED;
        }

        const size_t encryptedSize =
            static_cast<size_t>(buffers[0].cbBuffer) + buffers[1].cbBuffer + buffers[2].cbBuffer;
        if (!SendAll(ssl.native, packet.data(), encryptedSize)) {
            return SSL_ERR_SYSCALL;
        }
        total += chunk;
    }
    return static_cast<int32_t>(total);
}

static int32_t SslRead(SslSession& ssl, uint8_t* out, uint32_t size) {
    if (!out || size == 0) {
        return SSL_ERR_ZERO;
    }
    const int32_t handshakeRet = SslHandshake(ssl);
    if (handshakeRet != SSL_OK) {
        return handshakeRet;
    }

    if (ssl.plaintextWfc) {
        const int ret = recv(ssl.native, reinterpret_cast<char*>(out), static_cast<int>(size), 0);
        if (ret == 0) {
            return SSL_ERR_ZERO;
        }
        if (ret < 0) {
            return SSL_ERR_RAGAIN;
        }
        return ret;
    }

    while (ssl.decrypted.empty()) {
        std::vector<uint8_t> encrypted = std::move(ssl.encryptedExtra);
        ssl.encryptedExtra.clear();
        if (encrypted.empty()) {
            const int ret = RecvBlocking(ssl.native, encrypted);
            if (ret == 0) {
                return SSL_ERR_ZERO;
            }
            if (ret < 0) {
                return SSL_ERR_RAGAIN;
            }
        }

        for (;;) {
            SecBuffer buffers[4]{};
            buffers[0].BufferType = SECBUFFER_DATA;
            buffers[0].pvBuffer = encrypted.data();
            buffers[0].cbBuffer = static_cast<unsigned long>(encrypted.size());
            buffers[1].BufferType = SECBUFFER_EMPTY;
            buffers[2].BufferType = SECBUFFER_EMPTY;
            buffers[3].BufferType = SECBUFFER_EMPTY;

            SecBufferDesc desc{};
            desc.ulVersion = SECBUFFER_VERSION;
            desc.cBuffers = 4;
            desc.pBuffers = buffers;

            const SECURITY_STATUS status = DecryptMessage(&ssl.context, &desc, 0, nullptr);
            if (status == SEC_E_INCOMPLETE_MESSAGE) {
                const int ret = RecvBlocking(ssl.native, encrypted);
                if (ret == 0) {
                    return SSL_ERR_ZERO;
                }
                if (ret < 0) {
                    return SSL_ERR_RAGAIN;
                }
                continue;
            }
            if (status == SEC_I_CONTEXT_EXPIRED) {
                return SSL_ERR_ZERO;
            }
            if (!IsSecuritySuccess(status) && status != SEC_I_RENEGOTIATE) {
                return SSL_ERR_FAILED;
            }

            for (const SecBuffer& buffer : buffers) {
                if (buffer.BufferType == SECBUFFER_DATA && buffer.pvBuffer && buffer.cbBuffer) {
                    const auto* begin = static_cast<const uint8_t*>(buffer.pvBuffer);
                    ssl.decrypted.insert(ssl.decrypted.end(), begin, begin + buffer.cbBuffer);
                } else if (buffer.BufferType == SECBUFFER_EXTRA && buffer.pvBuffer && buffer.cbBuffer) {
                    const auto* begin = static_cast<const uint8_t*>(buffer.pvBuffer);
                    ssl.encryptedExtra.assign(begin, begin + buffer.cbBuffer);
                }
            }
            break;
        }
    }

    const uint32_t copied = std::min<uint32_t>(size, static_cast<uint32_t>(ssl.decrypted.size()));
    std::memcpy(out, ssl.decrypted.data(), copied);
    ssl.decrypted.erase(ssl.decrypted.begin(), ssl.decrypted.begin() + copied);
    return copied == 0 ? SSL_ERR_ZERO : static_cast<int32_t>(copied);
}
#elif defined(__APPLE__)
// --- SecureTransport backend -------------------------------------------------------------
// SecureTransport drives the TLS state machine and calls back into these two functions for
// raw bytes; they block on the Wii socket's native descriptor (SSL_CONNECT put it in blocking
// mode with a receive/send timeout) so SSLHandshake/SSLRead/SSLWrite complete synchronously,
// which is how the game's NHTTP thread expects the IOS calls to behave.
static OSStatus SslSocketRead(SSLConnectionRef connection, void* data, size_t* length) {
    const auto fd = static_cast<int>(reinterpret_cast<intptr_t>(connection));
    const size_t wanted = *length;
    size_t got = 0;
    while (got < wanted) {
        const ssize_t n = recv(fd, static_cast<uint8_t*>(data) + got, wanted - got, 0);
        if (n > 0) {
            got += static_cast<size_t>(n);
            continue;
        }
        *length = got;
        if (n == 0) {
            return errSSLClosedGraceful;
        }
        if (errno == EINTR) {
            continue;
        }
        return (errno == EAGAIN || errno == EWOULDBLOCK) ? errSSLWouldBlock : errSSLClosedAbort;
    }
    *length = got;
    return noErr;
}

static OSStatus SslSocketWrite(SSLConnectionRef connection, const void* data, size_t* length) {
    const auto fd = static_cast<int>(reinterpret_cast<intptr_t>(connection));
    const size_t wanted = *length;
    size_t sent = 0;
    while (sent < wanted) {
        const ssize_t n = send(fd, static_cast<const uint8_t*>(data) + sent, wanted - sent, 0);
        if (n > 0) {
            sent += static_cast<size_t>(n);
            continue;
        }
        *length = sent;
        if (n < 0 && errno == EINTR) {
            continue;
        }
        return (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) ? errSSLWouldBlock : errSSLClosedAbort;
    }
    *length = sent;
    return noErr;
}

static void ClearSslSession(SslSession& ssl) {
    if (ssl.context != nullptr) {
        SSLClose(ssl.context);
        CFRelease(ssl.context);
    }
    ssl = {};
}

// The peer chain against the roots the game installed. Returns true when the chain is
// anchored in one of them (or when the game installed none - see SslSession::rootCaDer).
static bool VerifyPeerAgainstGameRoots(SslSession& ssl) {
    if (ssl.rootCaDer.empty()) {
        return true;
    }
    SecTrustRef trust = nullptr;
    if (SSLCopyPeerTrust(ssl.context, &trust) != noErr || trust == nullptr) {
        return false;
    }
    CFMutableArrayRef anchors = CFArrayCreateMutable(kCFAllocatorDefault, 0, &kCFTypeArrayCallBacks);
    for (const std::vector<uint8_t>& der : ssl.rootCaDer) {
        CFDataRef data = CFDataCreate(kCFAllocatorDefault, der.data(), static_cast<CFIndex>(der.size()));
        SecCertificateRef cert = data ? SecCertificateCreateWithData(kCFAllocatorDefault, data) : nullptr;
        if (cert) {
            CFArrayAppendValue(anchors, cert);
            CFRelease(cert);
        }
        if (data) {
            CFRelease(data);
        }
    }
    bool ok = false;
    if (CFArrayGetCount(anchors) > 0 &&
        SecTrustSetAnchorCertificates(trust, anchors) == noErr &&
        SecTrustSetAnchorCertificatesOnly(trust, true) == noErr) {
        ok = SecTrustEvaluateWithError(trust, nullptr);
    }
    CFRelease(anchors);
    CFRelease(trust);
    return ok;
}

static int32_t SslHandshakeImpl(SslSession& ssl) {
    if (ssl.plaintextWfc) {
        ssl.handshaked = true;
        return SSL_OK;
    }
    if (ssl.native == kInvalidSocket) {
        return SSL_ERR_SYSCALL;
    }
    if (ssl.handshaked) {
        return SSL_OK;
    }
    if (ssl.context == nullptr) {
        ssl.context = SSLCreateContext(kCFAllocatorDefault, kSSLClientSide, kSSLStreamType);
        if (ssl.context == nullptr) {
            return SSL_ERR_FAILED;
        }
        if (SSLSetIOFuncs(ssl.context, SslSocketRead, SslSocketWrite) != noErr ||
            SSLSetConnection(ssl.context, reinterpret_cast<SSLConnectionRef>(static_cast<intptr_t>(ssl.native))) != noErr ||
            // IOS speaks TLS 1.0; the servers it was written for stop at 1.1. Offer 1.0-1.2 so a
            // modern host still gets the best it supports.
            SSLSetProtocolVersionMin(ssl.context, kTLSProtocol1) != noErr ||
            SSLSetProtocolVersionMax(ssl.context, kTLSProtocol12) != noErr ||
            SSLSetSessionOption(ssl.context, kSSLSessionOptionBreakOnServerAuth, true) != noErr) {
            return SSL_ERR_FAILED;
        }
        if (!ssl.hostname.empty() &&
            SSLSetPeerDomainName(ssl.context, ssl.hostname.c_str(), ssl.hostname.size()) != noErr) {
            return SSL_ERR_FAILED;
        }
    }
    for (;;) {
        const OSStatus status = SSLHandshake(ssl.context);
        if (status == noErr) {
            ssl.handshaked = true;
            return SSL_OK;
        }
        if (status == errSSLServerAuthCompleted) {
            if (!VerifyPeerAgainstGameRoots(ssl)) {
                return SSL_ERR_FAILED;
            }
            continue;
        }
        if (status == errSSLWouldBlock) {
            continue;
        }
        return status == errSSLClosedGraceful || status == errSSLClosedAbort || status == errSSLClosedNoNotify
                   ? SSL_ERR_SYSCALL
                   : SSL_ERR_FAILED;
    }
}

static int32_t SslWrite(SslSession& ssl, const uint8_t* data, uint32_t size) {
    if (!data || size == 0) {
        return SSL_ERR_ZERO;
    }
    const int32_t handshakeRet = SslHandshake(ssl);
    if (handshakeRet != SSL_OK) {
        return handshakeRet;
    }
    if (ssl.plaintextWfc) {
        uint32_t sent = 0;
        while (sent < size) {
            const ssize_t n = send(ssl.native, data + sent, size - sent, 0);
            if (n <= 0) {
                return SSL_ERR_SYSCALL;
            }
            sent += static_cast<uint32_t>(n);
        }
        return static_cast<int32_t>(size);
    }
    uint32_t total = 0;
    while (total < size) {
        size_t processed = 0;
        const OSStatus status = SSLWrite(ssl.context, data + total, size - total, &processed);
        total += static_cast<uint32_t>(processed);
        if (status == noErr) {
            continue;
        }
        if (status == errSSLWouldBlock) {
            continue;  // the socket callback blocks, so this only means "partial progress"
        }
        return status == errSSLClosedGraceful || status == errSSLClosedAbort || status == errSSLClosedNoNotify
                   ? SSL_ERR_SYSCALL
                   : SSL_ERR_FAILED;
    }
    return static_cast<int32_t>(total);
}

static int32_t SslRead(SslSession& ssl, uint8_t* out, uint32_t size) {
    if (!out || size == 0) {
        return SSL_ERR_ZERO;
    }
    const int32_t handshakeRet = SslHandshake(ssl);
    if (handshakeRet != SSL_OK) {
        return handshakeRet;
    }
    if (ssl.plaintextWfc) {
        const ssize_t ret = recv(ssl.native, out, size, 0);
        if (ret == 0) {
            return SSL_ERR_ZERO;
        }
        if (ret < 0) {
            return SSL_ERR_RAGAIN;
        }
        return static_cast<int32_t>(ret);
    }
    size_t processed = 0;
    const OSStatus status = SSLRead(ssl.context, out, size, &processed);
    if (processed > 0) {
        // Data first: a close alert that arrives with the last record is reported on the next
        // call, exactly as IOS reports it (the SDK reads until SSL_ERR_ZERO).
        return static_cast<int32_t>(processed);
    }
    if (status == noErr) {
        return SSL_ERR_ZERO;
    }
    if (status == errSSLWouldBlock) {
        return SSL_ERR_RAGAIN;
    }
    if (status == errSSLClosedGraceful || status == errSSLClosedNoNotify) {
        return SSL_ERR_ZERO;
    }
    return SSL_ERR_FAILED;
}
#else
static void ClearSslSession(SslSession& ssl) {
    ssl = {};
}

static int32_t SslHandshakeImpl(SslSession&) {
    return SSL_ERR_FAILED;
}

static int32_t SslWrite(SslSession&, const uint8_t*, uint32_t) {
    return SSL_ERR_FAILED;
}

static int32_t SslRead(SslSession&, uint8_t*, uint32_t) {
    return SSL_ERR_FAILED;
}
#endif

// The handshake runs on every SSL read/write, so a failure repeats for as long
// as the session lives; report only the first one.
static int32_t SslHandshake(SslSession& ssl) {
    const int32_t result = SslHandshakeImpl(ssl);
    if (result != SSL_OK && !ssl.loggedHandshakeFail) {
        ssl.loggedHandshakeFail = true;
        NetFail("ssl handshake FAILED host=%s ssl_err=%d",
                ssl.hostname.empty() ? "?" : ssl.hostname.c_str(), result);
    }
    return result;
}

void ClearSslSessionsForSocket(uint32_t fd) {
    for (SslSession& ssl : g_sslSessions) {
        if (ssl.active && ssl.socketFd == fd) {
            ClearSslSession(ssl);
        }
    }
}

int32_t HandleSslIoctlv(uint32_t cmd, const std::vector<IoVector>& in, const std::vector<IoVector>& out) {

    switch (cmd) {
    case IOCTLV_NET_SSL_NEW: {
        // out[0] carries the verify option. Its value is not consulted - every
        // certificate check is delegated to Schannel with
        // SCH_CRED_MANUAL_CRED_VALIDATION - but it is still read so that a
        // request pointing outside guest memory faults here as it always has.
        if (!out.empty() && out[0].address && out[0].size >= 4) {
            (void)Memory::Read32(out[0].address);
        }
        std::string hostname = out.size() > 1 ? ReadGuestString(out[1].address, out[1].size) : "";

        for (int i = 0; i < kMaxSslSessions; ++i) {
            if (!g_sslSessions[i].active) {
                ClearSslSession(g_sslSessions[i]);
                g_sslSessions[i].active = true;
                g_sslSessions[i].hostname = hostname;
                WriteSslReturn(in, i + 1);
                return 0;
            }
        }
        WriteSslReturn(in, SSL_ERR_FAILED);
        NetFail("SSL_NEW host=%s FAILED: all %d session slots in use",
                hostname.empty() ? "?" : hostname.c_str(), kMaxSslSessions);
        return 0;
    }
    case IOCTLV_NET_SSL_CONNECT: {
        const int sslId = ReadSslId(out);
        if (!IsSslIdValid(sslId)) {
            WriteSslReturn(in, SSL_ERR_ID);
            return 0;
        }
        if (out.size() < 2 || !out[1].address || out[1].size < 4) {
            WriteSslReturn(in, SSL_ERR_FAILED);
            return 0;
        }
        const uint32_t socketFd = Memory::Read32(out[1].address);
        WiiSocket* socket = GetWiiSocket(socketFd);
        if (!socket) {
            WriteSslReturn(in, SSL_ERR_SYSCALL);
            return 0;
        }

        SslSession& ssl = g_sslSessions[sslId];
        ssl.socketFd = socketFd;
        ssl.native = socket->native;
        ssl.plaintextWfc = false;
        socket->nonblocking = false;
        SetNonBlocking(socket->native, false);
        if (IsRetroPlaintextSslHost(ssl.hostname) && socket->peerPort == 443) {
            const int32_t reroute = ReconnectWiiSocket(*socket, 80);
            if (reroute != 0) {
                WriteSslReturn(in, SSL_ERR_SYSCALL);
                NetFail("SSL_CONNECT host=%s 443->80 plaintext reroute FAILED wii=%d",
                        ssl.hostname.c_str(), reroute);
                return 0;
            }
            ssl.native = socket->native;
            ssl.plaintextWfc = true;
        }
#ifdef _WIN32
        const int timeoutMs = 15000;
        setsockopt(socket->native, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeoutMs), sizeof(timeoutMs));
        setsockopt(socket->native, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&timeoutMs), sizeof(timeoutMs));
#elif defined(__APPLE__)
        // Same 15 s bound as Windows: the SecureTransport callbacks block on this socket.
        const timeval timeout{15, 0};
        setsockopt(socket->native, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        setsockopt(socket->native, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
#endif
        WriteSslReturn(in, SSL_OK);
        return 0;
    }
    case IOCTLV_NET_SSL_DOHANDSHAKE:
    case IOCTLV_NET_SSL_DOHANDSHAKEEX: {
        const int sslId = ReadSslId(out);
        WriteSslReturn(in, IsSslIdValid(sslId) ? SslHandshake(g_sslSessions[sslId]) : SSL_ERR_ID);
        return 0;
    }
    case IOCTLV_NET_SSL_WRITE: {
        const int sslId = ReadSslId(out);
        if (!IsSslIdValid(sslId)) {
            WriteSslReturn(in, SSL_ERR_ID);
            return 0;
        }
        if (out.size() < 2 || !out[1].address) {
            WriteSslReturn(in, SSL_ERR_FAILED);
            return 0;
        }
        const auto* data = Memory::GetPointer(out[1].address, out[1].size);
        std::vector<uint8_t> patched;
        const NasSslWriteAction nasAction = PrepareNasSslWrite(g_sslSessions[sslId], data, out[1].size, patched);
        if (nasAction == NasSslWriteAction::Buffered) {
            WriteSslReturn(in, static_cast<int32_t>(out[1].size));
            return 0;
        }

        const bool patchedSslWrite = nasAction == NasSslWriteAction::Ready;

        const uint8_t* writeData = patchedSslWrite ? patched.data() : data;
        const uint32_t writeSize = patchedSslWrite ? static_cast<uint32_t>(patched.size()) : out[1].size;
        SslSession& writeSession = g_sslSessions[sslId];
        int32_t result = SslWrite(writeSession, writeData, writeSize);
        if (patchedSslWrite && result == static_cast<int32_t>(writeSize)) {
            result = static_cast<int32_t>(out[1].size);
        }
        // SSL_ERR_WAGAIN is a retry, not a failure; the SDK re-issues the write,
        // so only a change of error is reported.
        if (result < 0 && result != SSL_ERR_WAGAIN && result != writeSession.lastLoggedWriteError) {
            writeSession.lastLoggedWriteError = result;
            NetFail("SSL_WRITE host=%s size=%u FAILED ssl_err=%d",
                    writeSession.hostname.empty() ? "?" : writeSession.hostname.c_str(), writeSize,
                    result);
        }
        WriteSslReturn(in, result);
        return 0;
    }
    case IOCTLV_NET_SSL_READ: {
        const int sslId = ReadSslId(out);
        if (!IsSslIdValid(sslId)) {
            WriteSslReturn(in, SSL_ERR_ID);
            return 0;
        }
        if (in.size() < 2 || !in[1].address) {
            WriteSslReturn(in, SSL_ERR_FAILED);
            return 0;
        }
        auto* data = Memory::GetPointer(in[1].address, in[1].size);
        SslSession& readSession = g_sslSessions[sslId];
        const int32_t result = SslRead(readSession, data, in[1].size);
        if (result < 0 && result != SSL_ERR_RAGAIN && result != readSession.lastLoggedReadError) {
            readSession.lastLoggedReadError = result;
            NetFail("SSL_READ host=%s FAILED ssl_err=%d%s",
                    readSession.hostname.empty() ? "?" : readSession.hostname.c_str(), result,
                    result == SSL_ERR_ZERO ? " (peer closed)" : "");
        }
        WriteSslReturn(in, result);
        return 0;
    }
    case IOCTLV_NET_SSL_SHUTDOWN: {
        const int sslId = ReadSslId(out);
        if (!IsSslIdValid(sslId)) {
            WriteSslReturn(in, SSL_ERR_ID);
            return 0;
        }
        ClearSslSession(g_sslSessions[sslId]);
        WriteSslReturn(in, SSL_OK);
        return 0;
    }
    case IOCTLV_NET_SSL_SETROOTCA: {
        // out[1] is the DER certificate the game installs as the trust root for this session.
        const int sslId = ReadSslId(out);
        if (!IsSslIdValid(sslId)) {
            WriteSslReturn(in, SSL_ERR_ID);
            return 0;
        }
#if defined(__APPLE__)
        if (out.size() > 1 && out[1].address && out[1].size > 0) {
            const uint8_t* der = Memory::GetPointer(out[1].address, out[1].size);
            g_sslSessions[sslId].rootCaDer.emplace_back(der, der + out[1].size);
        }
#endif
        WriteSslReturn(in, SSL_OK);
        return 0;
    }
    case IOCTLV_NET_SSL_SETCLIENTCERT:
    case IOCTLV_NET_SSL_SETCLIENTCERTDEFAULT:
    case IOCTLV_NET_SSL_REMOVECLIENTCERT:
    case IOCTLV_NET_SSL_SETROOTCADEFAULT:
    case IOCTLV_NET_SSL_SETBUILTINROOTCA:
    case IOCTLV_NET_SSL_SETBUILTINCLIENTCERT:
    case IOCTLV_NET_SSL_DISABLEVERIFYOPTIONFORDEBUG: {
        const int sslId = ReadSslId(out);
        WriteSslReturn(in, IsSslIdValid(sslId) ? SSL_OK : SSL_ERR_ID);
        return 0;
    }
    case IOCTLV_NET_SSL_DEBUGGETVERSION:
    case IOCTLV_NET_SSL_DEBUGGETTIME:
        WriteSslReturn(in, SSL_OK);
        return 0;
    default:
        WriteSslReturn(in, SSL_ERR_FAILED);
        return 0;
    }
}

}  // namespace NetworkHle

#if defined(__APPLE__)
#pragma clang diagnostic pop
#endif
