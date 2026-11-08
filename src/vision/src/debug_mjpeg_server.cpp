#include "vision/debug_mjpeg_server.hpp"
#include "vision/interactive_debug_page.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/sockios.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <vector>
#include <opencv2/imgcodecs.hpp>

namespace vision {
namespace {
using Clock = std::chrono::steady_clock;
constexpr auto kPeriod = std::chrono::nanoseconds(66666667); // <=15 fps
constexpr auto kSendDeadline = std::chrono::seconds(1);
constexpr auto kRequestDeadline = std::chrono::seconds(3);
constexpr std::size_t kMaxClients = 8, kMaxRequest = 4096;
constexpr char kStreamHeader[] =
    "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    "Cache-Control: no-store, no-cache\r\nConnection: close\r\n\r\n";
// At most one bounded HTTP response or in-flight JPEG per client.
struct Packet { std::uint64_t sequence{}; std::vector<unsigned char> bytes; };
std::shared_ptr<const Packet> Response(const std::string &type, const std::string &body,
                                     const std::string &status = "200 OK") {
  auto packet = std::make_shared<Packet>();
  const auto text = "HTTP/1.1 " + status + "\r\nContent-Type: " + type +
      "\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: " +
      std::to_string(body.size()) + "\r\n\r\n" + body;
  packet->bytes.assign(text.begin(), text.end());
  return packet;
}
}  // namespace

struct DebugMjpegServer::Impl {
  int listener{-1}, wake_read{-1}, wake_write{-1};
  std::uint16_t port{};
  std::atomic<bool> stopping{false}, running{false};
  std::atomic<unsigned> subscribers{0};
  std::atomic<std::uint64_t> encoded{0}, sent{0}, bytes{0}, errors{0}, drops{0}, slow{0};
  std::atomic<std::uint64_t> encode_us{0}, total_encode_us{0};
  std::mutex frame_mutex, jpeg_mutex;
  std::condition_variable condition;
  cv::Mat pending;
  std::uint64_t pending_id{};
  bool interactive{};
  std::mutex input_mutex;
  std::deque<Input> inputs;
  std::shared_ptr<const Packet> latest;
  std::thread encoder, network;
  Clock::time_point last_request{};

  struct Client {
    int fd{-1};
    bool streaming{false}, stream_header_sent{false}, close_after_send{false};
    bool waiting_drain{false};
    std::string request;
    std::shared_ptr<const Packet> packet;
    std::size_t offset{};
    std::uint64_t last_sequence{};
    Clock::time_point accepted{Clock::now()}, send_started{};
  };

  Impl(std::uint16_t requested_port, const char *address, bool accept_input)
      : interactive(accept_input) {
    try {
      listener = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
      if (listener < 0) Fail("socket");
      int reuse = 1;
      if (setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
        Fail("SO_REUSEADDR");
      sockaddr_in bind_to{};
      bind_to.sin_family = AF_INET;
      bind_to.sin_port = htons(requested_port);
      if (inet_pton(AF_INET, address, &bind_to.sin_addr) != 1)
        throw std::runtime_error("invalid MJPEG bind address");
      if (bind(listener, reinterpret_cast<sockaddr *>(&bind_to), sizeof(bind_to)) < 0)
        Fail("bind");
      if (listen(listener, static_cast<int>(kMaxClients)) < 0) Fail("listen");
      socklen_t size = sizeof(bind_to);
      if (getsockname(listener, reinterpret_cast<sockaddr *>(&bind_to), &size) < 0)
        Fail("getsockname");
      port = ntohs(bind_to.sin_port);
      int pipe_fds[2];
      if (pipe2(pipe_fds, O_NONBLOCK | O_CLOEXEC) < 0) Fail("pipe2");
      wake_read = pipe_fds[0]; wake_write = pipe_fds[1];
      running = true;
      encoder = std::thread([this] { Encode(); });
      network = std::thread([this] { Serve(); });
    } catch (...) {
      Stop();
      throw;
    }
  }
  ~Impl() { Stop(); }
  [[noreturn]] static void Fail(const char *operation) {
    throw std::runtime_error(std::string("MJPEG ") + operation + ": " + std::strerror(errno));
  }
  void Wake() noexcept {
    const unsigned char byte = 1;
    // EAGAIN means a wakeup is already pending. No retry or blocking write.
    if (wake_write >= 0) {
      const auto written = write(wake_write, &byte, 1);
      (void)written;
    }
  }
  void Stop() noexcept {
    // Set the predicate while holding the condition-variable mutex so a worker
    // cannot miss the shutdown notification between checking and sleeping.
    {
      std::lock_guard<std::mutex> lock(frame_mutex);
      stopping = true;
    }
    condition.notify_one();
    Wake();
    if (encoder.joinable()) encoder.join();
    if (network.joinable()) network.join();
    for (int fd : {listener, wake_read, wake_write}) if (fd >= 0) close(fd);
    listener = wake_read = wake_write = -1;
  }
  void Encode() noexcept {
    auto next = Clock::time_point{};
    try {
      while (!stopping) {
        cv::Mat image;
        std::uint64_t frame_id{};
        {
          std::unique_lock<std::mutex> lock(frame_mutex);
          condition.wait(lock, [this] { return stopping || !pending.empty(); });
          if (stopping) break;
          condition.wait_until(lock, next, [this] { return stopping.load(); });
          if (stopping) break;
          image = std::move(pending);
          frame_id = pending_id;
          pending.release();
        }
        if (subscribers == 0) continue;
        const auto start = Clock::now();
        next = start + kPeriod;
        try {
          std::vector<unsigned char> jpeg;
          if (!cv::imencode(".jpg", image, jpeg, {cv::IMWRITE_JPEG_QUALITY, 40})) {
            ++errors;
            continue;
          }
          const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
              Clock::now() - start).count();
          auto packet = std::make_shared<Packet>();
          packet->sequence = encoded.load() + 1;
          const auto header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
              std::to_string(jpeg.size()) +
              (interactive ? "\r\nX-Frame-Id: " + std::to_string(frame_id) : "") + "\r\n\r\n";
          packet->bytes.reserve(header.size() + jpeg.size() + 2);
          packet->bytes.insert(packet->bytes.end(), header.begin(), header.end());
          packet->bytes.insert(packet->bytes.end(), jpeg.begin(), jpeg.end());
          packet->bytes.insert(packet->bytes.end(), {'\r', '\n'});
          {
            std::lock_guard<std::mutex> lock(jpeg_mutex);
            latest = std::move(packet);
          }
          encode_us = static_cast<std::uint64_t>(elapsed);
          total_encode_us += static_cast<std::uint64_t>(elapsed);
          bytes += jpeg.size();
          ++encoded;
          Wake();
        } catch (const std::exception &) { ++errors; }
      }
    } catch (...) {
      ++errors;
      running = false;
    }
  }

  Stats StatsSnapshot() const {
    Stats s;
    s.encoded_frames = encoded; s.sent_frames = sent; s.encoded_bytes = bytes;
    s.encode_errors = errors; s.dropped_submissions = drops; s.slow_disconnects = slow;
    s.last_encode_ms = encode_us.load() / 1000.0;
    s.mean_encode_ms = s.encoded_frames ? total_encode_us.load() / (1000.0 * s.encoded_frames) : 0;
    s.clients = subscribers;
    return s;
  }
  std::string StatsJson() const {
    const auto s = StatsSnapshot();
    return "{\"encoded_frames\":" + std::to_string(s.encoded_frames) +
        ",\"sent_frames\":" + std::to_string(s.sent_frames) +
        ",\"encoded_bytes\":" + std::to_string(s.encoded_bytes) +
        ",\"last_encode_ms\":" + std::to_string(s.last_encode_ms) +
        ",\"mean_encode_ms\":" + std::to_string(s.mean_encode_ms) +
        ",\"clients\":" + std::to_string(s.clients) +
        ",\"encode_errors\":" + std::to_string(s.encode_errors) +
        ",\"dropped_submissions\":" + std::to_string(s.dropped_submissions) +
        ",\"slow_disconnects\":" + std::to_string(s.slow_disconnects) + "}\n";
  }
  void Close(Client &client) noexcept {
    if (client.streaming) {
      // Do not leave an orphaned TCP queue retransmitting an obsolete JPEG
      // after a slow viewer is evicted or the node is stopped.
      linger discard{1, 0};
      if (client.fd >= 0)
        (void)setsockopt(client.fd, SOL_SOCKET, SO_LINGER, &discard, sizeof(discard));
      --subscribers;
      client.streaming = false;
    }
    if (client.fd >= 0) close(client.fd);
    client.fd = -1;
    client.packet.reset();
  }
  void Queue(Client &client, std::shared_ptr<const Packet> packet) {
    client.packet = std::move(packet);
    client.offset = 0;
    client.send_started = Clock::now();
  }
  void ReadRequest(Client &client) {
    char buffer[1024];
    // One bounded recv per poll cycle; a noisy peer cannot monopolize this loop.
    const auto count = recv(client.fd, buffer, sizeof(buffer), MSG_DONTWAIT);
    if (count == 0) { Close(client); return; }
    if (count < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) Close(client);
      return;
    }
    if (client.streaming || client.close_after_send) { Close(client); return; }
    client.request.append(buffer, static_cast<std::size_t>(count));
    if (client.request.size() > kMaxRequest) { Close(client); return; }
    if (client.request.find("\r\n\r\n") == std::string::npos) return;
    const auto line_end = client.request.find("\r\n");
    const auto line = client.request.substr(0, line_end);
    if (interactive && (line == "POST /input HTTP/1.1" || line == "POST /input HTTP/1.0")) {
      const auto header_end = client.request.find("\r\n\r\n");
      std::string headers = client.request.substr(0, header_end);
      std::transform(headers.begin(), headers.end(), headers.begin(),
          [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      const auto position = headers.find("\r\ncontent-length:");
      std::size_t length = 0;
      if (position != std::string::npos) {
        std::istringstream size(headers.substr(position + 17));
        size >> length;
      }
      if (position == std::string::npos || length == 0 || length > 128) { Close(client); return; }
      if (client.request.size() < header_end + 4 + length) return;
      Input input;
      std::string kind, extra;
      std::istringstream body(client.request.substr(header_end + 4, length));
      bool valid = static_cast<bool>(body >> kind >> input.frame_id) && input.frame_id > 0;
      if (kind == "click") {
        input.kind = Input::Kind::Click;
        valid = valid && static_cast<bool>(body >> input.button >> input.x >> input.y) &&
            (input.button == 0 || input.button == 2) && input.x >= 0 && input.y >= 0 &&
            input.x < 8192 && input.y < 8192;
      } else if (kind == "key") {
        input.kind = Input::Kind::Key;
        valid = valid && static_cast<bool>(body >> input.key) && input.key >= 0 && input.key <= 127;
      } else valid = false;
      if (body >> extra) valid = false;
      bool queued = false;
      if (valid) {
        std::unique_lock<std::mutex> lock(input_mutex, std::try_to_lock);
        if (lock.owns_lock() && inputs.size() < 32) { inputs.push_back(input); queued = true; }
      }
      client.close_after_send = true;
      Queue(client, Response("text/plain", queued ? "Queued\n" : "Input rejected\n",
          queued ? "202 Accepted" : valid ? "429 Too Many Requests" : "400 Bad Request"));
      client.request.clear();
      return;
    }
    if (line == "GET /stream.mjpg HTTP/1.1" || line == "GET /stream.mjpg HTTP/1.0") {
      client.streaming = true;
      // Never serve a JPEG cached from a previous viewing session.
      client.last_sequence = interactive ? 0 : encoded.load();
      ++subscribers;
      auto packet = std::make_shared<Packet>();
      packet->bytes.assign(kStreamHeader, kStreamHeader + sizeof(kStreamHeader) - 1);
      Queue(client, std::move(packet));
    } else {
      client.close_after_send = true;
      if (line == "GET / HTTP/1.1" || line == "GET / HTTP/1.0") {
        if (interactive) Queue(client, Response("text/html; charset=utf-8", kInteractiveDebugPage));
        else Queue(client, Response("text/html; charset=utf-8",
            "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
            "<title>Vision debug</title><style>body{margin:0;background:#111}"
            "img{display:block;max-width:100%;height:auto;margin:auto}</style>"
            "<img src='/stream.mjpg' alt='Waiting for a debug frame'>"));
      } else if (line == "GET /stats HTTP/1.1" || line == "GET /stats HTTP/1.0") {
        Queue(client, Response("application/json", StatsJson()));
      } else Queue(client, Response("text/plain", "Not found\n", "404 Not Found"));
    }
    client.request.clear();
  }
  void Write(Client &client) {
    const auto &data = client.packet->bytes;
    // Bound work per client/cycle as well as the kernel send buffer.
    const auto count = send(client.fd, data.data() + client.offset,
        std::min<std::size_t>(16384, data.size() - client.offset), MSG_DONTWAIT | MSG_NOSIGNAL);
    if (count < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) Close(client);
      return;
    }
    client.offset += static_cast<std::size_t>(count);
    if (client.offset != data.size()) return;
    if (client.streaming && client.stream_header_sent) {
      client.last_sequence = client.packet->sequence;
      ++sent;
    }
    client.packet.reset();
    if (client.close_after_send) Close(client);
    else {
      client.stream_header_sent = true;
      client.waiting_drain = true;
    }
  }
  void Serve() noexcept {
    std::vector<Client> clients;
    try {
      clients.reserve(kMaxClients);
      while (!stopping) {
        clients.erase(std::remove_if(clients.begin(), clients.end(),
            [](const Client &c) { return c.fd < 0; }), clients.end());
        std::shared_ptr<const Packet> jpeg;
        {
          std::lock_guard<std::mutex> lock(jpeg_mutex);
          jpeg = latest;
        }
        const auto now = Clock::now();
        std::vector<pollfd> events{{listener, POLLIN, 0}, {wake_read, POLLIN, 0}};
        for (auto &client : clients) {
          if ((!client.streaming && !client.close_after_send && now - client.accepted > kRequestDeadline) ||
              ((client.packet || client.waiting_drain) && now - client.send_started > kSendDeadline)) {
            if (client.streaming) ++slow;
            Close(client);
          }
          if (client.fd >= 0 && client.waiting_drain) {
            // send() completion only means the kernel accepted the bytes.
            // Wait for the previous response to drain (including unacked
            // bytes) before selecting the latest JPEG. This bounds TCP's
            // hidden frame backlog as well as our application mailbox.
            int queued_bytes = 0;
            if (ioctl(client.fd, SIOCOUTQ, &queued_bytes) < 0) Close(client);
            else if (queued_bytes == 0) client.waiting_drain = false;
          }
          if (client.fd >= 0 && client.streaming && client.stream_header_sent &&
              !client.packet && !client.waiting_drain && jpeg && jpeg->sequence > client.last_sequence)
            Queue(client, jpeg);
          events.push_back({client.fd, static_cast<short>(POLLIN | (client.packet ? POLLOUT : 0)), 0});
        }
        const int ready = poll(events.data(), events.size(), 100);
        if (ready < 0) {
          if (errno == EINTR) continue;
          break;
        }
        if (events[1].revents & POLLIN) {
          unsigned char buffer[128];
          // Bounded drain; the next poll handles any remaining wakeups.
          const auto drained = read(wake_read, buffer, sizeof(buffer));
          (void)drained;
        }
        // Existing clients first; newly accepted clients have no poll entry yet.
        for (std::size_t i = 0; i < clients.size(); ++i) {
          auto &client = clients[i];
          const auto revents = events[i + 2].revents;
          if (revents & (POLLERR | POLLHUP | POLLNVAL)) Close(client);
          if (client.fd >= 0 && (revents & POLLIN)) ReadRequest(client);
          if (client.fd >= 0 && client.packet && (revents & POLLOUT)) Write(client);
        }
        if (events[0].revents & (POLLERR | POLLHUP | POLLNVAL)) break;
        if (events[0].revents & POLLIN) {
          // Limit accept work under a connection flood.
          for (std::size_t i = 0; i < kMaxClients; ++i) {
            const int fd = accept4(listener, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (fd < 0) break;
            if (clients.size() >= kMaxClients) { close(fd); continue; }
            int send_buffer = 16384;
            if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &send_buffer, sizeof(send_buffer)) < 0) {
              close(fd); continue;
            }
            Client client;
            client.fd = fd;
            clients.push_back(std::move(client));
          }
        }
      }
    } catch (...) { ++errors; }
    for (auto &client : clients) Close(client);
    running = false;
  }
};

DebugMjpegServer::DebugMjpegServer(std::uint16_t port, const char *address, bool interactive)
    : impl_(std::make_unique<Impl>(port, address, interactive)) {}
DebugMjpegServer::~DebugMjpegServer() = default;
bool DebugMjpegServer::RequestFrame() {
  if (!impl_->running || impl_->subscribers == 0) return false;
  const auto now = Clock::now();
  if (now - impl_->last_request < kPeriod) return false;
  impl_->last_request = now;
  return true;
}
bool DebugMjpegServer::Submit(const cv::Mat &image, std::uint64_t frame_id) {
  if (!impl_->running || impl_->subscribers == 0 || image.empty()) return false;
  std::unique_lock<std::mutex> lock(impl_->frame_mutex, std::try_to_lock);
  if (!lock.owns_lock()) { ++impl_->drops; return false; }
  impl_->pending = image; // Shallow reference: no raw-image copy on the producer.
  impl_->pending_id = frame_id;
  lock.unlock();
  impl_->condition.notify_one();
  return true;
}
bool DebugMjpegServer::TakeInput(Input &input) {
  if (!impl_->interactive) return false;
  std::unique_lock<std::mutex> lock(impl_->input_mutex, std::try_to_lock);
  if (!lock.owns_lock() || impl_->inputs.empty()) return false;
  input = impl_->inputs.front();
  impl_->inputs.pop_front();
  return true;
}
DebugMjpegServer::Stats DebugMjpegServer::GetStats() const { return impl_->StatsSnapshot(); }
std::uint16_t DebugMjpegServer::Port() const { return impl_->port; }
}  // namespace vision
