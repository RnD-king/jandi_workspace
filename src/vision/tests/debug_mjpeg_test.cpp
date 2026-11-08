#include "vision/debug_mjpeg_server.hpp"

#include <arpa/inet.h>
#include <dirent.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
void Check(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
void Wait(const std::function<bool()> &ready, const char *message) {
  const auto deadline = Clock::now() + std::chrono::seconds(4);
  while (!ready()) {
    Check(Clock::now() < deadline, message);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
struct Connection {
  int fd{-1};
  std::string buffered;
  explicit Connection(std::uint16_t port, bool small_receive_buffer = false) {
    fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    Check(fd >= 0, "client socket failed");
    if (small_receive_buffer) {
      int size = 1024;
      setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &size, sizeof(size));
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
      close(fd); fd=-1;
      throw std::runtime_error("client connect failed");
    }
  }
  ~Connection() { if (fd >= 0) close(fd); }
  void Send(const std::string &text) {
    Check(send(fd, text.data(), text.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(text.size()),
          "test request send failed");
  }
  void Receive() {
    pollfd event{fd, POLLIN, 0};
    Check(poll(&event, 1, 3000) > 0, "HTTP receive timed out");
    char block[16384];
    const auto n = recv(fd, block, sizeof(block), 0);
    Check(n > 0, "HTTP connection closed unexpectedly");
    buffered.append(block, static_cast<std::size_t>(n));
  }
  std::string Header() {
    std::size_t end;
    while ((end = buffered.find("\r\n\r\n")) == std::string::npos) Receive();
    auto header = buffered.substr(0, end + 4);
    buffered.erase(0, end + 4);
    return header;
  }
  std::vector<unsigned char> Frame() {
    const auto header = Header();
    Check(header.find("--frame\r\n") == 0, "invalid multipart boundary");
    const auto pos = header.find("Content-Length: ");
    Check(pos != std::string::npos, "missing JPEG content length");
    const auto size = std::stoul(header.substr(pos + 16));
    Check(size < 2000000, "unbounded JPEG size");
    while (buffered.size() < size + 2) Receive();
    std::vector<unsigned char> jpeg(buffered.begin(), buffered.begin() + size);
    Check(buffered.substr(size, 2) == "\r\n", "incomplete JPEG trailer");
    buffered.erase(0, size + 2);
    return jpeg;
  }
  void StartStream() {
    Send("GET /stream.mjpg HTTP/1.1\r\nHost: localhost\r\n\r\n");
    Check(Header().find("multipart/x-mixed-replace") != std::string::npos,
          "stream response is not MJPEG");
  }
};

void IdleAndMultipleViewers() {
  vision::DebugMjpegServer server(0, "127.0.0.1");
  Check(!server.RequestFrame(), "server rendered with no viewers");
  cv::Mat image(480,640,CV_8UC3,cv::Scalar(30,80,150));
  server.Submit(image);
  Check(server.GetStats().encoded_frames==0, "server encoded without viewers");
  {
    Connection page(server.Port());
    page.Send("GET / HTTP/1.1\r\nHost: localhost\r\n\r\n");
    Check(page.Header().find("200 OK")!=std::string::npos,"root page unavailable");
    while (page.buffered.find("/stream.mjpg")==std::string::npos) page.Receive();
    Check(!server.RequestFrame(),"HTML request incorrectly counted as a viewer");
  }
  {
    Connection a(server.Port()),b(server.Port());
    a.StartStream(); b.StartStream();
    Wait([&] { return server.GetStats().clients==2; },"viewers not registered");
    Check(server.RequestFrame(), "headless viewer did not request an image");
    Check(!server.RequestFrame(), "producer FPS limit failed");
    server.Submit(image);
    const auto first=a.Frame(),second=b.Frame();
    Check(first==second,"viewers did not share the same JPEG");
    std::vector<unsigned char> expected;
    cv::imencode(".jpg",image,expected,{cv::IMWRITE_JPEG_QUALITY,40});
    Check(first==expected,"JPEG quality or transmitted source changed");
    const auto decoded=cv::imdecode(first,cv::IMREAD_COLOR);
    Check(decoded.cols==640 && decoded.rows==480,"stream changed image dimensions");
    Wait([&] { return server.GetStats().sent_frames==2; },"viewers not served");
    Check(server.GetStats().encoded_frames==1,"encoded separately for each viewer");
    std::this_thread::sleep_for(std::chrono::milliseconds(160));
    Check(server.GetStats().encoded_frames==1 && server.GetStats().sent_frames==2,
          "repeated a frame without new input");

    // Flood the mailbox without producer gating to independently exercise the
    // encoder limiter and ensure it retains the final image, not a FIFO.
    const auto start=Clock::now();
    for (int i=0;i<10000;++i) server.Submit(image);
    cv::Mat final_image(480,640,CV_8UC3,cv::Scalar(170,90,20));
    server.Submit(final_image);
    bool got_final=false;
    for (int i=0;i<4 && !got_final;++i) {
      const auto ja=a.Frame(),jb=b.Frame();
      Check(ja==jb,"clients diverged while consuming the same frame");
      const auto pixels=cv::imdecode(ja,cv::IMREAD_COLOR);
      got_final=pixels.at<cv::Vec3b>(240,320)[0]>150;
    }
    Check(got_final,"latest submitted frame was lost behind stale work");
    const auto elapsed=std::chrono::duration<double>(Clock::now()-start).count();
    Check(server.GetStats().encoded_frames<=2+static_cast<std::uint64_t>(elapsed*15),
          "encoder exceeded 15 FPS");
    std::cout << "JPEG bytes=" << first.size()
              << " mean_encode_ms=" << server.GetStats().mean_encode_ms << '\n';
  }
  Wait([&] { return server.GetStats().clients==0; },"disconnect not detected");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const auto count=server.GetStats().encoded_frames;
  server.Submit(image);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  Check(!server.RequestFrame() && server.GetStats().encoded_frames==count,
        "encoding continued after all viewers disconnected");
  // The next viewer must wait for a new frame, not receive the cached JPEG.
  Connection reconnect(server.Port());
  reconnect.StartStream();
  pollfd event{reconnect.fd,POLLIN,0};
  Check(reconnect.buffered.empty() && poll(&event,1,100)==0,"replayed an old session frame");
  server.Submit(image);
  Check(!reconnect.Frame().empty(),"reconnect failed to receive a fresh frame");
  Connection stats(server.Port());
  stats.Send("GET /stats HTTP/1.1\r\nHost: localhost\r\n\r\n");
  Check(stats.Header().find("application/json")!=std::string::npos,"stats endpoint unavailable");
  while (stats.buffered.find("mean_encode_ms")==std::string::npos) stats.Receive();
}

void InteractiveInputIsOptInAndBounded() {
  using Server=vision::DebugMjpegServer;
  Server readonly(0,"127.0.0.1");
  const std::string body="click 17 0 123 234";
  auto post=[&](Server &server,const std::string &data,bool split=false) {
    Connection client(server.Port());
    const auto header="POST /input HTTP/1.1\r\nHost: localhost\r\nContent-Length: "+
        std::to_string(data.size())+"\r\n\r\n";
    client.Send(header+(split ? data.substr(0,3) : data));
    if(split) {std::this_thread::sleep_for(std::chrono::milliseconds(10));client.Send(data.substr(3));}
    return client.Header();
  };
  Check(post(readonly,body).find("404")!=std::string::npos,"competition server accepted input");
  Server::Input input;
  Check(!readonly.TakeInput(input),"read-only server produced an input");
  Server interactive(0,"127.0.0.1",true);
  Check(post(interactive,body,true).find("202")!=std::string::npos,"split POST rejected");
  Check(interactive.TakeInput(input) && input.frame_id==17 && input.x==123 && input.y==234,
        "input coordinates/frame association changed");
  Check(!interactive.TakeInput(input),"input repeated without a new request");
  Check(post(interactive,"key 17 3").find("202")!=std::string::npos,"key POST rejected");
  Check(interactive.TakeInput(input) && input.kind==Server::Input::Kind::Key && input.key==3,
        "Ctrl+C input was not preserved");
  Check(post(interactive,"click 17 0 -1 5").find("400")!=std::string::npos,"invalid input accepted");
  for(unsigned n=0;n<32;++n) Check(post(interactive,body).find("202")!=std::string::npos,"bounded queue failed early");
  Check(post(interactive,body).find("429")!=std::string::npos,"input queue was unbounded");
  for(unsigned n=0;n<32;++n) Check(interactive.TakeInput(input),"queued input lost");
  Check(!interactive.TakeInput(input),"extra queued input");
  Connection viewer(interactive.Port());viewer.StartStream();
  Wait([&]{return interactive.RequestFrame();},"interactive viewer not registered");
  Check(interactive.Submit(cv::Mat(480,640,CV_8UC3,cv::Scalar(20,40,80)),987),"frame handoff failed");
  auto header=viewer.Header();
  Check(header.find("X-Frame-Id: 987")!=std::string::npos,"JPEG missing original frame ID");
}

void SlowViewerAndShutdown() {
  auto server=std::make_unique<vision::DebugMjpegServer>(0,"127.0.0.1");
  Connection slow(server->Port(),true),healthy(server->Port()),partial(server->Port());
  slow.Send("GET /stream.mjpg HTTP/1.1\r\nHost: localhost\r\n\r\n");
  healthy.StartStream();
  partial.Send("GET / HTTP/1.1\r\nHost:");
  Wait([&] { return server->GetStats().clients==2; },"slow test viewer not connected");
  cv::Mat noise(480,640,CV_8UC3);
  cv::RNG(123).fill(noise,cv::RNG::UNIFORM,0,256);
  const auto start=Clock::now();
  unsigned received=0;
  while (Clock::now()-start<std::chrono::milliseconds(1600)) {
    if (server->RequestFrame()) {
      server->Submit(noise);
      Check(!healthy.Frame().empty(),"slow peer blocked healthy viewer");
      ++received;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  Check(received>=10,"healthy viewer stopped progressing");
  Check(server->GetStats().slow_disconnects>=1 && server->GetStats().clients==1,
        "slow peer was not bounded by a send deadline");
  const auto elapsed=std::chrono::duration<double>(Clock::now()-start).count();
  Check(received<=1+static_cast<unsigned>(elapsed*15),"stream exceeded 15 FPS");
  std::cout << "healthy_fps=" << received/elapsed << " noisy JPEG mean_bytes="
            << server->GetStats().encoded_bytes/server->GetStats().encoded_frames
            << " mean_encode_ms=" << server->GetStats().mean_encode_ms << '\n';
  const auto stopping=Clock::now();
  server.reset(); // Active stream plus incomplete HTTP request; neither may hang join.
  Check(Clock::now()-stopping<std::chrono::milliseconds(700),"server shutdown blocked");
}
void PortConflictAndLifetime() {
  vision::DebugMjpegServer owner(0,"127.0.0.1");
  const auto descriptors=[] {
    DIR *directory=opendir("/proc/self/fd");
    Check(directory!=nullptr,"cannot inspect test file descriptors");
    unsigned count=0;
    while (readdir(directory)) ++count;
    closedir(directory);
    return count;
  };
  const auto before=descriptors();
  bool rejected=false;
  try { vision::DebugMjpegServer conflict(owner.Port(),"127.0.0.1"); }
  catch(const std::runtime_error &) { rejected=true; }
  Check(rejected,"port conflict was silently accepted");
  for (int i=0;i<15;++i) { vision::DebugMjpegServer temporary(0,"127.0.0.1"); }
  Check(descriptors()==before,"server lifetime or failed bind leaked a descriptor");
}
void KernelBacklogDeadline() {
  vision::DebugMjpegServer server(0,"127.0.0.1");
  Connection stalled(server.Port(),true),healthy(server.Port());
  stalled.StartStream(); healthy.StartStream();
  Wait([&] { return server.GetStats().clients==2; },"kernel backlog viewers not registered");
  // This JPEG fits in SO_SNDBUF but exceeds the stalled receiver's window.
  // send() can finish while bytes remain queued/unacknowledged in TCP.
  const cv::Mat image(480,640,CV_8UC3,cv::Scalar(30,80,150));
  server.Submit(image);
  Check(!healthy.Frame().empty(),"healthy viewer did not receive the image");
  Wait([&] { return server.GetStats().sent_frames==2; },"small JPEG was not accepted by TCP");
  Wait([&] { return server.GetStats().slow_disconnects==1; },"undrained TCP bytes escaped the deadline");
  Check(server.GetStats().clients==1 && server.GetStats().encoded_frames==1,
        "kernel backlog handling affected a healthy viewer or replayed a frame");
}
}  // namespace
int main() {
  try {
    IdleAndMultipleViewers();
    InteractiveInputIsOptInAndBounded();
    SlowViewerAndShutdown();
    KernelBacklogDeadline();
    PortConflictAndLifetime();
    std::cout << "MJPEG headless transport tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
