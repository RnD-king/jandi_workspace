#include "vision/qt_web_view.hpp"
#include "vision/qt_web_page.hpp"
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QVBoxLayout>
#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock=std::chrono::steady_clock;
void Check(bool ok,const char* reason) { if (!ok) throw std::runtime_error(reason); }
template<class Predicate> void Wait(Predicate ready) {
  const auto deadline=Clock::now()+std::chrono::seconds(4);
  while (!ready()) {
    Check(Clock::now()<deadline,"HTTP/Qt test timed out");
    QApplication::processEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
struct Connection {
  int fd{-1};
  std::string buffered;
  explicit Connection(std::uint16_t port) {
    fd=socket(AF_INET,SOCK_STREAM|SOCK_CLOEXEC,0);
    Check(fd>=0,"socket failed");
    sockaddr_in address{}; address.sin_family=AF_INET;
    address.sin_addr.s_addr=htonl(INADDR_LOOPBACK); address.sin_port=htons(port);
    Check(connect(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0,"connect failed");
  }
  ~Connection() { if (fd>=0) close(fd); }
  void Send(const std::string& data) {
    std::size_t offset=0;
    while (offset<data.size()) {
      const auto n=send(fd,data.data()+offset,data.size()-offset,MSG_NOSIGNAL);
      Check(n>0,"send failed"); offset+=static_cast<std::size_t>(n);
    }
  }
  void Receive() {
    pollfd event{fd,POLLIN,0};
    Check(poll(&event,1,3000)>0,"response timed out");
    char block[16384]; const auto n=recv(fd,block,sizeof(block),0);
    Check(n>0,"response closed"); buffered.append(block,static_cast<std::size_t>(n));
  }
  std::string Header() {
    std::size_t end;
    while ((end=buffered.find("\r\n\r\n"))==std::string::npos) Receive();
    auto result=buffered.substr(0,end+4); buffered.erase(0,end+4); return result;
  }
  std::uint64_t Frame() {
    const auto header=Header();
    const auto id_pos=header.find("X-Frame-Id: ");
    const auto size_pos=header.find("Content-Length: ");
    Check(id_pos!=std::string::npos && size_pos!=std::string::npos,"frame metadata missing");
    const auto size=std::stoul(header.substr(size_pos+16));
    while (buffered.size()<size+2) Receive();
    buffered.erase(0,size+2);
    return std::stoull(header.substr(id_pos+12));
  }
};
void Input(std::uint16_t port,const std::string& text) {
  Connection request(port);
  request.Send("POST /input HTTP/1.1\r\nHost: localhost\r\nContent-Length: "+
      std::to_string(text.size())+"\r\n\r\n"+text);
  Check(request.Header().find("202 Accepted")!=std::string::npos,"input was not queued");
}
QPoint Center(QWidget& root,QWidget& widget) {
  return widget.mapTo(&root,widget.rect().center());
}
std::string Click(std::uint64_t id,QPoint p,int button=0) {
  return "click "+std::to_string(id)+" "+std::to_string(button)+" "+
      std::to_string(p.x())+" "+std::to_string(p.y());
}
void Bench() {
  QWidget root;
  root.resize(900,400);
  auto* layout=new QHBoxLayout(&root);
  auto* button=new QPushButton("Calculate"); layout->addWidget(button);
  auto* spin=new QDoubleSpinBox; spin->setRange(-10,10);
  spin->setDecimals(4); spin->setKeyboardTracking(false); layout->addWidget(spin);
  auto* image=new QWidget; image->setMinimumSize(160,100); layout->addWidget(image);
  auto* scroll=new QScrollArea; auto* content=new QWidget;
  auto* rows=new QVBoxLayout(content);
  for (int i=0;i<40;++i) rows->addWidget(new QLabel(QString::number(i)));
  scroll->setWidget(content); scroll->setWidgetResizable(true); layout->addWidget(scroll);
  // Enable normal Qt layout/focus while keeping every desktop surface offscreen.
  root.setAttribute(Qt::WA_DontShowOnScreen);
  root.show();
  QApplication::processEvents();
  vision::DebugMjpegServer server(0,"127.0.0.1",true,vision::kQtWebPage);
  vision::QtWebView web(root,server);
  {
    Connection page(server.Port());
    page.Send("GET / HTTP/1.1\r\nHost: localhost\r\n\r\n");
    const auto header=page.Header();
    Check(header.find("200 OK")!=std::string::npos,"GUI page unavailable");
    const auto position=header.find("Content-Length: ");
    Check(position!=std::string::npos,"GUI page size missing");
    const auto size=std::stoul(header.substr(position+16));
    while (page.buffered.size()<size) page.Receive();
    Check(page.buffered.find("<title>Shoot tuning</title>")!=std::string::npos,
          "server returned the camera page instead of the requested GUI page");
  }
  Connection stream(server.Port());
  stream.Send("GET /stream.mjpg HTTP/1.1\r\nHost: localhost\r\n\r\n");
  Check(stream.Header().find("200 OK")!=std::string::npos,"stream failed");
  std::uint64_t revision=1;
  int clicks=0,rejections=0,current_depth=123,clicked_depth=0,edits=0;
  QObject::connect(button,&QPushButton::clicked,[&]() { ++clicks; });
  QObject::connect(spin,&QDoubleSpinBox::editingFinished,[&]() { ++edits; ++revision; });
  const auto get_revision=[&]() { return revision; };
  const auto dispatch=[&]() {
    web.ProcessInputs(get_revision,[&](const std::any& context,QWidget* target,int,int) {
      if (target!=image) return false;
      clicked_depth=std::any_cast<int>(context); return true;
    },[&](const char*) { ++rejections; });
  };
  const auto publish=[&]() {
    Wait([&]() { return web.Present(get_revision,[]() {},[&]() -> std::any { return current_depth; }); });
    return stream.Frame();
  };
  auto id=publish();
  Input(server.Port(),Click(id,Center(root,*button)));
  Input(server.Port(),Click(id,Center(root,*button)));
  Check(clicks==0,"network worker executed a GUI/robot callback");
  dispatch();
  Check(clicks==1 && rejections>=1,"stale double click was executed twice");
  id=publish(); current_depth=999;
  Input(server.Port(),Click(id,Center(root,*image))); dispatch();
  Check(clicked_depth==123,"click used newer data instead of the displayed frame context");
  id=publish();
  auto* editor=spin->findChild<QLineEdit*>(); Check(editor,"spin editor missing");
  Input(server.Port(),Click(id,Center(root,*editor))); dispatch();
  for (int key : {1,45,49,46,50,53,13})
    Input(server.Port(),"key "+std::to_string(id)+" "+std::to_string(key));
  dispatch();
  Check(std::abs(spin->value()+1.25)<1e-9 && edits>0,"numeric Ctrl+A/type/Enter did not apply");
  id=publish(); ++revision;
  Input(server.Port(),Click(id,Center(root,*button))); dispatch();
  Check(clicks==1,"external state change did not reject a stale action click");
  id=publish();
  Input(server.Port(),Click(id,Center(root,*scroll->viewport()),2));
  Input(server.Port(),"key "+std::to_string(id)+" 16"); dispatch();
  Check(scroll->verticalScrollBar()->value()>0,"remote wheel did not scroll the GUI");
  const auto before=spin->value();
  id=publish(); Input(server.Port(),Click(id,Center(root,*button))); dispatch();
  Check(spin->value()==before,"GUI action unexpectedly edited numeric data");
  id=publish();
  const int completed=clicks;
  std::this_thread::sleep_for(std::chrono::milliseconds(3100));
  Input(server.Port(),Click(id,Center(root,*button))); dispatch();
  Check(clicks==completed,"expired reconnect frame executed a GUI action");
}
}
int main(int argc,char** argv) {
  QApplication app(argc,argv);
  Bench();
}
