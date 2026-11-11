#pragma once

#include "vision/debug_mjpeg_server.hpp"
#include <QApplication>
#include <QDoubleSpinBox>
#include <QImage>
#include <QKeyEvent>
#include <QLayout>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPointer>
#include <QSignalBlocker>
#include <QWheelEvent>
#include <QWidget>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <any>
#include <chrono>
#include <cctype>
#include <deque>
#include <functional>
#include <utility>

namespace vision {

// Called only on the Qt/ROS thread. HTTP workers enqueue the existing bounded
// Click/Key messages; they never touch widgets, images/depth or robot commands.
class QtWebView {
public:
  using Revision=std::function<std::uint64_t()>;
  using Click=std::function<bool(const std::any&,QWidget*,int,int)>;
  using Reject=std::function<void(const char*)>;

  QtWebView(QWidget& root,DebugMjpegServer& server):root_(root),server_(server) {}

  bool Present(const Revision& revision,const std::function<void()>& before_render,
               const std::function<std::any()>& capture) {
    if (!server_.RequestFrame()) return false;
    before_render();
    root_.ensurePolished();
    if (root_.layout()) root_.layout()->activate();
    QImage image(root_.size(),QImage::Format_RGB32);
    image.fill(Qt::black);
    root_.render(&image);
    const cv::Mat bgra(image.height(),image.width(),CV_8UC4,image.bits(),image.bytesPerLine());
    cv::Mat bgr;
    cv::cvtColor(bgra,bgr,cv::COLOR_BGRA2BGR);
    Snapshot snapshot{++next_id_,revision(),layout_revision_,capture()};
    if (!server_.Submit(bgr,snapshot.id)) return false;
    snapshots_.push_back(std::move(snapshot));
    while (snapshots_.size()>32) snapshots_.pop_front();
    return true;
  }

  void ProcessInputs(const Revision& revision,const Click& click,const Reject& reject) {
    DebugMjpegServer::Input input;
    for (unsigned n=0;n<32 && server_.TakeInput(input);++n) {
      const auto found=std::find_if(snapshots_.begin(),snapshots_.end(),
          [&](const auto& frame) { return frame.id==input.frame_id; });
      // Compare again for EVERY queued input: an earlier click in this same
      // batch may already have changed the target, parameters or motion state.
      if (found==snapshots_.end() || found->revision!=revision() ||
          found->layout_revision!=layout_revision_ ||
          std::chrono::steady_clock::now()-found->captured>std::chrono::seconds(3)) {
        reject("화면이 변경되었습니다. 최신 화면에서 다시 조작하세요.");
        continue;
      }
      if (input.kind==DebugMjpegServer::Input::Kind::Click) {
        const QPoint point(input.x,input.y);
        if (!root_.rect().contains(point)) continue;
        if (input.button==2) {
          // A right-button packet is the custom page's wheel-position marker.
          wheel_point_=point; wheel_frame_=input.frame_id;
          continue;
        }
        QWidget* target=root_.childAt(point);
        if (!target || !target->isEnabled()) continue;
        auto* next_spin=Spin(target);
        if (next_spin!=spin_.data()) FinishEditing();
        if (found->revision!=revision()) {
          reject("입력값이 반영되었습니다. 최신 화면에서 버튼을 다시 눌러 주세요.");
          continue;
        }
        if (click(found->context,target,input.x,input.y)) {
          editor_.clear(); spin_.clear(); ++layout_revision_;
          continue;
        }
        SelectEditor(target);
        const QPoint local=target->mapFrom(&root_,point);
        QMouseEvent press(QEvent::MouseButtonPress,QPointF(local),Qt::LeftButton,
                          Qt::LeftButton,Qt::NoModifier);
        QMouseEvent release(QEvent::MouseButtonRelease,QPointF(local),Qt::LeftButton,
                            Qt::NoButton,Qt::NoModifier);
        QApplication::sendEvent(target,&press);
        QApplication::sendEvent(target,&release);
        // Tabs, buttons, scrollbar tracks and splitter navigation consume their
        // displayed screen once. Editing keys may share one frame while typing.
        if (!editor_) ++layout_revision_;
      } else if (input.key==14 || input.key==16) {
        if (wheel_frame_!=input.frame_id) continue;
        QWidget* target=root_.childAt(wheel_point_);
        if (!target || !target->isEnabled()) continue;
        if (Spin(target)!=spin_.data()) FinishEditing();
        if (found->revision!=revision()) continue;
        SelectEditor(target);
        const QPoint local=target->mapFrom(&root_,wheel_point_);
        QWheelEvent wheel(QPointF(local),QPointF(target->mapToGlobal(local)),QPoint(),
                          QPoint(0,input.key==14 ? 120 : -120),Qt::NoButton,
                          Qt::NoModifier,Qt::NoScrollPhase,false);
        QApplication::sendEvent(target,&wheel);
        ++layout_revision_;
      } else {
        // Keyboard activation of a focused motion button is deliberately not
        // forwarded. Only editable numeric fields receive keys; actions require
        // an explicit click on the current enabled button.
        if (!editor_ || !editor_->isEnabled()) continue;
        if (input.key==9) { NextEditor(); continue; }
        int key=0;
        QString text;
        auto modifiers=Qt::NoModifier;
        switch (input.key) {
        case 1: key=Qt::Key_A; modifiers=Qt::ControlModifier; break;
        case 3: key=Qt::Key_C; modifiers=Qt::ControlModifier; break;
        case 8: key=Qt::Key_Backspace; break;
        case 11: key=Qt::Key_PageUp; break;
        case 12: key=Qt::Key_PageDown; break;
        case 13: key=Qt::Key_Return; break;
        case 27: key=Qt::Key_Escape; break;
        case 28: key=Qt::Key_Left; break;
        case 29: key=Qt::Key_Right; break;
        case 30: key=Qt::Key_Up; break;
        case 31: key=Qt::Key_Down; break;
        case 127: key=Qt::Key_Delete; break;
        default:
          if (input.key<32 || input.key>126) continue;
          key=std::toupper(static_cast<unsigned char>(input.key));
          text=QString(QChar(static_cast<ushort>(input.key)));
        }
        QKeyEvent press(QEvent::KeyPress,key,modifiers,text);
        QKeyEvent release(QEvent::KeyRelease,key,modifiers,text);
        QApplication::sendEvent(editor_,&press);
        if (editor_) QApplication::sendEvent(editor_,&release);
        if (input.key==13 && spin_) initial_value_=spin_->value();
      }
    }
  }

private:
  struct Snapshot {
    std::uint64_t id,revision,layout_revision;
    std::any context;
    std::chrono::steady_clock::time_point captured{std::chrono::steady_clock::now()};
  };
  static QDoubleSpinBox* Spin(QWidget* widget) {
    for (auto* parent=widget;parent;parent=parent->parentWidget())
      if (auto* spin=qobject_cast<QDoubleSpinBox*>(parent)) return spin;
    return nullptr;
  }
  void SelectEditor(QWidget* widget) {
    auto* spin=Spin(widget);
    auto* editor=spin ? spin->findChild<QLineEdit*>() : qobject_cast<QLineEdit*>(widget);
    if (spin!=spin_.data() || editor!=editor_.data()) {
      spin_=spin; editor_=editor;
      if (spin_) initial_value_=spin_->value();
    }
    if (editor_) editor_->setFocus(Qt::MouseFocusReason);
  }
  void FinishEditing() {
    if (spin_) {
      {
        const QSignalBlocker blocker(spin_);
        spin_->interpretText();
      }
      if (spin_->value()!=initial_value_)
        QMetaObject::invokeMethod(spin_,"editingFinished",Qt::DirectConnection);
    }
    if (editor_) editor_->clearFocus();
    editor_.clear(); spin_.clear();
  }
  void NextEditor() {
    QWidget* current=editor_.data();
    FinishEditing();
    if (!current) return;
    QWidget* next=current;
    for (unsigned n=0;n<512;++n) {
      next=next->nextInFocusChain();
      if (!next || next==current) break;
      if (qobject_cast<QLineEdit*>(next) && next->isEnabled() && next->isVisibleTo(&root_)) {
        SelectEditor(next);
        editor_->selectAll();
        break;
      }
    }
  }
  QWidget& root_;
  DebugMjpegServer& server_;
  std::deque<Snapshot> snapshots_;
  std::uint64_t next_id_{static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count())};
  std::uint64_t layout_revision_{0},wheel_frame_{0};
  QPoint wheel_point_;
  QPointer<QLineEdit> editor_;
  QPointer<QDoubleSpinBox> spin_;
  double initial_value_{};
};
} // namespace vision
