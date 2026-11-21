#include "DriveKeys.hh"
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <QKeyEvent>
#include <QQuickWindow>
#include <QQuickItem>
void DriveKeys::LoadConfig(const tinyxml2::XMLElement *) {
  pub=node.Advertise<gz::msgs::Twist>("/gui/cmd_vel");
  if (auto *main=gz::gui::App()->findChild<gz::gui::MainWindow *>())
    if (auto *window=main->QuickWindow()) window->installEventFilter(this);
  connect(&timer,&QTimer::timeout,this,[this](){if (!pressed.empty()) Publish();});
  timer.start(50);
}
void DriveKeys::Publish() {
  gz::msgs::Twist msg;
  msg.mutable_linear()->set_x(.3*(int(pressed.count(Qt::Key_W))-int(pressed.count(Qt::Key_S))));
  msg.mutable_angular()->set_z(.6*(int(pressed.count(Qt::Key_A))-int(pressed.count(Qt::Key_D))));
  pub.Publish(msg);
}
void DriveKeys::Stop() {if (!pressed.empty()) {pressed.clear();Publish();}}
bool DriveKeys::eventFilter(QObject *object,QEvent *event) {
  if (event->type()==QEvent::FocusOut || event->type()==QEvent::WindowDeactivate) Stop();
  if (event->type()!=QEvent::KeyPress && event->type()!=QEvent::KeyRelease)
    return QObject::eventFilter(object,event);
  auto *key=static_cast<QKeyEvent *>(event);
  int k=key->key();
  if (k!=Qt::Key_W && k!=Qt::Key_A && k!=Qt::Key_S && k!=Qt::Key_D && k!=Qt::Key_Space)
    return QObject::eventFilter(object,event);
  auto *window=qobject_cast<QQuickWindow *>(object);
  if (window && window->activeFocusItem() &&
      window->activeFocusItem()->flags().testFlag(QQuickItem::ItemAcceptsInputMethod)) {
    Stop();return QObject::eventFilter(object,event);
  }
  if (key->isAutoRepeat()) return true;
  if (k==Qt::Key_Space) {pressed.clear();Publish();return true;}
  if (event->type()==QEvent::KeyPress) pressed.insert(k);else pressed.erase(k);
  Publish();return true;
}
GZ_ADD_PLUGIN(DriveKeys,gz::gui::Plugin)
