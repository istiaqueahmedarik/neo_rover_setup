#pragma once
#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <QTimer>
#include <set>
class DriveKeys : public gz::gui::Plugin {
  Q_OBJECT
 public:
  void LoadConfig(const tinyxml2::XMLElement *) override;
 protected:
  bool eventFilter(QObject *, QEvent *) override;
 private:
  void Publish();
  void Stop();
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  QTimer timer;
  std::set<int> pressed;
};
