#pragma once
#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <QTimer>
#include <QQuickWindow>
class InputProbe : public gz::gui::Plugin {
 Q_OBJECT
 public: void LoadConfig(const tinyxml2::XMLElement *) override;
 private:
 void Tick();void Key(int,bool);void Mouse(QEvent::Type,Qt::MouseButton,Qt::MouseButtons,int,int);
 QQuickWindow *window=nullptr;QTimer timer;int tick=0,phase=0;
 gz::transport::Node node;gz::transport::Node::Publisher pub;
};
