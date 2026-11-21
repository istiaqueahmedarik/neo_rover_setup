#include "InputProbe.hh"
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/int32.pb.h>
#include <gz/plugin/Register.hh>
#include <QCoreApplication>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QFocusEvent>
void InputProbe::LoadConfig(const tinyxml2::XMLElement *) {
 if(auto *m=gz::gui::App()->findChild<gz::gui::MainWindow *>())window=m->QuickWindow();
 pub=node.Advertise<gz::msgs::Int32>("/gui/input_probe");
 connect(&timer,&QTimer::timeout,this,[this](){Tick();});timer.start(50);
}
void InputProbe::Key(int key,bool down){QKeyEvent e(down?QEvent::KeyPress:QEvent::KeyRelease,key,Qt::NoModifier);QCoreApplication::sendEvent(window,&e);}
void InputProbe::Mouse(QEvent::Type type,Qt::MouseButton button,Qt::MouseButtons buttons,int x,int y){
 auto global=window->mapToGlobal(QPoint(x,y));QMouseEvent e(type,QPointF(x,y),QPointF(global),button,buttons,Qt::NoModifier);QCoreApplication::sendEvent(window,&e);
}
void InputProbe::Tick(){
 if(!window)return;++tick;
 if(tick==100){QFocusEvent e(QEvent::FocusIn);QCoreApplication::sendEvent(window,&e);phase=10;Key(Qt::Key_W,true);}
 if(tick==150){Key(Qt::Key_W,false);phase=11;}
 if(tick==180){phase=12;Key(Qt::Key_A,true);}
 if(tick==220){Key(Qt::Key_A,false);phase=13;}
 if(tick==240){phase=14;Key(Qt::Key_W,true);}
 if(tick==260){QFocusEvent e(QEvent::FocusOut);QCoreApplication::sendEvent(window,&e);phase=15;}
 if(tick==280)Key(Qt::Key_W,false);
 if(tick==300){phase=16;Mouse(QEvent::MouseButtonPress,Qt::MiddleButton,Qt::MiddleButton,300,300);}
 if(tick>300 && tick<=316)Mouse(QEvent::MouseMove,Qt::NoButton,Qt::MiddleButton,300+8*(tick-300),300+3*(tick-300));
 if(tick==317)Mouse(QEvent::MouseButtonRelease,Qt::MiddleButton,Qt::NoButton,428,348);
 if(tick==330)phase=17;
 if(tick==350){phase=18;Mouse(QEvent::MouseButtonPress,Qt::LeftButton,Qt::LeftButton,300,300);}
 if(tick>350 && tick<=366)Mouse(QEvent::MouseMove,Qt::NoButton,Qt::LeftButton,300+5*(tick-350),300);
 if(tick==367)Mouse(QEvent::MouseButtonRelease,Qt::LeftButton,Qt::NoButton,380,300);
 if(tick==380)phase=19;
 if(tick==390)Mouse(QEvent::MouseButtonPress,Qt::RightButton,Qt::RightButton,300,300);
 if(tick>390 && tick<=406)Mouse(QEvent::MouseMove,Qt::NoButton,Qt::RightButton,300,300+5*(tick-390));
 if(tick==407)Mouse(QEvent::MouseButtonRelease,Qt::RightButton,Qt::NoButton,300,380);
 if(tick==420){phase=20;timer.stop();}
 gz::msgs::Int32 m;m.set_data(phase);pub.Publish(m);
}
GZ_ADD_PLUGIN(InputProbe,gz::gui::Plugin)
