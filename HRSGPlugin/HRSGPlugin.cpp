// ref: https://github.com/RyuYamamoto/ChoreonoidSample/blob/master/plugin/ComJacobianPlugin/ComJacobianPlugin.cpp
// ref: 

#include <cnoid/Plugin>  // Pluginクラスが定義。Pluginに必須。
#include <cnoid/MessageView>  // Message Viewに対応
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/TimeBar>
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/BodyLoader>
#include <cnoid/Link>

#include <fmt/format.h>

using namespace cnoid;

class HRSGPlugin : public Plugin  // Pluginのクラス名は最後が“Plugin”で終わる必要がある。
{
  // ScopedConnection time_connection;

  public:
    HRSGPlugin() : Plugin("HRSG") {  // コンストラクタ。他Pluginへの依存などはここに記す。
      require("Body");
    }

    virtual bool initialize() override {  // Pluginの初期化処理
      MessageView::instance()->putln("Hello World!");  // Message Viewにテキストを出力
      
      ToolBar* bar = new ToolBar("HRSGTest");
      bar->addButton("Inc")->sigClicked().connect(std::bind(&HRSGPlugin::onButtonClicked, this, 0.5));
      bar->addButton("Dec")->sigClicked().connect(std::bind(&HRSGPlugin::onButtonClicked, this, -0.5));
      addToolBar(bar);
/*
      time_connection = TimeBar::instance()->sigTimeChanged().connect(
        [this](double time){ return onTimeChanged(time); }
      );
*/
      return true;
    }
/* 周期が一定ではない（0.0~）ので、不採用。Threadを利用した周期的な処理が必要と思われる。ROSやOpenRTMは使いたくない（規模が大きいし手間だしこれだけのために入れるのは邪魔だし）。
    bool onTimeChanged(double time) {
      //if(std::floor(time*100) == time*100) {
      ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
      BodyPtr robot = bodyItems[0]->body();

      for(int j = 0; j < robot->numJoints(); ++j) {
        robot->joint(j)->q() += 0.1;
      }
      bodyItems[0]->notifyKinematicStateChange(true);

      MessageView::instance()->putln(fmt::format("time: {}", time));
      //}
      //else {
        //MessageView::instance()->putln(fmt::format("hogehogehoge: {}", time));
      //}
      return true;
    }
*/
    void onButtonClicked(double dq) {
      MessageView::instance()->putln(fmt::format("push Button!!!  dq: {}", dq));

      ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
      BodyPtr robot = bodyItems[0]->body();

      for(int j = 0; j < robot->numJoints(); ++j) {
        robot->joint(j)->q() += dq;
      }
      bodyItems[0]->notifyKinematicStateChange(true);

    }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(HRSGPlugin)  // Pluginエントリ。必須。
